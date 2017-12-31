
from dali.driver.base import SyncDALIDriver
from dali.frame import BackwardFrame
from dali.frame import ForwardFrame
import time
import serial
import logging
import random
from dali.gear.general import Compare
from dali.command import YesNoResponse


#coloredlogs.install(level='DEBUG',fmt='%(asctime)s %(levelname)s %(message)s')

log_format = '%(levelname)s: %(message)s'
logging.basicConfig(format=log_format, level=logging.INFO)
logger = logging.getLogger("LW14")



STATUS_BUS_ERR = 1<<7
STATUS_BUSY = 1<<6
STATUS_OVERRUN =1<<5
STATUS_FRAME_ERROR =1<<4
STATUS_VALID_REPLY =1<<3
STATUS_TIMEFRAME = 1<<2
STATUS_2BYTE_RCV = 1<<1
STATUS_1BYTE_RCV = 1<<0


START = 0x7C #124 |
STOP = 0x7D  #125 }
ESC = 0x7E   #126 ~
#https://eli.thegreenplace.net/2009/08/12/framing-in-serial-communications/

def _decode_status(s):
    s_d={}
    s_d["BUSY"]=(s & STATUS_BUSY)>0
    s_d["BUS_ERR"] = (s & STATUS_BUS_ERR)>0
    s_d["OVERRUN"] = (s & STATUS_OVERRUN)>0
    s_d["FRAME_ERROR"] = (s & STATUS_FRAME_ERROR)>0
    s_d["VALID_REPLY"] = (s & STATUS_VALID_REPLY)>0
    s_d["TIMEFRAME"] = (s & STATUS_TIMEFRAME)>0
    s_d["RCV_2BYTE"] = (s & STATUS_2BYTE_RCV)>0
    s_d["RCV_1BYTE"] = (s & STATUS_1BYTE_RCV)>0
    return s_d

#MSG HEADERS
HEADER ={
    "DALI_COMMANDS" :1,
    "DALI_RCV_1BYTES": 2,
    "DALI_RCV_2BYTES":3,
    "TIMEFRAME_EXPIRED" : 10,
    "INVALID_DALI_FRAME":11,
    "ECHO":30,
}

def header_from_str(header):
    return HEADER.get(header, None)


def header_from_int(header):
    d={v:k for k,v in HEADER.items()}
    return d.get(header, "UNKNOW_HEADER")

def invalid_frame_return(command):
    if isinstance(command,Compare):
        r=YesNoResponse(None)
        r._value = True
        return r
    else:
        return "INVALID_REPLY"



class LAUNCHPAD_LED_WARRIOR_14(SyncDALIDriver):

    def __init__(self,dev=None,port='/dev/ttyACM0'):
        # add ch to logger
        self.log = logger
        self.log.info("Init Serial")
        #
        if dev is not None:
            self.ser=dev
        else:
            self.ser = serial.Serial(port=port, timeout=0.01)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.status=None
        r = random.randint(0,255)
        self.log.debug("echo_byte: {}".format(hex(r)))
        assert r==self.serial_echo(r)

    def _write_ser_frame(self, frame):
        stuffed = bytearray()
        stuffed.append(START)
        # escape
        for i in frame:
            if i in [START, STOP, ESC]:
                stuffed.append(ESC)
                stuffed.append(i)
            else:
                stuffed.append(i)
        stuffed.append(STOP)
        #write to serial
        self.ser.write(stuffed)

    def _read_ser_frame(self, waitCycles=60):

        bytesRead=0
        count=0
        partial_frame = None
        esc = False
        while  (bytesRead<40) and (count<waitCycles):
            if self.ser.inWaiting():
                byte=int.from_bytes(self.ser.read(),'little')
                bytesRead += 1
                if isinstance(partial_frame,bytearray):
                        if esc:
                            esc=0
                            partial_frame.append(byte)
                        elif byte == ESC:
                            esc=True
                        elif byte==STOP:
                            return partial_frame,bytesRead
                        else:
                            partial_frame.append(byte)
                else:
                    if esc:
                        esc = 0
                    elif byte == ESC:
                        esc = True
                    elif byte == START:
                        partial_frame = bytearray()
            else:
                time.sleep(0.001)
                count+=1
        else:
            self.log.warning("_read_serial_frame return None. Wait cycles:{}/{}. Bytes read: {}/40. partial Frame:  {}".format(count,waitCycles,bytesRead,partial_frame))
            return None,bytesRead

    def _decode(self,frame, expected_header=None):

        header = header_from_int(frame[0])
        data = frame[1:]
        d = {'header': header}

        if header == "TIMEFRAME_EXPIRED":
            d['status'] = data[0]
            d['status_d'] = _decode_status(data[0])
            d['status_b'] = format(data[0], '#010b')

        elif header == "DALI_RCV_1BYTES":
            d['status'] = data[0]
            d['status_d'] = _decode_status(data[0])
            d['status_b'] = format(data[0], '#010b')
            d['dali_frame'] = BackwardFrame(int.from_bytes(data[1:2],'little'))

        elif header == "DALI_RCV_2BYTES":
            d['status'] = data[0]
            d['status_d'] = _decode_status(data[0])
            d['status_b'] = format(data[0], '#010b')
            d['dali_frame'] = ForwardFrame(16,[int.from_bytes(data[1:2],'little'),int.from_bytes(data[2:3],'little')])
        elif header == "ECHO":
            d['echo_byte']=data[0]

        elif header == "INVALID_DALI_FRAME":
            d['status'] = data[0]
            d['status_d'] = _decode_status(data[0])
            d['status_b'] = format(data[0], '#010b')

        self.log.debug('read serial frame:{}, decoded frame: {}.'.format(frame,d))
        return d

    def serial_echo(self,echo_byte):
        outFrame= bytearray([header_from_str("ECHO"),echo_byte])
        self._write_ser_frame(outFrame)
        inFrame, _ = self._read_ser_frame()
        inMsg = self._decode(inFrame, ["ECHO"])
        return inMsg['echo_byte']

    def construct(self, command,priority,delay):
        ad,cm = command.frame.as_byte_sequence
        data= bytearray([header_from_str("DALI_COMMANDS")])
        data.extend([ad,cm,priority,delay])
        if command.is_config:
            data.extend([ad, cm, 0, delay])
        self.log.debug('write serial command {}, frame: {}.'.format(str(command), data))
        return data

    def send(self, command,priority=0, delay=0):
        # send DALI command
        serialFrameOut=self.construct(command, priority,delay)
        self._write_ser_frame(serialFrameOut)
        #
        t0 =time.clock()
        timeout=0.02
        reply=None
        invalid_frame_flag = 0
        timeframe_expired_flag = 0
        while time.clock()-t0<timeout:
            inFrame, nbytes = self._read_ser_frame()
            #
            if inFrame is not None:
                inMsg=self._decode(inFrame, ["TIMEFRAME_EXPIRED", "INVALID_DALI_FRAME","DALI_RCV_1BYTES"])
                self.log.debug("in Frame: {}".format(inMsg))
                #
                if inMsg['header']=="TIMEFRAME_EXPIRED":
                    if inMsg['status_d']['VALID_REPLY'] and (not invalid_frame_flag):
                        timeout=0.04
                    elif invalid_frame_flag :
                        self.log.debug("return: INVALID_REPLY ")
                        return invalid_frame_return(command)
                    else:
                        frame = None
                        break;

                elif inMsg['header']=="INVALID_DALI_FRAME":
                    invalid_frame_flag=1

                elif inMsg['header']=="DALI_RCV_1BYTES":
                    frame=inMsg['dali_frame']
                    break;
        else:
            raise TimeoutError('no rcv frame')

        if command.is_query:
            return command.response(frame)
        else:
            return frame



