from dali.bus import Bus
from dali.driver.launchpad_lw14 import LAUNCHPAD_LED_WARRIOR_14
from dali.gear.general import *
from dali.address import Broadcast


#if __name__=="__main__":
llw = LAUNCHPAD_LED_WARRIOR_14(port='/dev/ttyACM0')
bus = Bus(interface=llw)
#bus.assign_short_addresses(reset=True)
