#  Raspberry Pi Leader for Arduino Follower
#  Uses lgpio library, compatible with kernel 5.11
#  Connects to Arduino via I2C and periodically blinks an LED
#  Author: William 'jawn-smith' Wilson

import lgpio
import time

addr = 0x0 # bus address

h = lgpio.i2c_open(1, addr)
while True:
    try:
        lgpio.i2c_write_byte(h, 0x0) # switch it off
        time.sleep(1)
        lgpio.i2c_write_byte(h, 0x1) # switch it on
        time.sleep(1)
    except KeyboardInterrupt:
        lgpio.i2c_write_byte(h, 0x0) # switch it off
        lgpio.i2c_close(h)
        break

