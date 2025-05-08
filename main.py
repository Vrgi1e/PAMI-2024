from machine import Pin, ADC, SoftI2C, I2C
from lcd_api import LcdApi
from i2c_lcd import I2cLcd
from time import sleep
from encoder_N20_esp import Motor,PID
from drive import DRIVE
import vl53l0x
import lib_trct

#i2c
i2c=SoftI2C(scl=Pin(22),sda=Pin(21), freq=40000)
tof=vl53l0x.VL53L0X(i2c)
tof.start()

lcd = I2cLcd(i2c, 0x27, 4, 20)
lcd.putstr("Hello World")

#moteurs
motor_left = Motor(16, 17, 10, 27, 14) # Motor(M1, M2, EN, C1, C2, frequency) Default frequency is set at 50Hz
motor_right = Motor(4, 2, 10, 25, 26) # Motor(M1, M2, EN, C1, C2, frequency) Default frequency is set at 50Hz

#capteur ir
pin_ir = lib_trct.init([36,39,34,35,32]) # se lit avec pin_ir[n].read()

frequency = 5000
buzzer = PWM(Pin(5), frequency)