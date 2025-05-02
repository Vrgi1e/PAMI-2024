import machine
from machine import Pin, ADC, time_pulse_us,disable_irq, enable_irq
from time import sleep
import lib_trct
from machine import SoftI2C,I2C
import vl53l0x

i2c=SoftI2C(scl=Pin(22),sda=Pin(21), freq=40000,timeout=50000)
tof=vl53l0x.VL53L0X(i2c)
tof._calibrate(1)
tof.start()

while True:
    print(tof.read())
''' 
def init_(index):
    obj = {
    "out":Pin(index["out"], Pin.IN),
    "S2":Pin(index["S2"], Pin.OUT),
    "S3":Pin(index["S3"], Pin.OUT)
    }
    return obj
def get_frequency(pin):
    sum = 0
    for i in range (10):
        sum = sum +time_pulse_us(p, 0, 1000000)
    return sum/i

def get_red(obj):
    obj["S2"].value(0)
    obj["S3"].value(0)
    return get_frequency(obj["out"])

def get_blue(obj):
    obj["S2"].value(0)
    obj["S3"].value(1)
    return get_frequency(obj["out"])

def get_green(obj):
    obj["S2"].value(1)
    obj["S3"].value(1)
    return get_frequency(obj["out"])

def get_color(obj):
    return {get_red(obj),get_blue(obj),get_green(obj)}

seuil_trct = 3000 #seuil on/off des pins 
trct_index = [34,35,32,33,25] #table des numéro de pin gpio
trct_obj = lib_trct.init(trct_index) #table des pins en tant qu'objets pin

tcs_index = {
    "out" : 32,
    "S2": 33,
    "S3": 25}

tcs_obj = init_(tcs_index)
p = Pin(32, Pin.IN)
'''
'''
while True:
    state= disable_irq()
    time_pulse_us(p, 1, 1000000)
    sum += time_pulse_us(p, 1, 1000000)
    enable_irq(state)
    state= disable_irq()
    print(1000000/sum)
'''
'''
while True:

    sum = 0
    loops = 100
    for _ in range(loops//2):
        state= disable_irq()
        time_pulse_us(p, 1, 1000000)
        sum += time_pulse_us(p, 1, 1000000)
        enable_irq(state)
        state= disable_irq()
        time_pulse_us(p, 0, 1000000)
        sum += time_pulse_us(p, 0, 1000000)
        enable_irq(state)
    print(500000*loops/sum)
    sleep(0.97)
'''