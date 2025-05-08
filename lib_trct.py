import machine
from machine import Pin, ADC
from time import sleep



def init(index):
    pin_obj = [0] * 5 
    for i in range (0,5):
        pin_obj[i]= ADC(Pin(index[i] ))
        pin_obj[i].atten(ADC.ATTN_11DB) # initialise le pin en entrée analogique de 0 = 0 à 3.3V = 4095
    return pin_obj

def print_(pin_obj):
    for i in range (0,5):
        print(pin_obj[i].read()*3.3/4095)
    print()
    sleep(0.1) 