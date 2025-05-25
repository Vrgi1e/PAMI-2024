from machine import Pin, ADC, SoftI2C, I2C
import machine
from lcd_api import LcdApi
from i2c_lcd import I2cLcd
from drive import DRIVE
from encoder_N20_esp import Motor, PWM, PID
from servo import Servo
import usocket
import time
import vl53l0x
import trct
import f
import network
def start() : return 1
IS_SUPER_STAR = True 
ID_ROBOT = 1
DIST_BORD = 50 #distance au dessus de la quelle la superstar considère être arrivé au bord de la scène
IR_TRESHOLD = 2500 #signal > IR_TRESHOLD => blanc
SET_POINT = 3
SIGNAL_DEPART = "top"
ssid = "SiERA"
password = "poupoule"


#i2c
i2c=SoftI2C(scl=Pin(22),sda=Pin(21), freq=40000)
tof=vl53l0x.VL53L0X(i2c)
tof.start()

lcd = I2cLcd(i2c, 0x27, 4, 20)
lcd.putstr("Hello")

#moteurs
motor_left = Motor(17,16,10,27,14)
motor_right = Motor(2, 4, 10, 25, 26) # Motor(M1, M2, EN, C1, C2, frequency) Default frequency is set at 50Hz
motor_left.speed(0)
motor_right.speed(0)

distance = 100
kp = 20
ki = 0.5
kd = 1.5
max_pwm = 350
p_left = PID(motor_left, kp*1.3, kd*1.1, ki*1.1, max_pwm*1.1)      # Tested Pid values for 12v 500rpm N20 motor
p_right = PID(motor_right, kp*0.9, kd, ki, max_pwm*1)      # Tested Pid values for 12v 500rpm N20 motor
drive = DRIVE(p_left, p_right, tof, distance)
#servos
servoA=Servo(pin=5)
servoB=Servo(pin=18)
#capteur ir
pin_ir = trct.init([36,39,34,35,32]) # se lit avec pin_ir[n].read()git list 
pin_values = [1,2,3,4,5]

buzzer = PWM(Pin(12, mode=Pin.OUT))
buzzer.freq(500)
buzzer.duty (0)
lcd.putstr("fin initialisation")
time.sleep(0.5)
#socket
lcd.clear()
lcd.putstr("setup wifi")
# Créer une interface Wi-Fi station
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.ifconfig(('192.168.0.142','192.168.0.1','255.255.255.0', '192.168.0.1'))
wlan.connect(ssid, password)

# Attendre la connexion
while not wlan.isconnected():
    lcd.clear()
    lcd.putstr("Connexion en cours...")
    time.sleep(1)
    
lcd.clear()
lcd.putstr("Connecté ! Adresse IP :" + str(wlan.ifconfig()[0]))

socket=usocket.socket(usocket.AF_INET,usocket.SOCK_DGRAM)
print("WLAN connecté :", wlan.isconnected())
print("Adresse IP :", wlan.ifconfig())
socket.bind(('0.0.0.0', 4000))
lcd.clear()  
lcd.putstr("en attente du signal...")
data = ""
print(SIGNAL_DEPART in data)
while not (SIGNAL_DEPART in data):
    data = socket.recv(2048)  # Taille max du message
    data = data.decode()
    print(SIGNAL_DEPART in data)
    print(data)
print("ok")
lcd.clear()
lcd.putstr("TOP !")


f.suivi_ligne(p_left, p_right,pin_ir,pin_values, tof,IS_SUPER_STAR,ID_ROBOT,DIST_BORD)
motor_left.speed(0)
motor_right.speed(0)
        
if IS_SUPER_STAR:
    f.avancer_bord()
    f.finish_super_star(servoA,servoB)
else : 
    f.go_zone()
    f.finish_suporter(servoA,servoB)
