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
import const

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
socket = f.setup_wifi(lcd)
lcd.clear()  
lcd.putstr("en attente du signal...")
data = ""

while not (const.SIGNAL_DEPART in data): #TODO ajouter tirette
    data = socket.recv(2048)  # Taille max du message
    data = data.decode()
    print(const.SIGNAL_DEPART in data)
    print(data)
print("ok")
lcd.clear()
lcd.putstr("TOP !")


f.suivi_ligne(p_left, p_right,pin_ir,pin_values, tof)

motor_left.speed(0)
motor_right.speed(0)

if const.IS_SUPER_STAR: #TODO tester les 2 comportements (superstar ou fan)
    lcd.clear()
    lcd.putstr("CAN'T STOP ADDICTED TO THE SHINDING !") #TODO afficher un truc mieux jsp
    f.avancer_bord(drive)
    f.finish_super_star(servoA,servoB)
else :
    lcd.clear()
    lcd.move_to(1,6)
    lcd.putstr("OUE OUE") #TODO pareil
    f.go_zone(drive)
    f.finish_suporter(servoA,servoB)
