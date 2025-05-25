from math import sin
from servo import Servo
from time import sleep
from encoder_N20_esp import PID
import trct

def finish_supporter(motorA, motorB):
    while True :
        for i in range (0,1000):
            a = 90*sin(2*3.14*i/50)
            print(a+90)
            print(a-90)
            motorA.move(a+90)
            motorB.move(a-90)
            sleep(0.015)
            
def finish_super_star(motorA, motorB):
    while True :
        for i in range (0,50):
            a = 90*sin(2*3.14*i/50)
            motorA.move(a+90)
            motorB.move(-a+90)
            sleep(0.015)
    
def get_avg(pin_ir,pin_values):
    pin_state = trct.read_bool(pin_ir,2000) #lis le capteur en on/off
    avg = 0
    total = 0
    for i in range (0,5):
        avg = avg + pin_state[i]*pin_values[i] #si un capteur est on sa	 position est ajouté au total
        total = total + pin_state[i]
    if total > 0 :
        avg = avg / total
    return avg

def suivi_ligne(motorA,motorB,pin_ir,pin_values,tof,IS_SUPERSTAR,ID_ROBOT,DIST_BORD):
    #PID
    base_speed = 9000
    kp = 50
    kd = 5
    previous_error = 0
    nb_croisement = 0
    setpoint = 3
    avg = 3
    while (IS_SUPERSTAR	and tof.read()>DIST_BORD or (not IS_SUPERSTAR) and nb_croisements < ID_ROBOT):
        temp = get_avg(pin_ir,pin_values)
        if temp != 0 : avg = temp 
        sleep(0.1)
        error = avg-setpoint
        P = error * kp
        D = (error-previous_error)*kd
        PID_Value = P+D
        previous_error = error
        right_motor_speed = base_speed - PID_Value
        left_motor_speed = base_speed + PID_Value
        print(str(left_motor_speed)+ " " +str(right_motor_speed) + " " + str(avg)+ " " +str(tof.read()))
        motorA.setSpeed(left_motor_speed)
        motorB.setSpeed(right_motor_speed)
    motorA.setSpeed(0)
    motorB.setSpeed(0)        

def avancer_bord():
    return 0
def go_zone():
    return 0
    
        
    