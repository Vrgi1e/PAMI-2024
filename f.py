from math import sin
from servo import Servo
from time import sleep
from encoder_N20_esp import PID
import trct
import const
import network
import usocket

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
    pin_state = trct.read_bool(pin_ir,const.IR_TRESHOLD) #lis le capteur en on/off
    avg = 0
    total = 0
    for i in range (0,5):
        avg = avg + pin_state[i]*pin_values[i] #si un capteur est on sa	 position est ajouté au total
        total = total + pin_state[i]
    if total > 0 :
        avg = avg / total
    return avg

def suivi_ligne(motorA,motorB,pin_ir,pin_values,tof):
    #PID
    base_speed = 9000
    kp = 50
    kd = 5
    previous_error = 0
    nb_croisement = 0
    setpoint = 3
    avg = 3
    print("nb_croisements :" + str(nb_croisement))
    while (const.IS_SUPER_STAR and tof.read()< const.DIST_BORD or (not const.IS_SUPER_STAR) and (nb_croisements < const.ID_ROBOT or tof.read() > const.DIST_OBST)) :
        temp = get_avg(pin_ir,pin_values)
        if temp != 0 : avg = temp 
        sleep(0.1)
        error = avg-const.SET_POINT
        P = error * kp
        D = (error-previous_error)*kd
        PID_Value = P+D
        previous_error = error
        right_motor_speed = base_speed - PID_Value #TODO si le robot tourne dans le mauvais sens inerser les + et -
        left_motor_speed = base_speed + PID_Value
        print(str(left_motor_speed)+ " " +str(right_motor_speed) + " " + str(avg)+ " " +str(tof.read()))
        motorA.setSpeed(left_motor_speed)
        motorB.setSpeed(right_motor_speed)
        if (pin_ir[2]  > const.IR_TRESHOLD) and (pin_ir[4] > const.IR_TRESHOLD):#TODO tester si les marques de zone sont bien détectés (celles ci : ═╬═)
           nb_croisement += 1
           print ("marque detecté " + str(nb_croisement))
    motorA.setSpeed(0)
    motorB.setSpeed(0)        

def avancer_bord(drive):
    drive.forward(100, False) #TODO régler la distancepour s'approcher du bord
def go_zone(drive, team):
    if team == "orange" :
        drive.turne(90) #TODO doit trouner à gauche (-90 sinon)
        drive.forward(100) #TODO régler la distance pour aller dans la zone 
    elif team == "bleu" :
        drive.turne(-90) #TODO doit trouner à droite (90 sinon)
        drive.forward(100) #TODO régler la distance pour aller dans la zone
def setup_wifi(lcd):
    # Créer une interface Wi-Fi station
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.ifconfig(('192.168.0.142','192.168.0.1','255.255.255.0', '192.168.0.1'))
    wlan.connect(const.ssid, const.password)

# Attendre la connexion
    while not wlan.isconnected():
        lcd.clear()
        lcd.putstr("Connexion en cours...")
        sleep(1)
    
    lcd.clear()
    lcd.putstr("Connecté ! Adresse IP :" + str(wlan.ifconfig()[0]))

    socket=usocket.socket(usocket.AF_INET,usocket.SOCK_DGRAM)
    print("WLAN connecté :", wlan.isconnected())
    print("Adresse IP :", wlan.ifconfig())
    socket.bind(('0.0.0.0', 4000))
    return socket
