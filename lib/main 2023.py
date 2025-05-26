from encoder_N20_esp import Motor, PID
from drive import DRIVE
from machine import UART, Pin
import time
import neopixel
import network
import usocket as socket
from machine import SoftI2C,I2C
import vl53l0x


np = neopixel.NeoPixel(Pin(10), 8)
yellow = (120,120,0)
blue = (0,0,120)
distance = 200



i2c=SoftI2C(scl=Pin(22),sda=Pin(21), freq=40000,timeout=50000)
tof=vl53l0x.VL53L0X(i2c)
tof.start()


def set_full_np(np, color):
    for i in range(np.n):
        if not i  == 3 and not i == 4:
            np[i]= color
        else :
            np[i]= (0,0,0)
    np.write()

set_full_np(np, (120, 0, 0))
motor_left = Motor(16, 17, 10, 33, 32) # Motor(M1, M2, EN, C1, C2, frequency) Default frequency is set at 50Hz
motor_right = Motor(4, 2, 10, 25, 26) # Motor(M1, M2, EN, C1, C2, frequency) Default frequency is set at 50Hz

kp = 20
ki = 0.5
kd = 1.5
max_pwm = 350
p_left = PID(motor_left, kp*1.3, kd*1.1, ki*1.1, max_pwm*1.1)      # Tested Pid values for 12v 500rpm N20 motor
p_right = PID(motor_right, kp*0.9, kd, ki, max_pwm*1)      # Tested Pid values for 12v 500rpm N20 motor




team = False
color = blue
beggined = False



drive = DRIVE(p_left, p_right, team, tof, distance)



#VVVVV   HERE ONLY   VVVVV
#Match
start_delay = 2 #secs

#Test
#start_delay = 0 #secs
#####UP ONLY########


while start_delay > 0:
    if start_delay % 2:
        set_full_np(np, color)
    else:
        set_full_np(np, (0,0,0))
    start_delay -=1
    time.sleep(1)





#VVVVV   HERE ONLY   VVVVV
#Match
drive.forward(100)
drive.turn_right(90)
drive.forward(100)
drive.turn_right(90)
drive.forward(100)
drive.turn_right(90)
drive.forward(100)
drive.turn_right(90)
drive.forward(100)
drive.turn_right(90)
drive.forward(100)
drive.turn_right(90)
#Test
#drive.forward(200)

#drive.turn_right(1400)
#####UP ONLY########


motor_left.speed(0)
motor_right.speed(0)



def wheel(pos):
    # Input a value 0 to 255 to get a color value.
    # The colors are a transition r - g - b - back to r.
    if pos < 0 or pos > 255:
        return (0, 0, 0)
    if pos < 85:
        return (int(pos * 3), int(255 - pos * 3), 0)
    if pos < 170:
        pos -= 85
        return (int(255 - pos * 3), 0, int(pos * 3))
    pos -= 170
    return (0, int(pos * 3), int(255 - pos * 3))

def rainbow_cycle(np, wait_ms=20, iterations=5):
    # Function to create rainbow cycle on NeoPixel strip.
    for j in range(255 * iterations):
        for i in range(np.n):
            pixel_index = (i * 256 // np.n) + j
            np[i] = wheel(pixel_index & 255)
        np.write()
        time.sleep_ms(wait_ms)

# Main program loop
while True:
    # Call the rainbow_cycle function to create the rainbow effect
    rainbow_cycle(np)



