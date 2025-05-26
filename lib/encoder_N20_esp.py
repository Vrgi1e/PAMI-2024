"""
MIT License

Copyright (c) 2022 rakesh-i

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""


from time import sleep_ms, ticks_us, ticks_ms
from machine import Pin, PWM, disable_irq, enable_irq

# A class for functions related to motors
class Motor:
    
    # Instance variable for keeping the record of the encoder position
    pos = 0

    # Interrupt handler
    def handle_interrupt(self,pin):
        a = self.px.value()
        if a > 0:
            self.pos = self.pos+1
        else:
            self.pos = self.pos-1
    
    # Constroctor for initializing the motor pins
    def __init__(self,m1, m2, en, c1, c2, freq=50):
        self.px = Pin(c1, Pin.IN)
        self.py = Pin(c2, Pin.IN)
        self.freq = freq
        self.p_in1 = Pin(m1, Pin.OUT)
        self.pwm_in1 = PWM(self.p_in1, freq)
        self.p_in2 = Pin(m2, Pin.OUT)
        self.pwm_in2 = PWM(self.p_in2, freq)
        #self.p_en = PWM(Pin(en,Pin.OUT), freq)
        # Interrupt initialization
        self.py.irq(trigger=Pin.IRQ_RISING, handler=self.handle_interrupt)
    
    # Arduino's map() function implementation in python 
    def convert(self, x, i_m, i_M, o_m, o_M):
        return max(min(o_M, (x - i_m) * (o_M - o_m) // (i_M - i_m) + o_m), o_m)

    # A function for speed control without feedback(Open loop speed control)
    def speed(self,M):
        pwm = self.convert(abs(M),0, 1000, 0, 1000) 
        #self.p_en.duty(pwm)
        #print(pwm)
        if M>=0:
            self.pwm_in1.duty(pwm)
            self.pwm_in2.duty(0)
            self.p_in2.off()
        else:
            self.p_in1.off()
            self.pwm_in1.duty(0)
            self.pwm_in2.duty(pwm)

# A class for closed loop speed and postion control
class PID:
    
    # Instance variable for this class
    posPrev = 0

    # Constructor for initializing PID values
    def __init__(self, M, kp=1, kd=0, ki=0, umaxIn=800, eprev=0, eintegral=0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.M = M                   # Motor object
        self.umaxIn  =umaxIn
        self.eprev = eprev
        self.eintegral = eintegral
        self.target = 0
        self.target_before_obstacle = 0
        self.since_Target = 0
        self.attTarget = False
        self.last_tick_ms = ticks_us()
        

    # Function for calculating the Feedback signal. It takes the current value, user target value and the time delta.
    def evalu(self,value, target, deltaT):
        
        # Propotional
        e = target-value 

        # Derivative
        dedt = (e-self.eprev)/(deltaT)

        # Integral
        self.eintegral = self.eintegral + e*deltaT
        
        # Control signal
        u = self.kp*e + self.kd*dedt + self.ki*self.eintegral
        
        # Direction and power of  the control signal
        if u > 0:
            if u > self.umaxIn:
                u = self.umaxIn
            else:
                u = u
        else:
            if u < -self.umaxIn:
                u = -self.umaxIn
            else:
                u = u 
        self.eprev = e
        return u
    
    # Function for closed loop position control
    def runTarget(self, obstacle):
        
        # Time delta is predefined as we have set a constat time for the loop.(Initial dealy is 
        # very high,interfers with response time)(Integral part becomes very high due to huge deltaT value at the beginning)
        # Use tick_us() to calculate the delay manually(remove slee_ms() if you use realtime delay)
        deltaT = (ticks_us() - self.last_tick_ms)/1000000
        #print("DeltaT : ", deltaT)
        self.last_tick_ms = ticks_us()
        
        
        # Disable the interrupt to read the position of the encoder(encoder tick)               
        state = disable_irq()
        step = self.M.pos
        
        # Enable the intrrupt after reading the position value
        enable_irq(state)

        # Control signal call
        x = int(self.evalu(step, self.target, deltaT))
        
        # Set the speed

        #print(step, self.target) # For debugging
        if not obstacle :
            self.M.speed(x)
        else :
            self.M.speed(0)

        
        condition = abs(x)
        #print("Conrrection : ", condition, ", pos = : ", step)
        if(condition<=20000*1/self.umaxIn):
            if(not self.attTarget):
                self.since_Target = ticks_ms()
                self.attTarget = True
        else :
            self.attTarget = False
        #print(ticks_ms()-self.since_Target)
        if(ticks_ms()-self.since_Target>200 and self.attTarget):
            print("Next Step")
            return True
        else :
            return False
            
    def setTarget(self, target):
        self.last_tick_ms = ticks_us()
        print(target)
        self.target = target
        
        
    # Function for closed loop speed control
    def setSpeed(self, target):
        state = disable_irq()
        posi = self.M.pos
        enable_irq(state)

        # Delta is high because small delta causes drastic speed stepping.
        deltaT = .05

        # Target RPM
        vt = target 

        # Current encoder tick rate
        velocity = (posi - self.posPrev)/deltaT
        self.posPrev = posi

        # Converted to RPM
        # 350 ticks per revolution of output shaft of the motor 
        # For different gearing differnt values
        # Run the function without setting the motor speed and calculate the ticks per revolution by manually rotaing the motor  
        v = velocity/350*60

        # Call for control signal
        x = int(self.evalu(v, vt, deltaT))

        # Set the motor speed
        self.M.speed(x)
        #print(v, vt)   # For debugging

        # Constant delay
        sleep_ms(50)
        
    def get_pos(self):
        return self.M.pos
    
    def set_tar_bfr_obs(self, tar):
        self.target_before_obstacle = tar
        
    def get_tar_bfr_obs(self):
        return self.target_before_obstacle
        
    def get_tar(self):
        return self.target
    
    
