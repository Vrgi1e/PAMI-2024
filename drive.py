from time import sleep_ms, ticks_us, ticks_ms

class DRIVE:
    def __init__(self, p_left, p_right, team, tof, threshold,stop_time = 99):
        self.PL = p_left
        self.PR = p_right
        self.mm_per_tick = 30 * 3.14159265358979323846 / 350
        spacing = 90
        self.deg_per_tick = (self.mm_per_tick*360)/(spacing * 3.14159265358979323846)
        if team :
            self.deg_per_tick = self.deg_per_tick *-1
        self.tof = tof
        self.obstacle = False
        self.threshold = threshold
        self.last_distance_aquired = 0
        self.start_time = ticks_ms()
        self.stop_time = stop_time*1000
    
    def forward(self, mm, final = True):
        target = int(mm/self.mm_per_tick)
        self.PL.setTarget(target + self.PL.get_tar())
        self.PR.setTarget(target + self.PR.get_tar())
        self.run(final)
    
    def turn_right(self, angle, final = True):
        target = angle/self.deg_per_tick
        self.PL.setTarget(-target+ self.PL.get_tar())
        self.PR.setTarget(target+ self.PR.get_tar())
        self.run(final)
    
    def turn_left(self, angle, final = True):
        target = angle/self.deg_per_tick
        self.PL.setTarget(target+ self.PL.get_tar())
        self.PR.setTarget(-target+ self.PR.get_tar())
        self.run(final)
        
    def run(self, capteur = True):
        arrived_l = False
        arrived_r = False
        
        
        while not arrived_l and not arrived_r :
            if ticks_ms()-self.start_time < self.stop_time:
                
                if ticks_ms() - self.last_distance_aquired > 100:
                    if capteur :
                        self.obstacle = self.tof.read()<self.threshold
                        self.last_distance_aquired = ticks_ms()
                        print("obstacle", self.obstacle)
                    else :
                        self.obstacle = False
                
                
                
                

                
                
                arrived_l = self.PL.runTarget(self.obstacle) and not self.obstacle
                arrived_r = self.PR.runTarget(self.obstacle) and not self.obstacle
                sleep_ms(5)
            else :
                print("finish")
                arrived_l = True
                arrived_r = True
        
        