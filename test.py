class PIDController:
    
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.cummulative_error = 0
        self.first = True
        
    def reset(self):
        self.prev_error = 0
        self.cummulative_error = 0
        self.first = True
        
        
        
    def shortest_path_error(self, target, current):
        diff = ( target - current + 180 ) % 360 - 180;
        if diff < -180:
                diff = diff + 360
        if (current + diff) % 360 == target:
                return diff
        else:
                return -diff

        
    def compute(self, target, current, dt=0.01):
        error = self.shortest_path_error(target, current)
        self.cummulative_error += error * dt
        d_error = (error - self.prev_error) if not self.first else 0
        output = self.Kp*error + self.Kd * d_error / dt + self.Ki * self.cummulative_error
        #print(f"target: {target} current: {current} torque: {output}")
        self.prev_error = error
        self.first = False
        return output
    
    

pid1 = PIDController(0.00358,0.,0.00004)
pid2 = PIDController(0.00358,0.,0.00004)

target1 = 160
target2 = 90


import time
t = time.perf_counter()
wait = 1. / 1e4
N=int(10e3) #10 seconds
N=int(30) #1 seconds
dt = 2. / 1e3
for i in range(N):
    t +=dt
    #run your code
    while(time.perf_counter()-t<dt):
        pass
        time.sleep(wait)


import time

start = time.time()
from motor_control.AROMotorControl import AROMotorControl

mc = AROMotorControl()
#mc.setZero(1)
#mc.setZero(2)

pids = [pid1, pid2]

def goto(target=target1, motorid=1):
        a1 = mc.readPosition(motorid)
        print("initpos", a1)
        positions = []
        taus = []
        times = []
        t = time.perf_counter()
        pid = pids[motorid-1]
        pid.reset()
        try:
                for i in range(N):
                        a1 = mc.readPosition(motorid)
                        tau1 = pid.compute(target,a1, dt)
                        current = tau1/0.16
                        _ = mc.applyTorqueToMotor(motorid,tau1)
                        #_ = mc.applyCurrentToMotor(motorid,current)                        
                        taus+=[tau1]
                        positions += [(int)(a1)]
                        #run your code
                        while(time.perf_counter()-t<dt):
                                pass
                                time.sleep(wait)
                        times+=[time.perf_counter()  - t  ]
                        t =time.perf_counter()
                a1 = mc.readPosition(motorid)
                print("final", a1)
                print("positions", positions)
                print("taus", taus)                        
                print("times", times)                    
                #print("len", len(times))
                print("total", sum(times))
        except KeyboardInterrupt:
                print("KeyboardInterrupt received, stopping motors...")
        except Exception as e:
                print(f"an error occurred: {e}")
        finally:
                mc.applyCurrentToMotor(1, 0)
                mc.applyCurrentToMotor(2, 0)
                print("motors stopped!")
