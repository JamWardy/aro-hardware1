class PIDController:
    
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.cummulative_error = 0
        
    def shortest_path_error(self, target, current):
        error = (target - current + 180) % 360 - 180
        return error
        
    def compute(self, target, current, dt=0.01):
        error = self.shortest_path_error(target, current)
        self.cummulative_error += error
        d_error = (error - self.prev_error) / dt
        output = self.Kp*error + self.Kd * d_error
        #print(f"target: {target} current: {current} torque: {output}")
        self.prev_error = error
        return output
    
    

pid1 = PIDController(5,0.,30)
pid2 = PIDController(5,0.,15)

target1 = 160
target2 = 90


import time
t = time.perf_counter()
N=int(10e1) #10 seconds
dt = 1. / 1e3
wait = 1. / 1e4
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


def goto(target=target1):
        positions = []
        taus = []

        try:
                for i in range(N):
                        t +=dt
                        a1 = mc.readPosition(1)
                        tau1 = pid1.compute(target,a1, dt) * 0.01
                        _ = mc.applyCurrentToMotor(1,tau1)
                        taus+=[tau1]
                        positions += [(int)(a1)]
                    #run your code
                while(time.perf_counter()-t<dt):
                        pass
                        time.sleep(wait)
                print("positions", positions)
                print("taus", taus)
                        
        except KeyboardInterrupt:
                print("KeyboardInterrupt received, stopping motors...")
        except Exception as e:
                print(f"an error occurred: {e}")
        finally:
                mc.applyCurrentToMotor(1, 0)
                mc.applyCurrentToMotor(2, 0)
                print("motors stopped!")
