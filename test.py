from motor_control.AROMotorControl import AROMotorControl

mc = AROMotorControl()
mc.readPosition(motorid=1)


from template import run_until

dt = 0.005
N = int(2. / dt)

anglevalues =[]

    
import matplotlib.pyplot as plt

    
def store_values_and_apply_torques(motorid, torque):
    global anglevalues
    mc.applyTorqueToMotor(motorid=motorid, torque=torque)
    anglevalues+= [mc.readPosition(motorid)]

def resandrun(torque):
        global anglevalues
        anglevalues =[]

        try:
            run_until(store_values_and_apply_torques, N=N, dt=0.005, motorid=1, torque=torque)
        except KeyboardInterrupt:
            print("KeyboardInterrupt received, stopping motors...")
        except Exception as e:
            print(f"an error occurred: {e}")
        finally:
            mc.applyCurrentToMotor(1, 0)
            mc.applyCurrentToMotor(2, 0)
            print("motors stopped!")

        time_values = [i * dt for i in range(len(anglevalues))]
        # Plotting the angle values
        plt.figure(figsize=(10, 5))
        plt.plot(time_values, anglevalues, marker='o', linestyle='-')
        plt.title('Motor Angle Values Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Angle Values (% 2Pi)')
        plt.grid()
        plt.show()
        
        
def clip(output):
    #return output
    outabs = abs(output)
    if outabs < 1e-4:
        return 0
    clipped = max(min(outabs, 0.1), 0.02)
    return clipped if output > 0 else -clipped

class PController:
    
    def __init__(self, Kp):
        self.Kp = Kp
                      
    def shortest_path_error(self, target, current):
        diff = ( target - current + 180 ) % 360 - 180;
        if diff < -180:
                diff = diff + 360
        if (current + diff) % 360 == target:
                return diff
        else:
                return -diff
        
    def compute(self, target, current):
        error = self.shortest_path_error(target, current)
        output = self.Kp*error
        return clip(output)
        
        
def goTo(controller, target, time = 1., dt = 0.005, motorid =1):
    anglevalues =[]
    N = (int)(time / dt)
    
    def oneStep():
        nonlocal anglevalues
        currentAngle = mc.readPosition(motorid)
        anglevalues+=[currentAngle]
        tau = controller.compute(target,currentAngle)
        mc.applyTorqueToMotor(motorid,tau)   
        
    run_until(oneStep, N=N, dt=dt)
    
    mc.applyCurrentToMotor(1, 0)
    mc.applyCurrentToMotor(2, 0)
    
    time_values = [i * dt for i in range(len(anglevalues))]
    # Plotting the angle values
    plt.figure(figsize=(10, 5))
    plt.plot(time_values, anglevalues, marker='o', linestyle='-')
    plt.axhline(y=target, color='r', linestyle='--', label='Target Value')  # Add horizontal line for target
    plt.title('Motor Angle Values Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle Values (% 2Pi)')
    plt.grid()
    plt.show()
    
    
class PDController:
    
    def __init__(self, Kp, Kd):
        self.Kp = Kp
        self.Kd = Kd
        self.prev_error = 0
        self.first_call = True #do not compute error variation on first call
        
    
    def reset(self):
        self.first_call = True
                      
    def shortest_path_error(self, target, current):
        diff = ( target - current + 180 ) % 360 - 180;
        if diff < -180:
                diff = diff + 360
        if (current + diff) % 360 == target:
                return diff
        else:
                return -diff
        
    def compute(self, target, current, dt):
        error = self.shortest_path_error(target, current)        
        d_error = (error - self.prev_error) if not self.first_call else 0
        output = self.Kp*error + self.Kd * d_error
        self.first_call = False
        return clip(output)
        
    
def goTo(controller, target, time = 1., dt = 0.005, motorid =1):
    anglevalues =[]
    N = (int)(time / dt)
    
    def oneStep():
        nonlocal anglevalues
        currentAngle = mc.readPosition(motorid)
        anglevalues+=[currentAngle]
        tau = controller.compute(target,currentAngle, dt)
        mc.applyTorqueToMotor(motorid,tau)   
        
    run_until(oneStep, N=N, dt=dt)
    
    mc.applyCurrentToMotor(1, 0)
    mc.applyCurrentToMotor(2, 0)
    
    
    time_values = [i * dt for i in range(len(anglevalues))]
    # Plotting the angle values
    plt.figure(figsize=(10, 5))
    plt.plot(time_values, anglevalues, marker='o', linestyle='-')
    plt.axhline(y=target, color='r', linestyle='--', label='Target Value')  # Add horizontal line for target
    plt.title('Motor Angle Values Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle Values (% 2Pi)')
    plt.grid()
    plt.show()
    
dt = 0.005
N = int(30. / dt)

pc1 = PDController(0.000016,0.000);
pc2 = PDController(0.000016,0.000);
def loop():
    def step():
        angle1 = mc.readPosition(1)
        angle2 = mc.readPosition(2)
        tau1 = pc1.compute(angle2,angle1, dt)
        tau2 = pc1.compute(angle1,angle2, dt)
        mc.applyTorqueToMotor(1,tau1)   
        mc.applyTorqueToMotor(2,tau2)   
    run_until(step, N=N, dt=dt)
    
    mc.applyCurrentToMotor(1, 0)
    mc.applyCurrentToMotor(2, 0)
    
    
try:
    loop()
except KeyboardInterrupt:
    print("KeyboardInterrupt received, stopping motors...")
except Exception as e:
    print(f"an error occurred: {e}")
finally:
    mc.applyCurrentToMotor(1, 0)
    mc.applyCurrentToMotor(2, 0)
    print("motors stopped!")
