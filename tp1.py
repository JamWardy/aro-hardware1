from motor_control.AROMotorControl import AROMotorControl

mc = AROMotorControl()
mc.readPosition(motorid=1)


from template import run_until
run_until(mc.applyTorqueToMotor, N=100, dt=0.005, motorid=1, torque=0.02)
mc.applyTorqueToMotor(1,0.)
