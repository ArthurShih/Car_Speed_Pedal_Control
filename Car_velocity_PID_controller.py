# TODO: Car Velocity PID controller
import numpy as np 
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import random

# Model: 
# m = mass of car, passengers and cargo 
# v = car velocity
# Fp = thrust parameter (N/%pedal)
# rho = air density
# A = vehicle cross section
# Cd = drag coeffecient
# theta = uphill or downhill degree
# m*(dv/dt) = F*u - 0.5*rho*A*Cd*v**2 - m*sin(theta)

# SYSTEM
# Model parameters
mc = 500 # mass of car = 500 kg
mp_c = 200 # mass of passangers and cargo = 200 kg
m = mc + mp_c # mass of car, passengers and cargo
Fp = 30 # 30 N/%pedal
rho = 1.225 #kg/m**3
A = 5 #m**2
Cd = 0.24
target_velocity = 30

# Simulation parameter
t_start = 0 
t_end = 100
t_step = 1
N = int(t_end/t_step)
t = np.arange(t_start,t_end,t_step)
theta = 0
udata = np.zeros(N)
udata[11:] = 45
theta = 0
vdata = np.zeros(N)

# Discrete Simulation
for i in range(0,N):
	vdata[i] = (t_step/m) * (Fp*udata[i] - 0.5*rho*A*Cd*vdata[i-1]**2 - m*np.sin(theta)) + vdata[i-1]

plt.figure(1)
plt.suptitle("Car Velocity plot")
plt.subplot(211)
plt.title("Car velocity, V end ="+str(round(vdata[-1])))
plt.plot(t,vdata,'r-')
plt.plot([t_start,t_end],[target_velocity,target_velocity],'k-') # Setpoint is 30
plt.legend(["Velocity","Set point"])
plt.ylabel("V (m/s)")
plt.grid()
plt.subplot(212)
plt.title("u (Pedal %)")
plt.plot(t,udata)
plt.grid()
plt.xlabel("t (s)")
plt.ylabel("%")
#########################################################################################################
# TODO: Use PID controller to control car velocity when passing through uphills or downhills

# By the graph we plot, notice that 
deltav = 43 
deltau = 45
Kp = deltav / deltau
theta_p = 0
tao_p = 15

# By IMC moderate tuning
tao_c = tao_p
Kc = (1/Kp)*((tao_p+0.5*theta_p)/(tao_c+0.5*theta_p))
tao_i = tao_p+0.5*theta_p
tao_d = tao_p*theta_p/(2*tao_p+theta_p)
Kc = 1/Kp
P = Kc*2 # modify P by doubling Kc 
I = Kc/tao_i
D = Kc*tao_d

# Simulation parameters
errordata = np.zeros(N)
vdata = np.zeros(N)
udata = np.zeros(N)
sum_error = 0
thetadata = np.zeros(N)

# Simualtion with PID controller
for i in range(0,N):
	error = target_velocity - vdata[i-1]
	errordata[i] = error
	sum_error = sum_error + error*t_step
	u = P*error + I*sum_error + D*(errordata[i]-errordata[i-1])/t_step 
	# Constrain umax and umin
	if u>=100:
		u=100
	elif u<=-50:
		u=-50
	udata[i] = u
	T = 10 # times of changing degree
	k = N/T
	if i%k == 0:
		theta = random.randint(-30,30)*np.pi/180 # randomly passing through uphills or downhills
	thetadata[i] = theta
	vdata[i] = (t_step/m) * (Fp*u - 0.5*rho*A*Cd*vdata[i-1]**2 - m*np.sin(theta)) + vdata[i-1]

plt.figure(2)
plt.subplot(221)
plt.suptitle("Car velocity by PID control")
plt.title("Car velocity")
plt.plot(t,vdata,'r-')
plt.plot([t_start,t_end],[target_velocity,target_velocity],'k-') # Setpoint is 30
plt.legend(["Velocity","Set point"])
plt.ylabel("V (m/s)")
plt.grid()
plt.subplot(222)
plt.plot(t,udata,'m-')
plt.grid()
plt.title("u (Pedal%)")
plt.ylabel("%")
plt.subplot(223)
plt.title("Velocity error")
plt.plot(t,errordata,'g-')
plt.xlabel("t (s)")
plt.ylabel("V_target - V (m/s)")
plt.grid()
plt.subplot(224)
plt.plot(t,thetadata*180/np.pi)
plt.title("Degree of incline or decline")
plt.xlabel("t (s)")
plt.ylabel("Degree")
plt.grid()
plt.show()
