# Car_Speed_Pedal_control
### Model
![Model](https://github.com/ArthurShih/Car_Velocity_PID_control/blob/master/figure/Model.png)
###### In model, F is thrust parameter(N/%pedal), and u is % of pedal.
###### Now, we have our system:
###### .
###### m(dv/dt) = F*u - 0.5*rho*A*Cd*v^2 - m*sin(theta)
###### .
###### Assume we'd like to control car's speed into 30m/s (~=67mph). We start to step on pedal 45% at 10s
##### Speed and u plot
![Speed](https://github.com/ArthurShih/Car_Velocity_PID_control/blob/master/figure/velocity.png)
###### .
###### From the plot above, we can find delta v = 43, delta u = 45, theta_p = 0, tao_p ~= 15.
###### Now, we can use values above to attemp our PID controller following IMC tunning.
#### IMC moderate tunning
###### Kp = (delta v)/(delta u)
###### tao_c is the larger of tao_p or 8*theta_p
###### Kc = (tao_p+0.5*theta_p)/(Kp*(tao_c+0.5theta_p))
###### tao_i = tao_p + 0.5*theta_p
###### tao_d = (tao_p*theta_p)/(2*tao_p+theta_p)
###### To accelerate controller, I doubled value Kc
#### Random road angle
###### Now we are ready to control car's speed into our desired speed. 
###### TODO: Control car's velocity when it passes through uphills or downhills.(angle of uphills and downhills are generated randomly between -30~30 degrees)
###### .
##### Result
![Result](https://github.com/ArthurShih/Car_Velocity_PID_control/blob/master/figure/result.png)
