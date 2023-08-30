# Vehicle Dynamics

This contains specifications of the Dallara AV-21R vehicle dynamics including actuation, powertrain, and aero.

## Vehicle properties

The AV-21 is built by Dallara and based on the Dallara IL-15 chassis. The engine is built by 4Piston Racing and based on a Honda K20.

**General Vehicle Specifications**

| Specification                  | Symbol          | Value | Units                |
|--------------------------------|-----------------|-------|----------------------|
| Mass                           | \( m \)        | 815.11   | kg                   |
| Moment of Inertia (x,y,z)      | \( I_x, I_y, I_z \) | 265,550,800 | kg·m<sup>2</sup> |
| Wheelbase                      | \( l \)        | 2.9718   | m                    |
| Center of Gravity to front axle| \( l_f \)      | 1.6785   | m                    |
| Center of Gravity to rear axle | \( l_r \)      | 1.2933   | m                    |
| Half track width front         | \( tw_f \)     | 1.638762 | m                    |
| Half track width rear          | \( tw_r \)     | 1.523969 | m                    |
| Tyre radius front              | \( R_f \)      | 0.3   | m                    |
| Tyre radius rear               | \( R_r \)      | 0.3   | m                    |

---

**Powertrain Specifications**

| Specification   | Value   | Units |
|-----------------|---------|-------|
| Engine max power| 473.25 @ 6110 rpm   | hp    |
| Engine max torque| 566.91 @ 5860 rpm   | Nm   |
| Engine redline  | 7500    | rpm  |
| Number of gears | 6     |     |
| Gear ratios     |   |     |
| 1st | 2.9167 | |
| 2nd | 1.8750 | |
| 3rd | 1.3809 | |
| 4th | 1.1154 | |
| 5th | 0.9600 | |
| 6th | 0.8889 | |
| Final drive | 3.0 | |
| Rear differential | LSD/Spool |  |

## Actuator Specifications

**Steer-by-wire Specifications**
| Specification                  | Symbol          | Value | Units                |
|--------------------------------|-----------------|-------|----------------------|
| Steering motor angle input range |\( delta_{cmd} \)|[-240, 240]|deg|
| Steering motor rate limit |\( delta_{rate} \)|500|deg/s|
| Steering ratio (motor to wheel) |\( delta_{ratio} \)|15.0|deg/deg|
| Steering motor pure delay |\( delta_{delay} \)|0.04|s|
| Steering motor bandwidth |\( delta_{bw} \)|3.0|Hz|

**Brake-by-wire Specifications**
| Specification                  | Symbol          | Value | Units                |
|--------------------------------|-----------------|-------|----------------------|
| Max brake pressure |\( b_{cmd} \)|6000|kPa|
| Max brake rate |\( b_{rate} \)|180000|kPa/s|
| Brake bias |\( b_{bias} \)|54|% Front|
| Brake constant ratio (pressure to torque) |\( b_{c} \)|1.0|Nm/kPa|
| Brake pure delay |\( b_{delay} \)|0.04|s|
| Brake bandwidth |\( b_{bw} \)|5.0|Hz|

## Aerodynamics

Aerodynamic effects are modeled with the drag coefficients and the lift coefficients and downforce is applied at each axle.
| Specification                  | Symbol          | Value | Units                |
|--------------------------------|-----------------|-------|----------------------|
| Coefficient of drag |\( C_d \)|0.8581||
| Coefficient of lift (front) |\( C_{lf} \)|-0.65||
| Coefficient of lift (rear) |\( C_{lr} \)|-1.18||
| Frontal area | \( A_f \) | 1.0 | \(m^2\) |