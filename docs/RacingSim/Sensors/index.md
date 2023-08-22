# Sensors

The AV-21R is instrumented with various sensors for localization, perception, control, and safety.

## Vehicle sensors

### Wheels
- Wheel speeds
- Tire temperature
- Tire pressure
- Brake temperatures

### Actuators
- Accelerator input and output
- Brake input and output
- Steering motor angle input and output
- Steering motor torque

### Miscellaneous
- Battery voltage
- Safety switch state
- Mode switch state

### Powertrain
- engine rpm
- vehicle speed
- current gear
- throttle position
- gear shift status
- torque estimate at wheels
- engine on status
- engine run switch status
- MAP sensor
- lambda sensor
- fuel level
- fuel pressure
- engine oil pressure
- engine oil temperature
- engine coolant pressure
- engine coolant temperature
- gearbox oil pressure
- gearbox accumulator pressure
- gearbox oil temperature

## Raptor state machine
The low-level ECU "Raptor" has a state machine to ensure safety during real world operation. This state machine handles the interactions between the autonomy software stack (ct_state), the low-level vehicle functions (sys_state), and race control (vehicle flag). 

In "Raptor Hot Start" mode, the vehicle will start in sys_state = 9, which means the vehicle is ready to drive immediately.

With "Hot Start" diabled, the vehicle will start in sys_state = 19, which means the vehicle is waiting for engine ignition. To start the engine, one must send an orange flag as well as a ct_state = 5. Once that is done, the orange flag can be removed, and the car can launch.

## GNSS/INS

The AV-21R has three GNSS/INS units:
- 1 VectorNav VN-310
- 2 Novatel PwrPak7D-E1

## Lidar
The AV-21R has three Luminar lidars.

## Camera
The AV-21R has six Allied Vision cameras.

## Radar
The AV-21R has three radars.