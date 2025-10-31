

<popup-img src="image.png" alt="image"></popup-img> 
[logitechg.com : Logitech G29 Steering wheel](https://www.logitechg.com/en-us/shop/p/driving-force-racing-wheel.941-000110)

## Overview
The `AWSIM-demo.x86-64` simulation (included in [AWSIM-demo.zip](../../../Downloads/index.md)) featured in the [Quick-start-demo](../../QuickStartDemo/index.md) page supports the Logitech G29 Steering wheel. This page explains how to set it up and configure it.

**The behavior of G29 varies depending on the control mode.**

- Control mode is `AUTONOMOUS` :  G29 automatically reflects the vehicle's steering angle.
- Control mode is `MANUAL` : User can control the vehicle via G29.

## How to setup

1. Connect the Logitech G29 to your PC. (PS3 mode recommended)

    !!! warning
        When connected to a PC, automatic calibration will start and the steering wheel will rotate. Please be careful to avoid injury.

1. Check the device path for G29.
    
    Enter `evtest` command.
    ```
    evtest
    ```
    Result.
    ```{.yml .no-copy}
    No device specified, trying to scan all of /dev/input/event*
    Not running as root, no devices may be available.
    Available devices:
    /dev/input/event16:	input-remapper gamepad
    /dev/input/event22:	Logitech G29 Driving Force Racing Wheel
    Select the device event number [0-22]:
    ```

    In this case, the device path for G29 is `/dev/input/event22`.

1. Enter the g29 device path in `sample-config.json`. Please replace `<device path number>` as appropriate.

    !!! info 
        `sample-config.json` is included in [AWSIM-demo.zip](../../../Downloads/index.md).

    ```json
    ~~
    "LogitechG29Settings": {
        "_devicePath": "/dev/input/event<device path number>",
    ~~
    ```

1. Launch `AWSIM-demo.x86-64` with `sample-config.json`
    ```
    ./AWSIM-demo.x86_64 --json_path <direcotry path>/<config json name>.json
    ```
1. Select Logitech G29 from the UI's Vehicle input device.

    <popup-img src="image_1.png" alt="image_1"></popup-img> 


## Key map

|Key|Feature|
|:--|:--|
|Triangle|Switch to move drive gear.|
|Square|Switch to move reverse gear.|
|Circle|Switch to neutral gear.|
|Cross|Switch to parking gear.|
|Throttle pedal|Forward acceleration.|
|Brake pedal|Reverse acceleration.|
|Steering wheel|Turning.|
|Left paddle|Turn left blinker on.|
|Right paddle|Turn right blinker on.|
|R2|Turn on hazard lights.|
|R3|Turn off blinker or hazard lights.|
|L2|Switch control mode `AUTONOMOUS` to `MANUAL`|
|L3|Switch control mode `MANUAL` to `AUTONOMOUS`|