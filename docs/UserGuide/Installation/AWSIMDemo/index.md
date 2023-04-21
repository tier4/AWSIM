## Demo configuration
<!-- copied 100% from old -->
The simulation provided in the AWSIM demo is configured as follows:

|AWSIM Demo Settings||
|:--|:--|
|Vehicle|Lexus RX 450h|
|Environment|Japan Tokyo Nishishinjuku|
|Sensors|Gnss * 1<br> IMU * 1<br> LiDAR * 1<br> Traffic camera * 1|
|Traffic|Randomized traffic|
|ROS2|humble|

- Download and run [60% current] (**gifs**)
<!-- TODO only copied from old -->
## Start the demo

### Running the AWSIM simulation demo

To run the simulator, please follow the steps below.

1. Install Nvidia GPU driver (Skip if already installed).
    1. Add Nvidia driver to apt repository
    ```
    sudo add-apt-repository ppa:graphics-drivers/ppa
    sudo apt update
    ```
    2. Install the recommended version of the driver.
    ```
    sudo ubuntu-drivers autoinstall
    ```
    3. Reboot your machine to make the installed driver detected by the system.
    ```
    sudo reboot
    ```
    4. Open terminal and check if `nvidia-smi` command is available and outputs summary similar to the one presented below.
    ```
    $ nvidia-smi 
    Fri Oct 14 01:41:05 2022       
    +-----------------------------------------------------------------------------+
    | NVIDIA-SMI 515.65.01    Driver Version: 515.65.01    CUDA Version: 11.7     |
    |-------------------------------+----------------------+----------------------+
    | GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
    | Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
    |                               |                      |               MIG M. |
    |===============================+======================+======================|
    |   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
    | 37%   31C    P8    30W / 250W |    188MiB / 11264MiB |      3%      Default |
    |                               |                      |                  N/A |
    +-------------------------------+----------------------+----------------------+

    +-----------------------------------------------------------------------------+
    | Processes:                                                                  |
    |  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
    |        ID   ID                                                   Usage      |
    |=============================================================================|
    |    0   N/A  N/A      1151      G   /usr/lib/xorg/Xorg                133MiB |
    |    0   N/A  N/A      1470      G   /usr/bin/gnome-shell               45MiB |
    +-----------------------------------------------------------------------------+
    ```

2. Install Vulkan Graphics Library (Skip if already installed).
    1. Update the environment.
    ```
    sudo apt update
    ```
    2. Install the library.
    ```
    sudo apt install libvulkan1
    ```

3. Download and Run AWSIM Demo binary.

    1. Download `AWSIM_v1.1.0.zip`.

        [Download AWSIM Demo for ubuntu](https://github.com/tier4/AWSIM/releases/download/v1.1.0/AWSIM_v1.1.0.zip){.md-button .md-button--primary}
    
    2. Unzip the downloaded file.

    3. Make the `AWSIM_demo.x86_64` file executable.

        Rightclick the `AWSIM_demo.x86_64` file and check the `Execute` checkbox

        ![Make binary executable image](Image_1.png)

        or execute the command below.

        ```
        chmod +x <path to AWSIM folder>/AWSIM_demo.x86_64
        ```

    4. Launch `AWSIM_demo.x86_64`.
        ```
        ./<path to AWSIM folder>/AWSIM_demo.x86_64
        ``` 
        
        !!! warning
        
            It may take some time for the application to start the so please wait until image similar to the one presented below is visible in your application window.

        ![Running system image](Image_0.png)

- Run with Autoware (**gifs/video**)
<!-- TODO whole section -->
