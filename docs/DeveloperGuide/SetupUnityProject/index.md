!!! info
    It is advised to checkout the [Quick Start Demo](../../GettingStarted/QuickStartDemo/index.md) tutorial before reading this section.


## 1. Environment preparation

Ubuntu OS is required.

1. Make sure your machine meets the [required hardware specifications](../../GettingStarted/QuickStartDemo/#1-preparation).
    - *NOTE: PC requirements may vary depending on simulation contents which may change as the simulator develops*
1. Prepare a desktop PC with Ubuntu 22.04 installed.
1. Install nvidia gpu driver (Skip if already installed).
    1. Add nvidia driver to apt repository.
    ```
    sudo add-apt-repository ppa:graphics-drivers/ppa &&
    sudo apt update
    ```
    1. Install the recommended version of the driver.
    ```
    sudo ubuntu-drivers autoinstall
    ```

        !!! info
            Version 570 or higher is recommended.

    1. Reboot your machine to make the installed driver detected by the system.
    ```
    sudo reboot
    ```

    1. Open terminal and check if nvidia-smi command is available and outputs summary similar to the one presented below.
    ```
    nvidia-smi 
    ```
    Check result.
    ```txt { .yaml .no-copy}
    Fri May  2 18:55:24 2025       
    +-----------------------------------------------------------------------------------------+
    | NVIDIA-SMI 570.124.06             Driver Version: 570.124.06     CUDA Version: 12.8     |
    |-----------------------------------------+------------------------+----------------------+
    | GPU  Name                 Persistence-M | Bus-Id          Disp.A | Volatile Uncorr. ECC |
    | Fan  Temp   Perf          Pwr:Usage/Cap |           Memory-Usage | GPU-Util  Compute M. |
    |                                         |                        |               MIG M. |
    |=========================================+========================+======================|
    |   0  NVIDIA GeForce RTX 4090        Off |   00000000:01:00.0  On |                  Off |
    |  0%   45C    P8             25W /  450W |    5920MiB /  24564MiB |      7%      Default |
    |                                         |                        |                  N/A |
    +-----------------------------------------+------------------------+----------------------+
    ```

1. Install vulkan graphics library (Skip if already installed).
    1. Update the environment.
    ```
    sudo apt update
    ```
    1. Install the library.
    ```
    sudo apt install libvulkan1
    ```

1. Install [git](https://git-scm.com/) (Skip if already installed).
1. Set the ROS 2 middleware and the localhost only mode as described in the official Autoware documentation:  
   [Autoware Documentation – DDS settings for ROS 2 and Autoware](https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/dds-settings/)
1. Restart PC.


AWSIM comes with a *standalone* flavor of [`Ros2ForUnity`](../../Components/ROS2/ROS2ForUnity/index.md). This means that, to avoid internal conflicts between different ROS 2 versions, you shouldn't run the Editor or AWSIM binary with ROS 2 sourced.

!!! warning
    **Do not run the AWSIM, Unity Hub, or the Editor with ROS 2 sourced.**

- Make sure that the terminal which you are using to run Unity Hub, Editor, or AWSIM doesn't have ROS 2sourced.
- It is common to have ROS 2 sourced automatically with `~/.bashrc` or `~/.profile`. Make sure it is not obscuring your working environment:
    - Running Unity Hub from the Ubuntu GUI menu takes the environment configuration from `~/.profile`.
    - Running Unity Hub from the terminal uses the current terminal configuration from `~/.profile` and `~/bashrc`.
    - Running Unity Editor from the UnityHub inherits the environment setup from the Unity Hub. 

## 2. Unity installation

!!! info

    AWSIM's Unity version is currently **6000.0.34f1**

1. Install [UnityHub](https://docs.unity3d.com/hub/manual/InstallHub.html).
1. Download Unity 6000.0.34f1 from the [download archive](https://unity.com/releases/editor/archive).
   <popup-img src="image_0.png" alt="image_0"></popup-img>

## 3. Open AWSIM project

1. Git clone AWSIM repository.
    ```
    git clone git@github.com:tier4/AWSIM.git
    ```

1. Open AWSIM project.
    Open a terminal and launch Unity Editor with the following command (adjust the path if necessary):  
    ```bash  
    ~/Unity/Hub/Editor/6000.0.34f1/Editor/Unity -projectPath "/home/user/AWSIM/"
    ```
    This ensures that the environment variables set in `~/.bashrc` are applied correctly.

    !!! info
        If you launch Unity Hub directly from the Ubuntu applications menu (without using the terminal), the environment variables from `~/.bashrc` will not be applied, and AWSIM may not work correctly. Always start Unity from the terminal as shown above.

        Alternatively, you can permanently fix this by wrapping the Unity binary with a small script.  
        Run the following commands (adjust the Unity version and paths as necessary):

        ```bash
        cd ~/Unity/Hub/Editor/6000.0.34f1/Editor/
        mv Unity Unity.bin

        tee Unity >/dev/null <<'SH'
        #!/usr/bin/env bash
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export CYCLONEDDS_URI=file:///home/user/cyclonedds.xml
        exec "$(dirname "$0")/Unity.bin" "$@"
        SH
        chmod +x Unity
        ```

        With this change, Unity will always start with the required environment variables, even when launched from Unity Hub.


    !!! warning

        If you get the safe mode dialog when starting UnityEditor, you may need to install openssl.

        1. download libssl  
        `$ wget http://security.ubuntu.com/ubuntu/pool/main/o/openssl1.0/libssl1.0.0_1.0.2n-1ubuntu5.13_amd64.deb`
        2. install  
        `sudo dpkg -i libssl1.0.0_1.0.2n-1ubuntu5.13_amd64.deb`
        
## 4. Import external packages

1. Download `Shinjuku.unitypackage`

    [Download Shinjuku.unitypackage](https://github.com/tier4/AWSIM/releases/download/v2.0.0/Shinjuku.unitypackage){ .md-button }

1. In Unity Editor, from the menu bar at the top, select `Assets -> Import Package -> Custom Package...` and navigate the `Shinjuku.unitypackage` file.
1. `Shinjuku.unitypackage` package has been successfully imported under `Assets\AWSIM\Externals` directory.
    
    !!! info

        The `Externals` directory is added to the `.gitignore` because the map has a large file size and should not be directly uploaded to the repository.

## 5. Run the demo in Editor
The following steps describe how to run the demo in Unity Editor:

1. Open the `AutowareSimulationDemo.unity` scene placed under `Assets/AWSIM/Scenes/` directory.
1. Run the simulation by clicking `Play` button placed at the top section of Edtior.
    <popup-img src="image_1.png" alt="image_1"></popup-img>