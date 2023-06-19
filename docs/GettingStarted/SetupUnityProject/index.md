# Setup Unity Project

!!! info

    It is advised to checkout the [Quick Start Demo](../QuickStartDemo) tutorial before reading this section.

This page is a tutorial for setting up a AWSIM Unity project.

## Environment preparation

### System setup

=== "Ubuntu 22"
    1. Make sure your machine meets the [required hardware specifications](../QuickStartDemo/#pc-specs).
        - *NOTE: PC requirements may vary depending on simulation contents which may change as the simulator develops*
    2. Prepare a desktop PC with Ubuntu 22.04 installed.
    2. Install [Nvidia drivers and Vulkan Graphics API](../QuickStartDemo/#running-the-awsim-simulation-demo).
    3. Install [git](https://git-scm.com/).

=== "Windows"
    1. Make sure your machine meets the [required hardware specifications](../QuickStartDemo/#pc-specs).
        - *NOTE: PC requirements may vary depending on simulation contents which may change as the simulator develops*
    2. Prepare a desktop PC with Windows 10 or 11 (64 bit) installed.
    3. Install [git](https://git-scm.com/).

### Unity installation

!!! info

    AWSIM's Unity version is currently **2021.1.7f1**

Follow the steps below to install Unity on your machine:

1. Install UnityHub to manage Unity projects. Please go to [Unity download page](https://unity3d.com/get-unity/download) and download latest `UnityHub.AppImage`.
![](image_1.png)
2. Install Unity 2021.1.7f1 via UnityHub.
    - Open new terminal, navigate to directory where `UnityHub.AppImage` is download and execute the following command:
```
./UnityHub.AppImage
```
    - To install Unity Editor please proceed as shown on the images below
![](image_2.png)
![](image_3.png)
![](image_4.png)
    - At this point, your Unity installation process should have started.

        === "Ubuntu 22"
        - *NOTE: If the installation process has not started after clicking the green button (image above), please copy the hyperlink (by rightclicking the button and selecting `Copy link address`) and add it as a argument for Unity Hub app. An example command:
        ```
        ./UnityHub.AppImage unityhub://2021.1.7f1/d91830b65d9b
        ```

    - After successful installation the version will be available under the `Installs` tab in Unity Hub.
![](image_5.png)

### Open AWSIM project

To open the Unity AWSIM project in Unity Editor:
1. Make sure you have the AWSIM repository cloned
    ```
    git clone git@github.com:tier4/AWSIM.git
    ```

2. Launch UnityHub.
    ```
    ./UnityHub.AppImage
    ```

3. Open the project in UnityHub
    - Click the `Open` button
![](image_6.png)

    - Navigate the directory where the AWSIM repository was cloned to
![](image_7.png)

    - The project should be added to `Projects` tab in Unity Hub. To launch the project in Unity Editor simply click the `AWSIM` item
![](image_8.png)

    - The project is now ready to use
![](image_9.png)

!!! warning

    If you get the safe mode dialog when starting UnityEditor, you may need to install openssl.

    1. download libssl  
    `$ wget http://security.ubuntu.com/ubuntu/pool/main/o/openssl1.0/libssl1.0.0_1.0.2n-1ubuntu5.13_amd64.deb`
    2. install  
    `sudo dpkg -i libssl1.0.0_1.0.2n-1ubuntu5.11_amd64.deb`

### Import external packages

To properly run and use AWSIM project in Unity it is required to download map package which is not included in the repository.

1. Download and import `Japan_Tokyo_Nishishinjuku.unitypackage`

    [Download Map files (unitypackage)](https://github.com/tier4/AWSIM/releases/download/v1.1.0/Japan_Tokyo_Nishishinjuku.unitypackage){.md-button .md-button--primary}

2. In Unity Editor, from the menu bar at the top, select `Assets -> Import Package -> Custom Package...` and navigate the `Japan_Tokyo_Nishishinjuku.unitypackage` file.
![](image_10.png)
![](image_11.png)
3. `Nishishinjuku` package has been successfully imported under `Assets/AWSIM/Externals/`directory.
![](image_12.png)

!!! info

    The Externals directory is added to the `.gitignore` because the map has a large file size and should not be directly uploaded to the repository.

## Run the demo in Editor

The following steps describe how to run the demo in Unity Editor:

1. Open the `AutowareSimulation.unity` scene placed under `Assets/AWSIM/Scenes/Main` directory
2. Run the simulation by clicking `Play` button placed at the top section of Editor.
![](image_13.png)
<br><br><br><br>
