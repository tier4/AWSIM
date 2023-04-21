- Download and open project [90% current - including warning] (**gifs**)
- Download and import map files (**gifs**)
<!-- TODO copied old and fixed links except of one below -->

<!-- TODO update link -->
!!! info

    It is advised to checkout the [Quick Start Demo](../QuickStartDemo) tutorial before reading this section. 

This page is a tutorial for setting up a AWSIM Unity project.

## Environment preparation

### System setup

=== "Ubuntu 22"
    1. Make sure your machine meets the [required hardware specifications](/UserGuide/Installation/Prerequisites/#pc-specs).
        - *NOTE: PC requirements may vary depending on simulation contents which may change as the simulator develops*
    2. Prepare a desktop PC with Ubuntu 22.04 installed.
    2. Install [Nvidia drivers and Vulkan Graphics API](/UserGuide/Installation/AWSIMDemo/#running-the-awsim-simulation-demo).
    3. Install [git](https://git-scm.com/).

=== "Windows"
    1. Make sure your machine meets the [required hardware specifications](/UserGuide/Installation/Prerequisites/#pc-specs).
        - *NOTE: PC requirements may vary depending on simulation contents which may change as the simulator develops*
    2. Prepare a desktop PC with Windows 10 or 11 (64 bit) installed.
    3. Install [git](https://git-scm.com/).

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
    `$ wget http://security.ubuntu.com/ubuntu/pool/main/o/openssl1.0/libssl1.0.0_1.0.2n-1ubuntu5.11_amd64.deb`
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
