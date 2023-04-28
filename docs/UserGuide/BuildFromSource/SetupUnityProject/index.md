<!-- TODO update link, but idk what should it point to, because what was once one page now is like 2 sections full of pages -->
!!! info

    It is advised to checkout the [Quick Start Demo](../QuickStartDemo) tutorial before reading this section.

This page is a tutorial for setting up a AWSIM Unity project.

## System setup

=== "Ubuntu 22"
    1. Make sure your machine meets the [required hardware specifications](../../Installation/Prerequisites/#pc-specs).
        - *NOTE: PC requirements may vary depending on simulation contents which may change as the simulator develops*
    2. Prepare a desktop PC with Ubuntu 22.04 installed.
    3. Install [Nvidia drivers and Vulkan Graphics API](../../Installation/Prerequisites/#linux).
    4. Install [git](https://git-scm.com/).

=== "Windows"
    1. Make sure your machine meets the [required hardware specifications](../../Installation/Prerequisites/#pc-specs).
        - *NOTE: PC requirements may vary depending on simulation contents which may change as the simulator develops*
    2. Prepare a desktop PC with Windows 10 or 11 (64 bit) installed.
    3. Install [git](https://git-scm.com/).

## Open AWSIM project

To open the Unity AWSIM project in Unity Editor:

1. Make sure you have the AWSIM repository cloned
    ```
    git clone git@github.com:tier4/AWSIM.git
    ```

1. Launch UnityHub.

1. Open the project in UnityHub
    - Click the `Open` button in Unity Hub
![](open_unity_project1.gif)

    - Navigate the directory where the AWSIM repository was cloned to, select it and click `Open` in file explorer
![](open_unity_project2.gif)

    - The project should open automatically, if it doesn't it still should be added to `Projects` tab in Unity Hub. To launch the project in Unity Editor simply click the `AWSIM` item
![](launch_unity_project.gif)

    - The project is now ready to use
![](opened_project.png)

!!! warning

    If you get the safe mode dialog when starting UnityEditor, you may need to install openssl.

    1. download libssl  
    `$ wget http://security.ubuntu.com/ubuntu/pool/main/o/openssl1.0/libssl1.0.0_1.0.2n-1ubuntu5.11_amd64.deb`
    2. install  
    `sudo dpkg -i libssl1.0.0_1.0.2n-1ubuntu5.11_amd64.deb`

## Import external packages

To properly run and use AWSIM project in Unity it is required to download map package which is not included in the repository.

1. Download `Japan_Tokyo_Nishishinjuku.unitypackage`

    [Download Map files (unitypackage)](https://github.com/tier4/AWSIM/releases/download/v1.1.0/Japan_Tokyo_Nishishinjuku.unitypackage){.md-button .md-button--primary}

1. In Unity Editor, from the menu bar at the top, select `Assets -> Import Package -> Custom Package...`
![](import_unity_package1.gif)

1. Navigate the `Japan_Tokyo_Nishishinjuku.unitypackage` file and open it
![](import_unity_package2.gif)

1. In import window that appeared click the `Import` button to confirm and wait for the process to finish.
![](import_unity_package3.gif)

1. `Nishishinjuku` package has been successfully imported under `Assets/AWSIM/Externals/`directory.
![](externals_dir.png)

!!! info

    The Externals directory is added to the `.gitignore` because the map has a large file size and should not be directly uploaded to the repository.
