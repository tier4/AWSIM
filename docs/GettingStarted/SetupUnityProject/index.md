# Setup Unity Project

!!! info

    Users are recommended to do the Quick start demo tutorial first. [Quick Start Demo](../QuickStartDemo/index.md)

This page is a tutorial for setting up a Unity project by cloning the AWSIM repository.

## How to setup on ubuntu

### 1. Preparation

=== "Ubuntu"
    1. Prepare a desktop PC with Ubuntu 20.04 installed. See [Check the required specifications](../QuickStartDemo/#1-check-the-required-specifications). (PC spec requirements vary depending on simulation content.)
    2. Install Nvidia driver and Vulkan Graphics API. See [Install Nvidia GPU driver](../QuickStartDemo/#3-launch-awsim-demo-binary).
    3. Install [git](https://git-scm.com/).

=== "Windows"
    1. TODO: write

### 2. Install Unity 2021.1.7f1
AWSIM's Unity version is currently **2021.1.7f1**


1. Install UnityHub to manage UnityProject. Go to [https://unity3d.com/get-unity/download](https://unity3d.com/get-unity/download) to download `UnityHub.AppImage`.
![](image_1.png)
2. Install Unity 2021.1.7f1 via UnityHub. 
```
./UnityHub.AppImage
```
![](image_2.png)
![](image_3.png)
![](image_4.png)    
installed.
![](image_5.png)    




### 3. Clone AWSIM repository
```
git clone git@github.com:tier4/AWSIM.git
```

### 4. Open AWSIM Unity project
1. Launch UnityHub.
```
./UnityHub.AppImage
```
2. Press Open button.
![](image_6.png)
![](image_7.png)
![](image_8.png)
![](image_9.png)

### 5. Import Environment custom package
1. Download and Import `Japan_Tokyo_Nishishinjuku.unitypackage`. this is environment 3d map.

    [Download Map files (unitypackage)](https://github.com/tier4/AWSIM/releases/download/v1.0.0/Japan_Tokyo_Nishishinjuku.unitypackage){.md-button .md-button--primary}

2. Import `Japan_Tokyo_Nishishinjuku.unitypackage`. Select from the menu bar at the top `Assets -> Import Package -> Custom Package`. 
!!! info

    Import the Custom Package into the gitignore directory because the environment map has a large file size.

![](image_10.png)
![](image_11.png)
![](image_12.png)
3. Check that `Nishishinjuku` Package has been imported under `Assets/AWSIM/Externals/`. (`Externals` directory is gitignored.)

### 6. Run demo scene
1. Open `Assets\AWSIM\Scenes\Main\AutowareSimulation.unity`
2. Run `AutowareSimulation.unity`
![](image_13.png)
<br><br><br><br>
