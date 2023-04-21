- UnityHub installation (**gifs**)
<!-- TODO gifs of command line?? -->
To install UnityHub please follow [these instructions](https://docs.unity3d.com/hub/manual/InstallHub.html#debian-or-ubuntu).

- Unity 2021.1.7f1 installation [70% current] (**gifs**)
<!-- TODO copied old, but it is outdated, UnityHub now comes as a package from apt, not an appImage,
so all instructiona are wrong -->

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
        - *NOTE*: If the installation process has not started after clicking the green button (image above), please copy the hyperlink (by rightclicking the button and selecting `Copy link address`) and add it as a argument for Unity Hub app. An example command:
        ```
        ./UnityHub.AppImage unityhub://2021.1.7f1/d91830b65d9b
        ```

    - After successful installation the version will be available under the `Installs` tab in Unity Hub.
![](image_5.png)
