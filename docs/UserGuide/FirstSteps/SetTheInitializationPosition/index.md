<!-- DM: prosilbym Ci tez zebys takei video nagrywal w blizej samego ego - zeby fullsceen nie byl konieczny, wgl skrocilbym to o te 20-30 sekund bezczynnosci  -->
## Automatic position initialization

When you run *Autoware* while the *AWSIM demo* is running in the background, then the *Ego* position should be initialized automatically in *Autoware*. It might take some time - depending on machine performance.

- As a result, `INITIALIZED` information should be visible in the `Localization` part of  the `AutowareStatePanel`:
    ![initialized](initialized.png)

- If the *Ego* position has not been initialized precisely please see this [section](#manual-position-initialization).


!!! success
    The correct result of the automatic position initialization should look like this.<br>
    Now you can proceed to the next step: [setting a single goal](../SetASingleGoal/).
    ![success](success.png)

!!! example
    An example of the full process of automatic position initialization.
    <video width="1920" controls>
    <source src="autoware_launch.webm" type="video/webm">
    </video>

!!! question
    If automatic initialization does not work, make sure you start *Autoware* with the correct path to the map files `<mapfiles_dir_path>`.
    ![uninitialized](uninitialized.png)

## Manual position initialization
<!-- DM: dobrze jakby pojazd na gifie byl widoczny w zlym miejscu i przeniosl sie na to co sugerujesz  -->
In order to set position of *Ego* vehicle manually

1. Click `2D Pose Estimate` button:

    ![pose estimate button](click_estimate.gif)

2. Click and drag on the road area position to set estimated position and orientation:

    ![pose estimate](estimate_pose.gif)

3. Wait for the system to correct *Ego's* location based on your defined position.

!!! success
    The correct result of the precise position initialization should look like this.<br>
    Now you can proceed to the next step: [setting a single goal](../SetASingleGoal/).
    ![manual_success](manual_success.png)

<!-- 
    !!! important
        The position and orientation to set in demo should be the same as in the video in [this section](#automatic-position-initialization). -->
