This section describes the automatic and manual process of initializing the vehicle position in *Autoware*.

## Automatic position initialization

When you run *Autoware* while *AWSIM* demo is running in the background, then the vehicle (*Ego*) position should be initialized automatically in *Autoware*. It might take some time - depending on machine performance.

- As a result, `INITIALIZED` information should be visible in the `Localization` part of  the `AutowareStatePanel`:
  
    ![initialized](initialized.png)

- If the *Ego* position has not been initialized precisely please see this [section](#manual-position-initialization).


!!! success
    The correct result of the automatic position initialization should look like this.<br>
    Now you can proceed to the next step: [setting a single goal](../SetASingleGoal/).
    ![success](success.png)

!!! example "Automatic position initialization example"
    ![automatic position initialization](spawn_vehicle.gif)

!!! question
    If automatic initialization does not work, make sure you start *Autoware* with the correct path to the map files `<mapfiles_dir_path>`. In this case, the state of panel `AutowareStatePanel` is as follows:<br>
    ![uninitialized](uninitialized.png)

## Manual position initialization
In order to set position of *Ego* vehicle manually

1. Click `2D Pose Estimate` button

    ![pose estimate button](click_estimate.gif)

2. Click and drag on the road area position to set estimated position and orientation.
Then wait for the *Ego* vehicle to localize itself.

    ![pose estimate](pose_estimate.gif)

!!! success
    The correct result of the precise position initialization should look like this.<br>
    Now you can proceed to the next step: [setting a single goal](../SetASingleGoal/).<br>
    ![manual_success](manual_success.png)
