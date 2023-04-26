## Automatic position initialization

When launching Autioware after AWSIM has been launched the ego vehicle position should be automatically initialized in Autoware.
When this does not happen please see this [section](#manual-position-initialization).

The full process can be seen in the video below (open in full screen mode to see the details).

<video width="1920" controls>
  <source src="autoware_launch.webm" type="video/webm">
</video>

## Manual position initialization

In order to set position of ego vehicle manually

1. Click `2D Pose Estimate` button.

    ![pose estimate button](click_estimate.gif)

1. Click and drag on the road area position to set estimated position and orientation.

    ![pose estimate](estimate_pose.gif)

    !!! important
        The position and orientation to set in demo should be the same as in the video in [this section](#automatic-position-initialization).

1. Wait for the ego vehicle to appear.
