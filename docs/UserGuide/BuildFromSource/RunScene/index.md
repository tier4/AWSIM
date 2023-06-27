Before following through with this section make sure you setup *AWSIM* *Unity* project in accordance with section [Setup Unity Project](../SetupUnityProject/).

# Run the scene

To run the *AWSIM* scene in *Unity* with *Autoware* follow these steps:

1. Open the `AutowareSimulation.unity` scene placed under the following directory:

    ```
    Assets/AWSIM/Scenes/Main/AutowareSimulation
    ```

    ![unity open scene](open_unity_scene.gif)

1. Run the simulation by clicking `Play` button placed at the top section of Editor:

    ![unity play scene](unity_play.gif)

1. Check whether the *AWSIM* has started communication channels

    1. Source *ROS* if you haven't already:

        ```bash
        source /opt/ros/humble/setup.bash
        ```

    1. Run the following command listing all open communication channels (topics)

        ```bash
        ros2 topic list
        ```

        ![ros2 topic list](demo_topic_list.gif)

        !!! warning "No communication channels"
            If the command above does not produce any output, then something in your configuration is broken.

            For specific information on possible fixes please visit [the trouble shooting page](../../../DeveloperGuide/TroubleShooting/).

1. Launch the *Autoware* (the same way as [with AWSIM Demo](../../Installation/RunDemo/#run-with-autoware)) by executing the commands with your own path to the `map files` and `Autoware workspace`:

    ```bash
    source /opt/ros/humble/setup.bash
    source <autoware_workspace_path>/install/setup.bash
    ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit map_path:=<mapfiles_dir_path>
    ```

!!! success
    The *Autoware* that has been started and communicating properly with *AWSIM* should look like this:

    ![autoware](autoware.png)
