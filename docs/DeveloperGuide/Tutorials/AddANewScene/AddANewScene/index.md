!!! Draft-note
    - Add a ClockPublisher (ros2 topic, qos)
    - Add MainCamera
        - Camera component (**screen**, hyperlinks, configure fov)
        - Follow Camera Script (target, parameters, example - **screen**)
    - Add a Vehicle (exists prefabs, positioning on the scene, **gifs**)
    - Add a Scene Manager Script (hyperlink)
    - Add a Environment prefab (hyperlink)

## Add a ClockPublisher
1. Add a child *Object* to a *Simulation*.

    <!-- ![clock publisher object](clock_publisher_add_object.gif) -->
    <img src=clock_publisher_add_object.gif width=600px>

1. Click 'Add Component' button, search for `Clock Publisher` Script and select it.

    ![clock publisher add script](clock_publisher_add_component.gif)

    ![clock publisher search](clock_publisher_search.png)

1. Finally you can configure your Clock Publisher by changing the topic or Quality of Service (qos) settings.

    !!!danger
        Change these settings only when you know what you are doing, otherwise you can break the communication between the simulation and autonomous driving software.

## Add MainCamera
<!-- TODO -->

## Add a Vehicle
<!-- TODO -->

## Add a Scene Manager Script
Tutorial on how to add a `SceneManager` Script can be found [here](../AddASceneManager/).

## Add an Environment prefab
<!-- TODO -->
