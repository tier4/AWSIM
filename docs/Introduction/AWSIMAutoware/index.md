!!! draft-note
    <!-- TODO: remove when done -->
    (how the AWSIM-Autoware connection architecture look like - **screens**, hyperlink to list of used topics)

## Connection architecture
<!-- Tutaj mozna zrobic prosty graf co skad jest brane, plus screen z topickami -->

[**ROS 2 topics**](../../UserGuide/ProjectGuide/Ros2TopicList/)


## Features
!!! draft-note
    <!-- TODO: remove this note when done -->

    - Engagement (driving straight, turning - with a view of the intersection shape, **gifs**)
    - Traffic light recognition
        - Stopping at a red and yellow lights (**gifs**)
        - Running a green and yellow lights (**gifs**)
    - Interaction with vehicles
        - Following (with suddenly stop, **gifs**)
        - Right-of-way at the intersection (turning right, forcing->stopping, **gifs**)
        - Cut-in situation (**gifs**)
    - Interaction with pedestrians
        - Right-of-way at a crosswalk (with red light and forcing, **gifs**)
        - Pedestrian on the road beyond the crosswalk (**gifs**)
    - Detecting bad behaviors (**Future**, **gifs)**

### Engagement
Autoware in combination with AWSIM can participate in common road situations

- Drive straight in lane on an intersection

    <!-- ![go straight](straight_green.gif) -->

    <video width="1920" controls autoplay muted loop>
    <source src="DRIVE_STRAIGHT.mp4" type="video/mp4">
    </video>

- Turn on an intersection

    <!-- ![turn](turn_green.gif) -->

    <video width="1920" controls autoplay muted loop>
    <source src="DRIVE_TURN.mp4" type="video/mp4">
    </video>

### Traffic light recognition
AWSIM allows Autoware to recognize traffic lights and act accordingly

- Stop at a red light

    <!-- ![stop on red](stop_red.gif) -->

    <video width="1920" controls autoplay muted loop>
    <source src="WAIT_RED.mp4" type="video/mp4">
    </video>

- Drive at green light

    <!-- ![run on green](straight_green.gif) -->

    <video width="1920" controls autoplay muted loop>
    <source src="DRIVE_GREEN.mp4" type="video/mp4">
    </video>

- Stop at yellow light

    <video width="1920" controls autoplay muted loop>
    <source src="WAIT_YELLOW2.mp4" type="video/mp4">
    </video>

- Still drive at yellow light (only when it is too late to stop)

    <!-- ![run on yellow](straight_yellow.gif) -->

    <video width="1920" controls autoplay muted loop>
    <source src="DRIVE_YELLOW2.mp4" type="video/mp4">
    </video>

### Interaction with vehicles
<!-- TODO -->

### Interaction with pedestrians
<!-- TODO -->