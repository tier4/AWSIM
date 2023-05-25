<!-- DM: sprawdz czy stan tego menu jest inny jak nie ma goala i jak jest, jesli tak to pokaz ta zmiane w 'Set a single goal' i napisz tutaj ze pojazd musi miec zainicjlizoana pozycje i zaplanowana trase (z goal) wtedy dziala AUTONOMY, mozesz dopisac tez ze jest mozliwosc opublikowania Engage na topicku (tak jak bylo to kiedys)  -->
<!-- widze ze ego u Ciebie sie nie wyswietla, uporzadkuj sobie rviza do takich nagran, warto zeby ego bylo.. ;/ -->

Before performing this step, make sure that the *Ego* position has been automatically initialized and the path to the goal has been planned according to this [section](../SetASingleGoal/).
# Enable self-driving
To start autonomous-driving:

1. Click `AUTO` button in the `OperationMode` of the `AutowareStatePanel`.
  ![Auto button](auto_button.gif)

1. As a result, `Ego` should begin to follow the planned path.

2. An examples of tracking a path to a single goal and with an added checkpoint are shown below.

!!! success 
    You already know how to move the `Ego` to one planned goal. In order to create a list of goals, you can use the `AutomaticGoal` plugin - it is described in this [section](../SetGoalList/).

!!! example
    An example of drive to single goal.
    <video width="1920" controls>
      <source src="set_goal.mp4" type="video/mp4">
    </video>
!!! example
    An example of drive to single goal with an added checkpoint. 
    <video width="1920" controls>
      <source src="goal_checkpoint.mp4" type="video/mp4">
    </video>
