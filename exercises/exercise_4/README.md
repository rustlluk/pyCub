# Resolved-Rate Motion Control 
The goal of this task is to implement a Resolved-Rate Motion Control (RRMC) controller that moves the limbs away
from collision using feedback from the artificial skin. There will be ball falling on the robot and you should move 
the correct body part away from it, i.e., move it against the normal of the contact. 

<table>
     <tr>
       <td><img src="https://raw.githubusercontent.com/rustlluk/pycub/master/exercises/exercise_4/gifs/exercise_4_0.gif" alt="Leg"></td>
       <td><img src="https://raw.githubusercontent.com/rustlluk/pycub/master/exercises/exercise_4/gifs/exercise_4_1.gif" alt="Arm"></td>
     </tr>
     <tr>
       <td><img src="https://raw.githubusercontent.com/rustlluk/pycub/master/exercises/exercise_4/gifs/exercise_4_2.gif" alt="Leg and arm"></td>
       <td><img src="https://raw.githubusercontent.com/rustlluk/pycub/master/exercises/exercise_4/gifs/exercise_4_3.gif" alt="Leg and two arms"></td>
     </tr>
</table>

## Task
Implement process() function in [exercise_4.py](https://github.com/rustlluk/pycub/blob/master/exercises/exercise_4/exercise_4.py)
that will control the robot using RRMC. This time, the function is a method of a class. The reason is the option
to store you own variables in the class.
   - the class takes one argument:
     - client - instance of pycub class that controls the simulation
   - the function should control joints using RRMC -> using velocity control
   - you **should not** call `update_simulation()` in any of your code

You will be given four different scenarios with different body_parts being hit by the ball and with different number of balls.
You should move every skin part that is in collision with any ball! But always use one contact per skin part, i.e., 
you should find the biggest cluster of activated taxels (skin points) on each skin part.

The tester script [exercise_4_tester.py](https://github.com/rustlluk/pycub/blob/master/exercises/exercise_4/exercise_4_tester.py)
will run all four tests in sequence from the easier one. You need to manually close the visualization window after each test.

## Scoring
There is no automatic evaluation for this task. But basically the task will be correctly fulfilled if the robot moves 
away from the ball. You can consult the video on top of this README to see possible outcomes. However, keep in mind
that those are not the only correct solutions as the movements depends on the parameters selected in RRMC.

## Requirements
**Those apply mainly for** [exercise_4_tester.py](https://github.com/rustlluk/pycub/blob/master/exercises/exercise_4/exercise_4_tester.py) **to work correctly**:
  - do not create new client instance, use the one that is passed as an argument to the RRMC class
  - do not rename the function, file or class
  - **do not call** `update_simulation()` in any of your code

**Those apply so that you fulfill the exercise as intended:**
  - **Do not** turn of gravity