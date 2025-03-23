# Smooth movements
The task is to perform smooth movements with the arms of the robot. The robot should be able to move in a straight 
line or a circle with a smooth trajectory. In other words, the movement should be continuous and without jerks.

![Smooth Movements](https://raw.githubusercontent.com/rustlluk/pycub/dev/exercises/exercise_2/exercise_2.gif)

## Task
Implement function move() in [exercise_2.py](https://github.com/rustlluk/pycub/blob/master/exercises/exercise_2/exercise_2.py)
that will move the robot's arm in a straight line or a circle.  

   - the function takes four arguments:
     - client - instance of pycub class that controls the simulation
     - action - string "line" or "circle"
     - axis - list of ints. For “circle” it the length is always 1. For “line” the length can be from 1-3. 
       Individual numbers on the list are axes along which the robot should move. 
       For example, [0, 1] means that the robot should move in x- and y-axis.
     - r - list of floats. The same length as axis. Number in metres. For “circle” it is the radius of the circle. 
       For “line” it is the length of the line in the given axis.
   - example arguments:
     - action=“circle”, axis=[0], r=[0.01] - the end-effector should move in a circle around the x-axis with a 
       radius of 0.01m
     - action=“line”, axis=[1], r=[-0.05] - the end-effector should follow a line in the y-axis with a 
       distance of -0.05m
     - action=“line”, axis=[0, 1], r=[0.05, -0.05] - the end-effector should follow a line in the x-axis for 0.05cm 
       and y-axis for -0.05m
       - it should be simultaneous movement in both axes, i.e., it will be one line
   - the function **have to** return two arguments: start and end pose of the trajectory
     - both should be obtained with `client.end_effector.get_position()`

## Scoring
 - you will be given 10 random test cases
   - each will be awarded with 1 points
 - both circle and line movements will be evaluated with three sub-criteria:
   - circle:
     - standard deviation of each point of the circle to center; must be < r*0.125
     - difference between expected and real radius; must be < r*0.1
     - mean distance of all points from the expected plane; must be < 0.1
   - line:
     - difference from real length to expected length; must be < 0.0075
     - angle between the expected and real line; must be <0.1
     - mean distance of all points from the expected line; must be < 0.01
 - **the smoothness of the movements will is not checked by the automatic evaluation tool!**
 - **the code should be as general as possible to work for any allowed input.** Meaning that it should not be, for example, 
   a long if/else (switch) statement for each axis etc.

## Requirements:
**Those apply mainly for** [exercise_2_tester.py](https://github.com/rustlluk/pycub/blob/master/exercises/exercise_2/exercise_2_tester.py) **to work correctly**:
  - do not create new client instance, use the one that is passed as an argument
  - do not rename the function or file
  - **return two arguments:** start and end pose of the trajectory of type utils.Pose

**Those apply so that you fulfill the exercise as intended:**
  - **Do not** turn of gravity
  - Make the movement as smooth as possible