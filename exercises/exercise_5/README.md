# Grasp It!
The goal of the task is to grasp a ball from the table. The exercise is divided into two parts:  
 - find the ball on a table in the image plane. The ball is in the visual field of the robot and is green.
 - grasp the ball. Use the position of ball in image plane to compote its 3D position and grasp it.

![Grasp It!](https://raw.githubusercontent.com/rustlluk/pycub/dev/exercises/exercise_5/exercise_5.gif)


## Task
Implement find_the_ball() and grasp() functions in [exercise_5.py](https://github.com/rustlluk/pycub/blob/master/exercises/exercise_5/exercise_5.py)
that will find and grasp the ball. The function are methods of a class Grasper, that contains several useful methods. 
After grasp, you should move the ball up such that it is at least 5cm above the table.

The class takes two arguments
   - client - instance of pyCub class that controls the simulation
   - eye - name of the eye in which the camera is placed; 'l_eye' or 'r_eye'
The two function have predefined arguments and returns:
   - find_the_ball() - takes no arguments and should return (u, v) - center of the ball in image plane
   - grasp() - takes one argument `center` (the 2D center returned by find_the_ball()) and should return 0 in case of successful grasp
You are free to add any method or variable to the class, but only find_the_ball() and grasp() will be called.

The class Grasper contains several helpful methods, e.g., to get RGB and Depth image; to deproject the 2D point; to close fingers.

The ball is always green (0, 255, 0) and it's radius is 2.5cm.

## Scoring
 - If the ball is 5cm above the table and in the hand of the robot after you return from grasp(), you passed the test

## Requirements
**Those apply mainly for** [exercise_5_tester.py](https://github.com/rustlluk/pycub/blob/master/exercises/exercise_5/exercise_5_tester.py) **to work correctly**:
  - do not create new client instance, use the one that is passed as an argument to Grasper class
  - do not rename the functions find_the_ball(), grasp() or the file
  - The ball must be 5cm above the table and in the hand of the robot after grasp() returns

**Those apply so that you fulfill the exercise as intended:**
  - **Do not** turn of gravity