# Push the Ball
The goal is to hit the ball to push it as far as possible from **any part** of the table.
The ball will always spawn at the same place. The robot can be moved with position or cartesian control.
The trajectories should be collision free and max allowed velocity is 10 (rad/s).  

The resulting distance is checked after 2 seconds.

![Push the Ball!](https://raw.githubusercontent.com/rustlluk/pycub/dev/exercises/exercise_1/exercise_1.gif)

## Task
Implement `push_the_ball` function in [exercise_1.py](https://github.com/rustlluk/pycub/blob/master/exercises/exercise_1/exercise_1.py) that will push the ball as far as possible from the table.  
  - the function takes one argument:
    - client - instance of pycub class that controls the simulation
      - do not create new one in your code! 

## Scoring
 - points for distance are computed as: min(3, distance*2)
   - i.e., you get max 3 points if you push the ball 1.5m away

## Requirements:
**Those apply mainly for** [exercise_1_tester.py](https://github.com/rustlluk/pycub/blob/master/exercises/exercise_1/exercise_1_tester.py) **to work correctly**:
  - do not create new client instance, use the one that is passed as an argument
  - do not rename the function or file
  - **Do not** introduce artificial delays in the code, e.g., sleep() or using update_simulation() after the movement is done  

**Those apply so that you fulfill the exercise as intended:**
  - Trajectories must be collision free with the table (self-collisions of the robot are allowed)
  - Max allowed velocity is 10 (rad/s)
  - **Do not** turn of gravity