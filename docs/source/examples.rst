Example controllers
===================

Open these examples in Normal Mode to run or modify them. Each example creates
its own client and selects a world. The excerpts below come from the scripts
installed with Studio.

Joint control
-------------

Set individual joint targets to position the arm and push the ball.

.. literalinclude:: ../../icub_pybullet/examples/push_the_ball_pure_joints.py
   :pyobject: push_the_ball
   :language: python

Cartesian control
-----------------

Move the end effector relative to its current pose.

.. literalinclude:: ../../icub_pybullet/examples/push_the_ball_cartesian.py
   :pyobject: push_the_ball
   :language: python

Skin contacts
-------------

Use the skin-test world to see which sensors activate as balls fall on the robot.

.. literalinclude:: ../../icub_pybullet/examples/skin_test.py
   :pyobject: main
   :language: python

Minimal simulation loop
-----------------------

:download:`Download the starter script <empty_example.py>` and open it in
Normal Mode. Add your robot commands after creating the client.

.. literalinclude:: empty_example.py
   :language: python
