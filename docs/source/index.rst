pyCub Studio handbook
=====================

.. container:: handbook-intro

   .. rst-class:: introduction

      Learn to control the iCub humanoid in simulation. Start with a wave,
      write your own controller, then test it against the course exercises.

   .. image:: ../static/images/icub.png
      :class: handbook-robot
      :alt: Simulated iCub humanoid waving behind a table with a red ball

.. rst-class:: handbook-sections

* :doc:`Using Studio <studio>`

  Choose a mode, set up your workspace and run a simulation.

* :doc:`Programming the robot <programming>`

  Move joints, read sensors and inspect Python calls.

* :doc:`Example controllers <examples>`

  Follow working joint, Cartesian and skin examples.

* :doc:`Course exercises <exercises>`

  Practice motion control, gaze and grasping.

* :doc:`Project README <readme>`

  Installation, Docker setup, features overview and citation.

* :doc:`API reference <autoapi/index>`

  Look up classes, functions and their arguments.

.. only:: html

   Offline manual: `Download PDF handbook <pycub.pdf>`_

.. toctree::
   :hidden:
   :maxdepth: 2

   studio
   programming
   examples
   exercises
   readme
   autoapi/index
