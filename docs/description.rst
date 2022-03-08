
The Armstron package allows you to  This ROS package and associated GUI make use of the 6-axis force/torque sensor on the Universal Robot e-series to apply complex loads to things. We can essentially perform tests similar to an Instron Uniaxial Testing machine, but in all axis.


.. dropdown::  :fa:`eye,mr-1` Table of Contents

   .. toctree::
      :caption: Documentation
      :maxdepth: 2
      :titlesonly:

      Home <self>
      quickstart
      examples/index
      reference/index

   .. toctree::
      :caption: Appendix

      contributing
      genindex



Install
=======
1. Clone this package to the `src` folder of your catkin workspace
2. In the root folder of your workspace, install dependencies:
   - ``rosdep install --from-paths src --ignore-src -r -y``
3. Navigate to the armstron package folder and install a few extra non-ROS python requirements:
   - ``cd src/armstron/armstron``
   - ``pip install -r requirements.txt``
5. Navigate back to the workspace folder, and build your workspace (``catkin_make``)



Explore the Examples
====================

Check out the :ref:`Examples <examples>`, or run any of the launch files in the ``armstron/launch`` folder.




Links
=====

- **Documentation:** `Read the Docs <https://armstron.readthedocs.io/en/latest/>`_
- **Source code:** `Github <https://github.com/harvard-microrobotics/armstron>`_


Contact
=======

If you have questions, or if you've done something interesting with this package, get in touch with `Clark Teeple <mailto:cbteeple@g.harvard.edu>`_, or the `Harvard Microrobotics Lab <https://www.micro.seas.harvard.edu/>`_!

If you find a problem or want something added to the library, `open an issue on Github <https://github.com/harvard-microrobotics/armstron/issues>`_.




Used In...
===========

Armstron will enable many figure works!

