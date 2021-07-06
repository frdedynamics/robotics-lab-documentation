.. _Install-VM:

****************************
Install VM
****************************

Virtual Machine Install
==========================
Follow the `official VirtualBox download link <https://www.virtualbox.org/wiki/Downloads>`_. Choose your operating system.

.ova import
--------------
We provide you ready-to-use virtual copy of what you need. Please download it using `this link <https://hvl365.sharepoint.com/:u:/r/sites/RobotikkUndervisningHVL/Delte%20dokumenter/ROSTeaching/ROS_MELODIC_VM_with_TB.ova?csf=1&web=1&e=hdnigs>`_. The followings are installed:

* `Ubuntu 18.04 LTS <https://releases.ubuntu.com/18.04/>`_
* `ROS Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu>`_
* Necessary `Turtlebot packages <https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/>`_.
* `MATLAB 2020a <https://se.mathworks.com/products/new_products/release2020a.html>`_

.. note::
  For those who don't use the given virtual copy should install the necessary software and packages by themselves manually. The links are provided as clickable.

To import the virtual copy of *everything*, which is an **.ova** file, run your Virtualbox.

  .. figure:: ../../_static/images/ros/vm-ss.png
          :align: center

The left bar under **Tools** must be empty for your case. From ** Files>Import Appliences** select your .ova file and press **Next** and then **Import**. Now, you will be able to see **ROS_MELODIC_VM** on the left bar.

Afterwards, you should select **Start** with the green arrow kew on the upper toolbar.

.. warning::
   Troubleshooting:

   #. Fullscreen
   #. Using host's peripherals like USB, webcam, etc: `Download VM expansion package <https://hvl365.sharepoint.com/sites/RobotikkUndervisningHVL/Delte%20dokumenter/ROSTeaching/Oracle_VM_VirtualBox_Extension_Pack-6.1.22.vbox-extpack>`_
