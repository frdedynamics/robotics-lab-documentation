.. _Install-VM:

****************************
Install VM
****************************
Virtual machines are basically some software which run just like a regular PC but without any physical components. They use the *host*'s hardware equipments as a *guest*. Since ROS works best on Linux based operating systems but we assume most of you have Windows PCs, we provide this ready-to-use solution for you to start ROS as smooth as possible.

There will be two components: 

#. A software to run the virtual appliance (VMware or VirtualBox)
#. The virtual appliance (an .ova file which contains everything you would need)



Virtual Appliance Player
===========================

A virtual appliance player is a software on which you can run your virtual appliance (aka. a virtual copy of a system). We suggest you to download VMware (or VirtualBox) for this purpose.


VMware Install
------------------

You can download VMware here: (`Windows/Linux download <https://www.vmware.com/products/workstation-player/workstation-player-evaluation.html>`_, `Mac download <https://www.vmware.com/products/fusion/fusion-evaluation.html>`_). After installing it you can import the .ova file by clicking **Open a Virtual Machine** and choose the .ova file you downloaded.

After you import .ova, which can take upto ~15 mins, go to **Edit virtual machine settings** and in Display settings enable **Accelerate 3D graphics** and choose recommended Graphics Memory from the dropdown box as shown in the pictures.

  .. figure:: ../../_static/images/ros/VM-settings.png
          :align: center

  .. figure:: ../../_static/images/ros/VM-settings2.png
          :align: center



.. warning::
   Troubleshooting:

   #. Please contact teaching assistants in installation related issues before ROS activities starts (Week-40). Unfortulately, we will not have any time during the lecture to solve installation problems so please make sure that all ROS-related installations are ready to use.
   #. To use host's peripherals like USB, webcam, etc: `Download VM extension package <https://www.oracle.com/virtualization/technologies/vm/downloads/virtualbox-downloads.html#extpack>`_



Virtual Appliance
===================================
A virtual appliance is a copy of a working operating system and its programs. We provide you ready-to-use virtual copy of what you need. To download it, use `this link <https://hvl365.sharepoint.com/:u:/s/RobotikkUndervisningHVL/Ed3rtAqeA3lAhUOkc4qREMkB4Awbm7UKbAtPxNdOaqBgpw?e=G4G2Vk>`_ with your HVL credentials. The followings are installed:

* `Ubuntu 18.04 LTS <https://releases.ubuntu.com/18.04/>`_
* `ROS Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu>`_
* Necessary `Turtlebot packages <https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/>`_.
* `MATLAB 2020a <https://se.mathworks.com/products/new_products/release2020a.html>`_

.. note::
  For those who don't use the given virtual copy should install the necessary software and packages by themselves manually. The links are provided as clickable.

