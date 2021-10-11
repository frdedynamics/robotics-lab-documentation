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
.. note::
   VMware is an alternative to VirtualBox. You should have only one of them.

`VirtualBox <https://www.virtualbox.org/wiki/Downloads>`_ is one of the most well known virtual machine program but it has some limitations. Especially graphical-intensive programs (such as Gazebo) are lagging. As an alternative you can use VMware (`Windows/Linux download <https://www.vmware.com/products/workstation-player/workstation-player-evaluation.html>`_, `Mac download <https://www.vmware.com/products/fusion/fusion-evaluation.html>`_) just as in Virtualbox. You can import the same .ova file by clicking **Open a Virtual Machine** and choose the .ova file you downloaded.

After you import .ova, which can take upto ~15 mins, go to **Edit virtual machine settings** and in Display settings enable **Accelerate 3D graphics** and choose recommended Graphics Memory from the dropdown box as shown in the pictures.

  .. figure:: ../../_static/images/ros/VM-settings.png
          :align: center

  .. figure:: ../../_static/images/ros/VM-settings2.png
          :align: center

**PS:** If you have any problems in running Gazebo, plese `follow this link for troubleshooting <https://robocademy.com/2020/05/02/solved-opengl-issues-with-gazebo-and-vmware/>`_.

.. warning::
   Troubleshooting:

   #. Please contact teaching assistants in installation related issues before ROS activities starts (Week-40). Unfortulately, we will not have any time during the lecture to solve installation problems so please make sure that all ROS-related installations are ready to use.
   #. To use host's peripherals like USB, webcam, etc: `Download VM expansion package <https://hvl365.sharepoint.com/sites/RobotikkUndervisningHVL/Delte%20dokumenter/ROSTeaching/Oracle_VM_VirtualBox_Extension_Pack-6.1.22.vbox-extpack>`_


VirtualBox Install
---------------------
Follow the `official VirtualBox download link <https://www.virtualbox.org/wiki/Downloads>`_. Choose your operating system and download.


Virtual Appliance
===================================
A virtual appliance is a copy of a working operating system and its programs. We provide you ready-to-use virtual copy of what you need. If you use WMware as is recommended, use `this link <https://hvl365.sharepoint.com/:u:/s/RobotikkUndervisningHVL/Ed3rtAqeA3lAhUOkc4qREMkB4Awbm7UKbAtPxNdOaqBgpw?e=G4G2Vk>`_ to download a copy of the virtual appliance using you HVL credentials. If you are not using VMware, use `this link <https://hvl365.sharepoint.com/:u:/s/RobotikkUndervisningHVL/EVDejQL1F7lMtC8NMmHY8S0BhopabPJn68poCpHLvJIcCg?e=UXBG7Q>`_ to download the .ova file which can be imported in any player. The followings are installed:

* `Ubuntu 18.04 LTS <https://releases.ubuntu.com/18.04/>`_
* `ROS Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu>`_
* Necessary `Turtlebot packages <https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/>`_.
* `MATLAB 2020a <https://se.mathworks.com/products/new_products/release2020a.html>`_

.. note::
  For those who don't use the given virtual copy should install the necessary software and packages by themselves manually. The links are provided as clickable.
  **Update:** The *ready-to-use virtual copy* created by us has been updated on **26/08/2021**. For those who downloaded it before that date might have some missing libraries. Either download the newer version of the copy now or just install necessary packages as they are needed in the future.

To import the virtual copy of *everything*, which is an **.ova** file, run your Virtualbox.

  .. figure:: ../../_static/images/ros/vm-ss.png
          :align: center

The left bar under **Tools** must be empty for your case. From ** Files>Import Appliences** select your .ova file and press **Next** and then **Import**. Now, you will be able to see **ROS_MELODIC_VM** on the left bar.

Afterwards, you should select **Start** with the green arrow kew on the upper toolbar.
