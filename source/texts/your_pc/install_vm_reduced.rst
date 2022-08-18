.. _Install-VM:

****************************
Install VM
****************************
Virtual machines are basically some software which run just like a regular PC but without any physical components. They use the *host*'s hardware equipments as a *guest*. Since ROS works best on Linux based operating systems but we assume most of you have Windows PCs, we provide this ready-to-use solution for you to start ROS as smooth as possible.

There will be two components: 

#. The virtual appliance
#. A software to run the virtual appliance (VMware)

The admin password inside the virtual appliance is: **ros** 

Virtual Appliance
===================================
A virtual appliance is a copy of a working operating system and its programs. We provide you ready-to-use virtual copy of what you need. To download it, use `this link <https://hvl365.sharepoint.com/:u:/s/RobotikkUndervisningHVL/Eb_Fy_CWLGNFkK0zY8PSimoBwOSer6dVaL8LRUHWVQUTNQ?e=llEm2S>`_ with your HVL credentials. After downloading, extract the folder inside the .zip file.  

Virtual Appliance Player
===========================

A virtual appliance player is a software on which you can run your virtual appliance (aka. a virtual copy of a system). We suggest you to download VMware for this purpose.


VMware Install
------------------

You can download VMware here: `Windows/Linux download <https://www.vmware.com/products/workstation-player/workstation-player-evaluation.html>`_, `Mac download <https://www.vmware.com/products/fusion/fusion-evaluation.html>`_. After installing it you can import the virtual appliance by clicking **Open a Virtual Machine** and choose the .vmx file from inside the previously downloaded and extracted folder.

After you finished importing the virual appliance, go to **Edit virtual machine settings** and in Display settings enable **Accelerate 3D graphics** and choose recommended Graphics Memory from the dropdown box as shown in the pictures.

  .. figure:: ../../_static/images/ros/VM-settings.png
          :align: center

  .. figure:: ../../_static/images/ros/VM-settings2.png
          :align: center
          
Also make sure, that in the **Network Adapter** settings, under **Network connection**, **NAT** is selected, as shown in the pictures below.

  .. figure:: ../../_static/images/ros/vm_network_settings.PNG
          :align: center

When first opening the virtual machine, the following window will pop-up. Select **I Copied It** to continue.

  .. figure:: ../../_static/images/ros/vm_installation_popup.PNG
          :align: center

Troubleshooting
===========================

.. note::
  For those who don't use the given virtual copy and choose to install the necessary software and packages by themselves use the following links: `Ubuntu 18.04 LTS <https://releases.ubuntu.com/18.04/>`_, `ROS Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu>`_, `Necessary Turtlebot packages <https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/>`_, `MATLAB 2020a <https://se.mathworks.com/products/new_products/release2020a.html>`_.
