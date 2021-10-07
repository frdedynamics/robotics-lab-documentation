.. _Apply-on-Real-Turtlebot:

***************************************
Apply on Real Turtlebot
***************************************
So far we have done everything on the PC but how to use a *real* robot and apply all the knowledge on it?

Theme (Lab5)
==============================================

#. Turtlebot tutorials
#. ROS
#. Turtlebot
#. SLAM
#. Navigation

Equipment (Lab5)
==============================================
#. PC with ROS installed
#. OPTIONAL: A real turtlebot
#. Turtelbot tutorials from `robotis.com <https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/>`_

Before the lab (Lab5)
==============================================
#. Do the previous lab.

#. Try to get together in a group of 2-4 people.

#. To use a real turtlebot, find your way to the HVL Robotics lab.


Report (Lab5)
==============================================
There is no need to hand in a report after this lab.

Signed attendance will suffice as approved lab exercise.

Tasks (Lab5)
==============================================
#. `SLAM`_
#. `Navigation`_

.. warning::
    Batteries are dangerous.


Setting up the Virtual Machine Network
==============================================
In order to communicate with your Turtlebot in real world, you need to do some network settings. In the official Turtlebot tutorials, the procedure is well explained. However, for those who are using virtual machine, the process might look different. This section is a tailored version for our use case.


VM settings
--------------

#. Select your virtual appliance on the left bar 
#. Click *Edit virtual machine settings*
#. Select *Network Adapter* 
#. Select the first option *Bridged: Connected directly to the physical network* also check the *Replicate physical network connection state*
#. Go to *Configure Adapters* and ONLY select the wireless adapter which your PC has. In our case it is "Killer(R) Wi-Fi 6 AX1650 160MHz Wireless Network Adapter"
#. Save everything and start your virtual machine.

   .. figure:: ../_static/images/ros/vm_bridge_settings.png
          :align: center


Connect to internet
---------------------
#. Make sure that you and your Turtlebot are connected to the same network.
#. Check if they are: `ping TURTLEBOT_IP`

.. note::
   It is better to set a static IP address in order not to change `ROS_MASTER_URI` settings after every restart. Go to your network settings in your virtual machine and set it to *Manual* from *DHCP*. 

   IP adress: 172.31.1.XXX
   where XXX is any number between 0-255 (excluding 0,1,131,132,133,134,135)
   Netmask: 255.255.255.0
   Gateway and DNS empty

    Check if your host machine and the guest machine in the same domain:

    - Windows host: Go to search bar at the bottom. Type `ipconfig`. Look at your wireless IPv4 IP number.
    - Linux host: Open a terminal and type `ifconfig`. Look at yout wlo section and look at your inet and netmask.
    - Mac host: Open a terminal and type `/sbin/ifconfig`

    Afterwards, go to your virtual machine and open a terminal, type `ifconfig`. Look at yout wlo section and look at your inet and netmask. If the first 3 numbers of X.X.X.X are the same (172.31.1.xxx) then you are good.

    .. figure:: ../_static/images/ros/ifconfig.png
          :align: center

    .. figure:: ../_static/images/ros/ipconfig.png
          :align: center


Set Your Virtual Machine (Master)
------------------------------------

#. Open a terminal
#. Type `ifconfig` and write down your IP address down somewhere.
#. Type `gedit ~/.bashrc`
#. Add at the end `export ROS_IP=YOUR_IP_ADDRESS`
#. Save and exit
#. Close the terminal and use a new one when your need.

Set Your Turtlebot (Slave)
----------------------------

#. Open a terminal
#. Type `ssh pi@TURTLEBOT_IP`
#. Password: turtlebot
#. Make sure the green text on the terminal is not **rosuser@rosuser-VirtualBox** but it is **pi@raspberrypi**. Then type `nano ~/.bashrc`
#. Find `export ROS_MASTER_URI`
#. Change it with your IP address.
#. Hit Ctrl+X to exit editor mode in Nano.
#. Hit 'Y' to save.


_`SLAM`
==============================================
Follow the `SLAM tutorial <https://emanual.robotis.com/docs/en/platform/turtlebot3/slam>`_ to
do some SLAM'in.

Remember roslaunch turtlebot3_gazebo turtlebot3_world.launch from the previous lab?


_`Navigation`
==============================================
See `navigation tutorial <https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/>`_.


Questions
==============================================

#. What is SLAM? What hardware is used for this?
#. Did you use a real robot?
#. Which flavour of OS and ROS did you use?
#. What is a key difference between RViz and Gazebo?
#. Did you remember to have fun?
