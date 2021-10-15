****************************
Select OS
****************************

.. warning::

   This part has not been double checked. If you have any issues in any steps, please directly contact to your lecturer so that we can fix it in advance and update the website accordingly.

We will help you out if you use Windows 10 (W10), Ubuntu or MacOS.
The main reason for needing Ubuntu is ROS and controlling stuff really fast.

If you are hesitant to move out of your operating system (OS) comfort zone, try out a new OS in a Virtual Machine (VM) first.


Options
=======================================
#. `Dual boot W10 and at least 90Gb Ubuntu 18.04 LTS`_ [**recommended**]
#. `Native Ubuntu 18.04`_
#. `W10 with Windows Subsystem for Linux (WSL) from Microsoft Store (Ubuntu 18.04)`_
#. `MacOS with Parallels VM`_ (if you can afford a Mac, you can afford Parallels)
#. `W10 with VirtualBox VM`_
#. `ROS on Windows`_
#. `ROS 2`_


.. note::
	Gazebo requires a bit of resources. A decent CPU makes life better. It doesn't really utilize GPU's.


.. note::
	We use Ubuntu 18.04 and ROS Melodic in these tutorials.


.. note::
	Ubuntu 20.04 and ROS Noetic is not for you. Noetic uses Python 3 which the 
	turtlebot tutorials is not yet up to date 
	

.. note::
	We have not yet migrated to ROS 2.. soon though. Get excited!


.. note::
	ROS on windows exists, even in some form on MacOS, but we don't support this yet.
	ROS on windows is starting to look good and is on the road map for 2021.


_`Dual boot W10 and at least 90Gb Ubuntu 18.04 LTS`
==============================================================================

Get full use of Ubuntu, while keeping your "normal" W10. Select what to use 
each time you turn on your PC.

Recommended for everyone.

How to dual boot:

#. Download the Ubuntu version you want. You want `18.04 LTS <http://releases.ubuntu.com/18.04/>`_
#. Set aside 90Gb or more on your disk and install Rufus as shown at `tecmint.com <https://www.tecmint.com/install-ubuntu-alongside-with-windows-dual-boot/>`_
#. Don't bother with installation type "Something else", just go for "Install Ubuntu alongside Windows 10".
#. Follow the `Native Ubuntu 18.04`_ steps afterwards **for ROS and necessary packages installation part only**.


_`Native Ubuntu 18.04`
==============================================================================


You can download the `Ubuntu 18.04 LTS (Bionic Beaver) following this link <https://releases.ubuntu.com/18.04/>`_

After installing Ubuntu, it is highly recommended to du `sudo apt update` and `sudo apt upgrade`. If at some point you are asked to *upgrade Ubuntu*, please click no because we want to stick on Ubuntu 18.04 LTS version.

After your Linux system is up and running, you need to **install ROS and necessary packages**. 

#. `Install ROS Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu>`_. You will see different installation options in this website. You are supposed to follow `sudo apt install ros-melodic-desktop-full` one.

#. `Create ~/catkin_ws <http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment>`_

#. Install necessary packages that we use in our tutorials.

	#. The packages directly installed: (Note that some packages might be missing but you can install them as they are needed later)

	::

		sudo apt install ros-melodic-moveit
		sudo apt install ros-melodic-ros-controll ros-melodic-ros-controllers
		sudo apt install ros-melodic-moveit-simple-controller-manager
		sudo apt install ros-melodic-rqt*

	#. The packages which will be located in ~/catkin_ws/src (Note that you need to `cd ~/catkin_ws/src` before computing the following commands).

	::

		git clone -b melodic-devel https://github.com/ROBOTIS-GIT/open_manipulator.git
		git clone -b melodic-devel https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git
		git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
		git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git
		git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
		git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

		# After clonning the repositories above, compile your workspace
		rosdep install --from-paths src --ignore-src -r -y
		catkin_make
		source devel/setup.bash
		rospack profile



.. note::

   ONLY FOR ELE306 STUDENTS:
   You will need to install Matlab. Please follow this link `to install MATLAB 2020a <https://se.mathworks.com/matlabcentral/answers/518584-how-do-i-install-on-ubuntu>`_



_`W10 with Windows Subsystem for Linux (WSL) from Microsoft Store (Ubuntu 18.04)`
==================================================================================

.. role:: strike

Best of both worlds? :strike:`Definetly not. It gives you an Ubuntu terminal. Recommended for W10 users who want to try ROS and maybe suffer thru a lecture in terminal hell. There is no desktop or IDE, just terminals.`

Update summer 2019: Believe the hype, WSL is great! It`s like a VM, but it`s not, still, expect real connections to robots to be slow and file read/write to be slow (at least until WSL 2, late 2019). Furthermore, and this is really great, Visual Studio Code has a couple of very useful features for ROS and WSL, so you get a good IDE. 

How to WSL:

#. Follow Microsoft `instructions <https://docs.microsoft.com/en-us/windows/wsl/install-win10>`_
#. Install `VS code with remote WSL <https://code.visualstudio.com/docs/remote/wsl>`_
#. Install an X server, e.g. VcXsrv Windows X Server and finally add the following line to your ~/.bashrc: `export DISPLAY=:0`

Can't ping your W10 machine from a different machine? Check your windows firewall and private/public network settings.

VS code has some nice plugins for ROS and tools like linters.

Recommended for W10 users that don't need real time connection to real robots (you could always borrow a PC with ubuntu when you need that).

_`MacOS with Parallels VM`
==============================================================================
All good here. Except real time connection to devices (robots).

Recommended for Macintossers.

Don`t try to make your Mac dual boot, it`s not meant to be.

_`W10 with VirtualBox VM`
==============================================================================
Works, but not for real time connection to devices (robots).

Keep in mind that this is intense stuff for your RAM and CPU.
Unless you have 32Gb of RAM and a gazillion CPU cores, you should worry.

Recommended for W10 users who still are not convinced to make their PC dual boot. 

_`ROS on Windows`
==============================================================================
ROS can be installed on Windows using Chocolatey. The install is now easy enough
and turtlebot tutorials builds ok for the most part. We will support this soon.

_`ROS 2`
=============================================================================
ROS 2 is the best. We will migrate at some point. Either bridged or everything at once.
For us it all depends on what's supported by turtlebots and other equipment at the lab.



