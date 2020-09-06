****************************
Select OS
****************************

We will help you out if you use Windows 10 (W10), Ubuntu or MacOS.
The main reason for needing Ubuntu is ROS and controlling stuff really fast.

If you are hesitant to move out of your operating system (OS) comfort zone, try out a new OS in a Virtual Machine (VM) first.


Options
=======================================
#. `Dual boot W10 and at least 90Gb Ubuntu 16.04 LTS`_ [**recommended**]
#. `Native Ubuntu 16.04`_
#. `W10 with Windows Subsystem for Linux (WSL) from Microsoft Store (Ubuntu 16.04)`_
#. `MacOS with Parallels VM`_ (if you can afford a Mac, you can afford Parallels)
#. `W10 with VirtualBox VM`_
#. `ROS on Windows`_
#. `ROS 2`_


.. warning::
	Gazebo requires a bit of resources. A decent CPU makes life better. It doesn't really utilize GPU's.


.. note::
	Ubuntu 16.04 is recommended for ROS Kinetic. Feel free to go for 18.04 and ROS Melodic at your own discretion.


.. note::
	Ubuntu 20.04 and ROS Noetic is not for you. Noetic uses Python 3 which the 
	turtlebot tutorials is not yet up to date 
	

.. note::
	We have not yet migrated to ROS 2.. soon though. Get excited!


.. note::
	ROS on windows exists, even in some form on MacOS, but we don't support this yet.
	ROS on windows is starting to look good and is on the road map for 2021.


_`Dual boot W10 and at least 90Gb Ubuntu 16.04 LTS`
==============================================================================
Get full use of Ubuntu, while keeping your "normal" W10. Select what to use 
each time you turn on your PC.

Recommended for everyone.

How to dual boot:

#. Download the Ubuntu version you want. You want `16.04 LTS <http://releases.ubuntu.com/16.04/>`_
#. Set aside 90Gb or more on your disk and install Rufus as shown at `tecmint.com <https://www.tecmint.com/install-ubuntu-alongside-with-windows-dual-boot/>`_
#. Don't bother with installation type "Something else", just go for "Install Ubuntu alongside Windows 10". Tut und kjør.


_`Native Ubuntu 16.04`
==============================================================================
Like the developer boss you are, you only need Ubuntu.

Recommended for 31337 h4x0r.

Most likely, you don't need installation instructions from here.


_`W10 with Windows Subsystem for Linux (WSL) from Microsoft Store (Ubuntu 16.04)`
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



