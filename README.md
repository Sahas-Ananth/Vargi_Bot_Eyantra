# Vargi Bot e-Yantra 

This is a ROSject for the competition [e-Yantra](https://portal.e-yantra.org/) (2020-2021) for the theme **_"Vargi Bots"_** from team #2195.

The problem statement for **_"Vargi Bots"_** is 
>Inspired by this visualisation of Industry 4.0, the current edition of the e-Yantra Robotics Competition features a theme called ‘Vargi-Bots’. Vargi is taken from a Sanskrit word, Vargikaran (वर्गीकरण) which means to separate objects based on their category. The theme is set in the abstraction of a warehouse management system designed in Gazebo, which is a 3D dynamic simulator used to efficiently simulate robots in complex environments.

>The arena is an automated warehouse setting where essential packages are required to be sent out to different parts of a city. Since Industry 4.0 heavily focuses on automation here the warehouse will only consist of two industrial robotic arms which will be used by the teams. As the requirements are sent to the warehouse, one robotic arm will identify the packages from a shelf and place them on a conveyor belt and the other robotic arm at the end of the conveyor belt will pick these objects from the conveyor and place them into bins. Each bin represents a destination for the package. As the packages are sent out from the warehouse there will also be alerts sent to the user via email notifying them about the package being shipped from the warehouse.

>The packages to be delivered have their own priorities. Packages having a higher priority are intended for a natural disaster or a pandemic situation. Other packages with lower priorities are for general purposes. Similar to a conductor in an orchestra, in this theme, the participants have to design their own conductor (controller) for their warehouse to make smart decisions in order to deliver high priority packages as quickly as possible.

## Prerequisites

Before you begin, ensure you have met the following requirements:

* [Ubuntu 18.04 Bionic Beaver](https://releases.ubuntu.com/18.04/)
* Python 2.7.17 install by `$ sudo apt install python`
* [ROS Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu)
* Catkin tools install by `$ sudo apt-get install python-catkin-tools`

## Installing Vargi Bot

To install Vargi Bot, follow these steps:

Ubuntu 18.04:
```
$ mkdir -p catkin_ws/src 
$ cd catkin_ws/src
$ git clone https://github.com/Sahas-Ananth/Vargi_Bot_Eyantra.git
$ cd ..
$ catkin init
$ catkin build
```

<!-- ## Using Vargi Bots

To use Vargi Bots, follow these steps:

```
$ roslaunch vargi_bots display.launch
``` -->

## Contributors

Thanks to the following people who have contributed to this project:

* [@Sahasrajit A.](https://github.com/Sahas-Ananth) 💻
* [@Nitheesh Babu G.S.](https://github.com/gs-niteesh) 💻
* [@Vishal Balaji](https://github.com/The-SocialLion) 💻
* [@Promoth N](https://github.com/pro-07)💻

## Contact

If you want to contact me you can reach us contact: 
* sahas_ananth@outlook.com.
* niteesh.gs@gmail.com
* vishalsivaraman5@gmail.com
* promoth.n.2018.ece@rajalakshmi.edu.in
