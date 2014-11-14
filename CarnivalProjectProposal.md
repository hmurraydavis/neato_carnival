#Project: Neato Carnival
Anne LoVerso, Halie Murray-Davis, Kaitlin Gallagher

##Overview

We want to create an obstacle course for the Neato, where it will get placed in a world and locate and identify obstacles to complete.  After an obstacle has been located and identified, the Neato will navigate towards it and complete the obstacle and complete it through a pre-programmed controller.  When the obstacle is complete, the Neato will switch back to searching for another obstacle and reset the process.  The Neato will use computer vision to recognize each obstacle, and in conjunction with Lidar to help it complete the course.  We’ll use what we learned in mobile robotics to navigate the robot to complete the course.

###MVP:

Three fully-programmed obstacles that the Neato can navigate.  It will identify them through use of fiducials, and complete each obstacle completely.

###Stretch goals:
Some ways to extend the project include adding more obstacles.  We also want to look into using computer vision and object recognition to eliminate use of fiducials for identifying and locating obstacles.

##Frameworks/Algorithms

**Computer vision and fiducial recognition** - We’ll use what we learned in the computer vision module to identify fiducials, and possibly later using object recognition

**Control** - The robot will be controlled through an object-oriented finite-state machine, where example states might include “searching for obstacle” where it drives around looking for a fiducial, “identifying obstacle” where it finds a fiducial and classifies the obstacle it represents, and “completing obstacle”, which might instantiate a Controller object for that specific obstacle.

##Weekly Tasks
![Figure 1](/photos/scheduleImage.png)
Fig 1. Gantt chart with tasks and proposed timelines across project

##Obstacles to Success

Mechanical construction of some obstacles will take away from coding time
Getting a lot of obstacles that creates an interesting world for the Neato
Completely exhaustive code that covers all possible ways that a Neato could approach an obstacle, and making sure it knows how to complete the task

##Help Needed:

We’ll need some budget for the mechanical construction of some obstacles.  Additionally, be there to advise us on code, reading fiducials, and making suggestions on how to improve the project and make it even cooler.

##Specific Obstacle Ideas:

- Bridge
- Tunnel
- Knocking over colored cups/dominoes
- Weaving through cones
- Teeter totter
- Jousting or target practice (have a pole on top, knock something into a target, or push a box into place)
- Coloring booth (attach marker to Neato, color a pattern)
- Mine field (avoid red sticky notes)
- Slide down a slide on a towel





