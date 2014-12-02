neato_carnival
==============

Code to let Neato robots go to a carnival.

Installing AR_pose (for fiducial recognition):

sudo apt-get install freeglut3-dev
cd ~/catkin_ws/src
git clone git@github.com:arlolinscope/ar_tools.git
cd ~/catkin_ws/src/ar_tools
rosmake
cd ~/catkin_ws
catkin_make


