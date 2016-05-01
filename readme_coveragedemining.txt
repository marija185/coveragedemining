Make catkin folder, e.g. rhcnavigation:
mkdir rhcnavigation
cd rhcnavigation
mkdir src
cd src
catkin_init_workspace
git clone https://github.com/amor-ros-pkg/rosaria.git
git clone https://github.com/marija185/movingobstaclesrhc.git
git clone https://github.com/marija185/mapsforsimulatorstage.git
git clone https://github.com/marija185/coveragedemining.git
cd ..
catkin_make
set up the path:
cd devel
source setup.bash
(put it into ~/.bashrc)



necessary packages: navigation_stage (for simulating), amcl, map-server, libaria-dev, sicktoolbox_wrapper

                   
compiling:
catkin_make

run the simulation
roslaunch movingobstaclesrhc clearmapstage.launch (starts only map)
roslaunch movingobstaclesrhc galerijatest.launch  (make sure navigation is commented, starts only map)
roslaunch coveragedemining startcoveragesim.launch (starts coverage)


run the robot





