# image_transport_profiler
Stress tests the ROS network to find out the maximum bandwidth at which images stream successfully

## Features of the profiler
* Set the resolution of image we wish to stream
* Set the rate we wish to stream at
* Set the size of the queue the publisher pushes to

## Coming soon
* Enable or disable compression

## Build and run
1. Clone the repo into the source folder in your workspace.
2. Run `catkin_make` in the workspace root.
3. Run `roslaunch image_transport_profiler profiler.launch` to start the stress test.
4. Use `rostopic list` to view topics. Use `rostopic bw <topic_name>` to view bandwidth.
