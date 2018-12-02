# RoboticsMini



Code explanation:

Our commands:

(1) In move forward: We use laser to identify obstacles. 

(2) In turn around: We claculte the angle the robot should reach, and turning him around till he gets there.

(3)In order to identify the distance a color: We take a picture with the camera, make a make including just the relevant, after combining it with our picture we get a black picture with only the relevant colors contours. Then we identify the center of the object in the picture, and convert it to the 3D center of our world. We calculate the angle between the center of the world to the center of the obj and turn the robot around, so the robot will face the object. Then we use laser to measure the distance to the object in front of us.

(4)In order to find an object with the relevant color in the world: 
We use a loop, that continues till the robot is 50-100cm close to the object:
- The robot look for the relevant color using (3).
- If he found the colors, he moves forward to it 50 cm using (1).
- If he doesn't find, the robot turns 360 degrees searching the color at its sides: if still didn't find it, he move forward using (1).
- If he can't move forward, he turns around 100 degrees. 



In order to run:
1. In launch folder, in turtlebot3_bgu_world.launch, edit the world path to the location of the world in you computer under "value".
2. Run export TURTLEBOT3_MODEL=burger
3. Run gazibo with the command: roslaunch assignment2 turtlebot3_bgu_world.launch
4. In a new terminal run: catkin_make, TURTLEBOT3_MODEL=burger, and then run - rosrun assignment2 get_command_from_user.py 

Have fun

