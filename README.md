# Dataset Aracati 2017

This project provides ROS nodes to support the play dataset ARACATI 2017.

The data was collected on the marina of Yacht Club of Rio Grande - Brazil by a Remote Operated Vehicle LBV 300-5 from Seabotix using a forward looking sonar Blue View P900-130 from Teledyne BlueView. A floating board holds the vehicle underwater and a DGPS and a GoPro on the surface. The vehicle drives the floating board and the GPS records the ground truth localization of the vehicle. The data also has a compass, USBL localization, and surface images from GoPro.

The sonar maximum range was set to 50 meters it covers 130 degrees forward the vehicle. 


Preview of the dataset available at: 


[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/RaooXyLv3-s/0.jpg)](https://youtu.be/RaooXyLv3-s)


## Instructions

This project must be cloned on the src folder of your ROS workspace.

1. Access src folder of your workspace:

`$ cd {path to your workspace}/src/`

2. Clone this repository on the src folder of your ROS workspace:

`$ git clone https://github.com/matheusbg8/aracati2017.git`

3. Download the bag file of dataset ARACATI 2017 on [this address](https://drive.google.com/file/d/1dbpfd3jElTdHmnceKE5RL8hzU-BDYaW-/view?usp=sharing) and save on folder bags of this repository.

4. Build your workspace:

`$ catkin build aracati2017`

5. Source your workspace again to update the package:

`$ source ../devel/setup.bash`

 If you set your ros workspace on bashrc, you can just close and re-open your terminal.
 
6. Run the dataset: 

`$ roslaunch aracati2017 run.launch`

## Topics

* /aerial_img - A compressed aerial image with the sonar field of view and vehicle path.
* /cmd_vel - Command of velocity of the vehicle, the angular velocity in Z (heading) is estimated from the vehicle compass.
* /dgps - DGPS measurements
* /initialpose - Initial vehicle position. Published once on start of node /odom.
* /odom_pose - Vehicle pose estimated from topic /cmd_vel by node /odom.
* /pose_gt - DGPS position in meters regarding the first vehicle position.
* /son/compressed - Acoustic images from P900-130.
* /son_aerial - Sonar Field of View on aerial image publish by node /aerial_image.
* /surface/compressed - GoPro images on surface.
* /usbl - USBL measurements. When the ping fails the transponder position is published.
* /usbl_point - Vehicle position from USBL in meters regarding vehicle first position.

## ROS Nodes

The run.launch file runs the following two nodes:

* odom: Provides dead reckoning localization from the /cmd_vel topic . 
  - Subscribes on topic "/cmd_vel"
  - Advirtise topic "/odom_pose"

* aerial_image: Publish aerial images and plot vehicle path
  - Subscribes on topoics: /odom_pose and /pose_gt
  - Advertise topics: /aerial_image and /son_aerial

