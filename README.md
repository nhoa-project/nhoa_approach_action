# nhoa_approach_action 
A ROS action to perform a "soft" following of an indicated tracked person. It uses move_base to send navigation goals. 


## Actions implemented

* **Approach**. A client of this action can sent an *ApproachGoal* with the id of the desired person to be followed. If the target id string is equals to "-1", the action will try to follow the first person indicated in the list of *hri_msgs/IdsList* message published in the topic */humans/bodies/tracked* (if any person is detected).


## Parameters

* **control_frequency**. Frequency in Hz of execution, publishing and checking of the status of the action (default to 1 Hz).
* **robot_frame**. frame of robot base (default: "base_link").
* **person_max_angle_diff**. If the angle (in radians) between the robot heading and the person position is higher than this parameter, the robot will turn to the person (default: 0.6 rads).
* **move_close**. Boolean to indicate whether to move closer to the person. If false, the robot will only turn on the spot to look at the person.
* **person_max_dist**. If the **move_close** parameter is True, the robot will move closer to the person is the distance between the robot and the person is higher than this value (Default: 4 meters).
* **hri_ids_topic**. Name of the topic where the list of tracked people is being published. (default: */humans/bodies/tracked*).


## Functioning
 
* To run the actionlib actions, launch the file nhoa_approach_action.launch in the launch directory.
* In the scripts directory, there is a simple Python example of an action client to control the action. 


## Dependences

* **actionlib**
* **move_base**
* **tf2**
* **hri_msgs**


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
