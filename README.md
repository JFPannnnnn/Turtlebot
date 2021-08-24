The files in this repository largely come from my coursework in MIE443 Mechatronics System: Integration and Design. There are three design contests on the simulated Turtlebot in ROS Gazebo. The contests are completed by a team of four, where each of us contributes almost equally to their successes. 

The goal of contest 1 is to autonomously drive the Turltebot in an unknown simulated
environment, while dynamically mapping out the surrounding within 15 minutes.
Specifically, the Turtlebot needs to achieve mapping, localization and planning to
successfully complete the task. The turtlebot will be simulated in the Gazebo simulator using the Robotic Operating System (ROS). The Turtlebot will use the depth sensor reading from the Kinect sensor to map the surrounding environment and major
obstacles. Moreover, it will use the GMapping package to achieve localization during
exploration. Thus, our main goal is to utilize sensor readings and design algorithms so that the turtlebot can navigate safely in various environments and avoid major
obstacles. Eventually, the environment and key obstacles are mapped and visualized in
RViz in the given 15 minutes.

The main objective of contest 2 is to design a programme that enables the turtlebot to
traverse the environment and navigate to and identify 10 different objects placed on the map. The general layout of the map including the location and orientation of the 10 objects will be provided at the beginning of the run. The turtlebot is required to use the given map to travel to each of the 10 objects, and identify the tags placed on each of these objects. The robot is also expected to return back to its starting position after identifying all 10 tags. All of this needs to be accomplished within the allotted time of 8 minutes. The team is responsible for designing an algorithm that can use the map provided to navigate towards each of the 10 objects in a time efficient manner and also utilize the robot RGB sensor to identify the tags and their corresponding locations for all 10 images before returning to the starting location.

The goal of contest 3 is to let the simulated Turtlebot explore and identify seven victims located randomly in an unknown disastrous environment and provide them with
emergency evacuation announcements. As a result, the Turtebot should find all the
victims by continuously exploring and mapping the unknown environment. After
reaching the victims, the Turtlebot should identify their emotions and interact with them according to their emotional states. The frontier exploration and victim identification algorithms are provided. Thus, the team needs to design algorithms for the Turtlebot to identify and respond to the seven user emotions and perform unique interactions using primary and secondary emotions. Each robot emotion has to be unique and thus must not be duplicated among the primary and secondary emotions. 