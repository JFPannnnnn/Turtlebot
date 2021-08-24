# MIE443 Contest3 Group 7
# Yuhang Song, Jianfei Pan, Qiwei Zhao, Amanat Dhaliwal
Steps to run the code: 

//Assume Gazebo, Gmapping is working

#File Setup
1. Emotion Classifier:  There was a version issue when loading the mdl_best.pth file, so we add a parameter to the loading function:
 self.model.load_state_dict(torch.load(args.model_file,map_location=torch.device('cpu')))

The modification has been made inside the emotionClassifier.py, please directly use our code located at MIE443_Contest3_Group7/src

2. In order to clear the frontier blacklist to resolve the stuck issue, we modify the explore file by adding a public function to modify the private variable - "frontier_blacklist_" the public function is called "clear blacklist" and inside the function it will clear the black list for us.

The modification has been made inside the explore.cpp and explore.h, please directly use our code located at:

MIE443_Contest3_Group7/src for explore.cpp

MIE443_Contest3_Group7/include for explore.h

3. Eventually, replace the contest3.cpp file at MIE443_Contest3_Group7/src

4. Our Training file for the classifier is also located at MIE443_Contest3_Group7/src

5. Our trained model is named 'mdl_best.pth' located at MIE443_Contest3_Group7/src please use this file for testing the accuracy.

6. The images and sounds are put respectively inside the "images" and "sounds" folders located inside MIE443_Contest3_Group7, they are used for emotion response. They should be placed inside the MIE443_Contest3_Group7 folder beside 'src' folder as the way they are right now inside our zip file. 

#Start the Program

1. conda activate mie443
	a. roscd mie443_contest3 -> cd src/ -> python emotionClassifier.py 
 	b. roscd mie443_contest3 -> cd src/ -> python victimLocator.py 
	c. roslaunch mie443_contest3 turtlebot_world.launch world:=practice
2. cd catkin_ws/ -> catkin_make
3. roslaunch mie443_contest3 turtlebot_world.launch world:=practice
4. roslaunch mie443_contest3 gmapping.launch 
5. roslaunch mie443_contest3 contest3.launch

_When starting the second trial, please restart emotionClassifer and victimLocator and gmapping.launch_

#Emotion Responses
1. Reponse including poping up images, sounds, and motions, also instruction will be shown in the terminal, message will be either red, yellow, or white




