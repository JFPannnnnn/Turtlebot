# MIE443 Contest2 Group 7
# Yuhang Song, Jianfei Pan, Qiwei Zhao, Amanat Dhaliwal
Steps to run the code: 

//Assume Gazebo, Rviz, AMCL is working, the position is estimated correctly

#File Setup
1. The original imagePipeline.h file is modified, plese replace it with our imagePipeline.h file located at MIE443_Contest2_Group7/mie443_contest2/include
2. Correspondingly, the imagePipeline.cpp is also modified, please replace our imagePipeline.cpp file located at MIE443_Contest2_Group7/mie443_contest2/src
3. Eventually, replace the contest2.cpp file at MIE443_Contest2_Group7/mie443_contest2/src

#Start the Program
1. cd catkin_ws -> source devel/setup.sh -> rosrun mie443_contest2 contest2

#Getting Result
1. The result will be saved at catkin workspace with name "BoxIDs" (Your directory + /catkin_ws)
2. Please note that the file will be automatically replaced when the program is re-runned, so please change the name of the text file before running the next trial




