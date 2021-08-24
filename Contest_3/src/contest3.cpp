// New Stuck-solving code
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>


#include "explore.h"

#include <stdio.h>
#include <cmath>
#include <vector>
#include <chrono>
#include <algorithm>

#include<nav_msgs/Odometry.h>
#include<tf/transform_datatypes.h>
//
// If you cannot find the sound play library try the following command.
// sudo apt install ros-kinetic-sound-play
#include <sound_play/sound_play.h>
#include <ros/console.h>
#include <std_msgs/Int32.h>
/////// add new library/////
#include <cv.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#include <cv.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
/////////////////////////////////
#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) *180./M_PI)
#define DEG2RAD(deg) ((deg) *M_PI /180.)
#define ANGULAR_SPEED (M_PI/3)
uint8_t bumper[3]={kobuki_msgs::BumperEvent::RELEASED,kobuki_msgs::BumperEvent::RELEASED,kobuki_msgs::BumperEvent::RELEASED};

float minLaserDist=std::numeric_limits<float>::infinity();
float minDistL = 100.0;
float minDistR = 100.0;
float angleadjust = 0.0;
float lavg;
float ravg;
float turning_prob;
float sum_eugene;
float lasterror;
float lastavgerror;
float minDistabs = 0;
float prevtime;

int32_t image_number = 7;
uint8_t victimcount = 0;
std::vector<float> PosXList;
std::vector<float> PosYList;
std::vector<float> Laserlist;
std::vector<bool> TurnLeftList;
////// richard add////
std::vector<int> victimList;

int prev_image_number;

enum emotion{Angry,Disgust,Fear,Happy,Sad,Surprise,Neutral,none};



/////////////////////////////
int32_t nLasers=0,desiredNLasers=0,desiredAngle=12;

bool isleftbumperpressed = false;
bool right_bumper_pressed = false;
bool front_bumper_pressed = false;
bool left_bumper_pressed = false;
bool turn360requested = false;
bool turndesired = false;
bool turn = false;
bool maxreach = false;
bool turndesiredold = false;
bool pause3 = false;
bool Newvictim = false;
bool tryblacklist = false;

int counting_backward_step = 0;
int countstps = 0;
const int stpsize = 3;

float angular =0.0;
float linear =0.0;
float distinit = 0.0, distprev = 0.0;
float posX=0.0, posY=0.0, yaw =0.0, yawinit = 0, yawdiff = 0, yawprev = 0;
float maxdist = 0.0;
float LRerror = 0.0;
float changeangle = 0.0;
float currtime = 0;
float LRavgerror = 0.0;
int maxElementIndex = 0;
float maxElement = 0;

std::random_device rd;
std::mt19937 mt(rd());
std::bernoulli_distribution dist(0.55); // this is the chance of getting true, between 0 and 1;
std::vector< float > distmax;
std::vector< float > distmaxYaw;
std::vector<float> preferred_dir;
std::vector<float> preferred_dist;
std::vector<float> Yawlist;
bool turnleft = false;

int Checksimilarity(std::vector<float> &PosXList,std::vector<float> &PosYList, float &posX, float&posY){
    std::vector<int> simlist;
    for(uint32_t i = 0;i<(PosXList.size());++i) {
         if (fabs(posX-PosXList[i]) < 0.2 && fabs(posY-PosYList[i]) < 0.2){
             simlist.push_back(i);
         }          
    }
    if(simlist.size()>0){
        return simlist[simlist.size()-1];
    }
    return PosXList.size();
}

bool checkstuck(std::vector<float> &Laserlist,std::vector<float> &Yawlist){
    float dist_errsum = 0;
    float dist_erravg = 0;
    float yaw_errsum = 0;
    float yaw_erravg = 0;

    for(uint32_t i = 1;i<(Laserlist.size());++i) {
        if(Laserlist[i]> 50){
            return true;
        }
        dist_errsum += fabs(Laserlist[i]-Laserlist[i-1]);
        yaw_errsum += fabs(Yawlist[i]-Yawlist[i-1]);
    }
    dist_erravg = dist_errsum/(Laserlist.size()-1);
    yaw_erravg = yaw_errsum/(Yawlist.size()-1);
    //ROS_INFO("percentage: %f,%f", dist_erravg, yaw_erravg);
    if ((dist_erravg<0.001||dist_erravg > 100) && yaw_erravg <0.001){
       return true ;
    }
    return false;
}

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{

    bumper[msg->bumper] =msg->state;

}

int avoidgoingbackward(std::vector<float> &preferred_dir, float &yawinit){
    ROS_INFO("Checking if going backward");
    for (uint32_t i=0; i<preferred_dir.size(); i++){
        if(fabs(fabs(preferred_dir[i]-yawinit)-M_PI)<0.80){  // check if its the backward direction
            return i;
        }
    }
    return preferred_dir.size(); 
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
		//fill with your code
    minLaserDist=std::numeric_limits<float>::infinity();
    minDistR = 100;
    minDistL = 100;
    lavg = 0.0;
    ravg = 0.0;
    nLasers=(msg->angle_max-msg->angle_min) /msg->angle_increment;
    desiredNLasers=DEG2RAD(desiredAngle)/msg->angle_increment;
    sum_eugene = 0.001;
    int i = 0;
    //ROS_INFO("Size of laser scan array: %i and anglemin: %f and anglemax: %f", nLasers, msg->angle_max,msg->angle_min);

    if(desiredAngle*M_PI/180<msg->angle_max&&-desiredAngle*M_PI/180>msg->angle_min) {
        for(uint32_t laser_idx=nLasers/2-desiredNLasers;laser_idx<nLasers/2+desiredNLasers;++laser_idx){
            minLaserDist=std::min(minLaserDist,msg->ranges[laser_idx]);
        }
    }
    else {
        for(uint32_t laser_idx=0;laser_idx<nLasers;++laser_idx) {
            minLaserDist=std::min(minLaserDist,msg->ranges[laser_idx]);
        }
    }
    for(uint32_t laser_idx=20;laser_idx<nLasers/2;++laser_idx){
            minDistL=std::min(minDistL,msg->ranges[laser_idx]);
            if (!std::isnan(msg->ranges[laser_idx])){
            sum_eugene += msg->ranges[laser_idx];
            i++;}
    }
    if(i>0){
    lavg = sum_eugene/(i);}
    sum_eugene = 0.0, i = 0;
    for(uint32_t laser_idx=nLasers/2;laser_idx<nLasers-20;++laser_idx) {
            minDistR=std::min(minDistR,msg->ranges[laser_idx]);
            if (!std::isnan(msg->ranges[laser_idx])){
            sum_eugene += msg->ranges[laser_idx];
            i++;}
    }
    if(i>0){
    ravg = sum_eugene/(i);}
    sum_eugene = 0.0, i = 0;
    LRerror = minDistL - minDistR;
    LRavgerror = lavg - ravg;
    minDistabs = msg -> ranges[nLasers/2];

    Laserlist.push_back(minLaserDist);

}

void odomCallback(const nav_msgs::Odometry::ConstPtr&msg)
{
    //fill code
    posX=msg->pose.pose.position.x;
    posY=msg->pose.pose.position.y;
    yaw=tf::getYaw(msg->pose.pose.orientation);
    tf::getYaw(msg->pose.pose.orientation);
    Yawlist.push_back(yaw);
    //ROS_INFO("Position: (%f,%f) Orientation:%frad or%fdegrees.", posX, posY,yaw,RAD2DEG(yaw));  
}

float distcal(float px1, float py1, float px2, float py2){
    return sqrt(pow(fabs(px2-px1),2) + pow(fabs(py2-py1),2)); 
}


void EmotionCallback(const std_msgs::Int32 intmsg){
        
        if(image_number != intmsg.data){
            ROS_INFO("Find a Victim");
            Newvictim = true;

        }
        image_number = intmsg.data;
    }

int main(int argc, char** argv) {
    //
    // Setup ROS.
    ros::init(argc, argv, "contest3");
    ros::NodeHandle n;

    ros::Subscriber emotion_sub = n.subscribe("/detected_emotion" , 1, &EmotionCallback);
    ros::Subscriber bumper_sub = n.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = n.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom=n.subscribe("odom", 1, &odomCallback); 
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;


    float angular = 0.0;
    float linear = 0.0;
    int bumper_check = 0;

    //
    // Frontier exploration algorithm.
    explore::Explore explore;
    //
    // Class to handle sounds.
    sound_play::SoundClient sc;
    //
    // The code below shows how to play a sound.
    std::string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
    ros::Duration(0.1).sleep();
    sc.playWave(path_to_sounds + "sound.wav");
    //
    // The code below shows how to start and stop frontier exploration.
    //own code for exploration at specific problem case

    /////////////////richard to set up the image ///////

    std::string path_2_imgs = ros::package::getPath("mie443_contest3") + "/images/";

    Mat angry = imread(path_2_imgs + "fear.png", IMREAD_COLOR);
    Mat disgust = imread(path_2_imgs + "resentment.png", IMREAD_COLOR);
    Mat fear = imread(path_2_imgs + "positively_excited.png", IMREAD_COLOR);
    Mat happy = imread(path_2_imgs + "disgusted.png", IMREAD_COLOR);
    Mat neutral = imread(path_2_imgs + "discontent.png", IMREAD_COLOR);
    Mat sad = imread(path_2_imgs + "embarrasment.png", IMREAD_COLOR);
    Mat surprise = imread(path_2_imgs + "surprise.png", IMREAD_COLOR);


///Spin 360 initially
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed2 = 0;
    while(ros::ok() && secondsElapsed2<10){
        ros::spinOnce();
        vel.angular.z = M_PI/3;
        vel.linear.x =  0;
        vel_pub.publish(vel);
        secondsElapsed2 = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    explore.start();
    start = std::chrono::system_clock::now();
    secondsElapsed2 =0;
    ROS_INFO("Searching the victim....");
    while(ros::ok() && secondsElapsed2 < 1200) {
        // Your code here.
        ros::spinOnce();

         if (Newvictim){

            bool any_bumper_pressed=false;
            for(uint32_t b_idx=0; b_idx<N_BUMPER; ++b_idx) 
            {
                any_bumper_pressed|=(bumper[b_idx] ==kobuki_msgs::BumperEvent::PRESSED);
            }
            float settime = 6;
            explore.stop();
            std::chrono::time_point<std::chrono::system_clock> start;
            start = std::chrono::system_clock::now();
            uint64_t secondsElapsed = 0;
            linear = 0.3;
            angular = M_PI/3;
            while(ros::ok() && secondsElapsed < settime){
                ros::spinOnce();
                
            switch (image_number){  //

                case Angry://scared and step back
                 if (secondsElapsed ==2){
                    ROS_WARN("Victim Emotion Identified to be Angry");
                    cv::imshow("Display window", angry);
                    waitKey(5);
                        ros::Duration(0.1).sleep();
                    sc.playWave(path_to_sounds + "fear_angry.wav");}

                    settime = 7;
                    angular = 0;
                    if(!any_bumper_pressed){
                    if(secondsElapsed<1){
                    linear = -0.3;}else{
                        if (secondsElapsed%2!=0){
                            linear = 0;
                        }else{
                            linear = 0.1;
                        }
                    }}
                    break;
                
                case Disgust: // shaking
                    ROS_WARN("Victim Emotion Identified to be Disgust");
                    if (secondsElapsed ==2){
                    cv::imshow("Display window", angry);
                    waitKey(5);
                        ros::Duration(0.1).sleep();
                    sc.playWave(path_to_sounds + "fear_angry.wav");

                    }
                    settime = 6;
                    angular = 0;
                    linear *= -0.95; 
                    break;
                

                case Fear://shake head
                    ROS_WARN("Victim Emotion Identified to be Fear");
                   
                    if (secondsElapsed ==2){
                    cv::imshow("Display window", fear);
                    waitKey(5);
                        ros::Duration(0.1).sleep();
                    sc.playWave(path_to_sounds + "positivelyexcited_fear.wav");
                    }
                    
                    linear = 0;
                    settime  = 4;
                    if(secondsElapsed <1 || secondsElapsed>2.9){
                        angular = -M_PI/6;
                    }else {
                        angular = M_PI/6;
                    }
                    break;
                

                case Happy: // Shake head show disgust

                    ROS_WARN("Victim Emotion Identified to be Happy");   

                    if (secondsElapsed ==2){
                    cv::imshow("Display window", happy);
                    waitKey(5);
                        ros::Duration(0.1).sleep();
                    sc.playWave(path_to_sounds + "disgusted_happy.wav");
                    }

                    linear = 0;
                    settime  = 4;
                    if(secondsElapsed <1 || secondsElapsed>2.9){
                        angular = -M_PI/6;
                    }else {
                        angular = M_PI/6;
                    }
                    break;
                

                case Sad: // Turn a circle to cheer victim up

                    ROS_WARN("Victim Emotion Identified to be Sad");

                    if (secondsElapsed ==2){
                    cv::imshow("Display window", sad);
                    waitKey(5);
                        ros::Duration(0.1).sleep();
                    sc.playWave(path_to_sounds + "embarrassment_sad.wav");
                    }

                    settime = 6;
                    linear = 0; 
                    break;
                

                case Surprise: // get suprised

                    ROS_WARN("Victim Emotion Identified to be Surprise");

                    if (secondsElapsed ==2){
                    cv::imshow("Display window", surprise);
                    waitKey(5);
                        ros::Duration(0.1).sleep();
                    sc.playWave(path_to_sounds + "surprise_surprise.wav");
                    }
                    if(secondsElapsed<2){
                    linear *= -1;
                    angular *= -1;} else{
                    linear = 0;
                    angular = 0;
                   }
                    break;
                

                case Neutral:

                    ROS_WARN("Victim Emotion Identified to be Neutral");

                    if (secondsElapsed ==2){
                    cv::imshow("Display window", neutral);
                    waitKey(5);
                        ros::Duration(0.1).sleep();
                    sc.playWave(path_to_sounds + "discontent_neutral.wav");

                    }
                    angular = 0;
                    linear = 0;
                    break;

                
                case none: 

                    ROS_WARN("Victim Emotion Identified to be None");
                

            }
                vel.angular.z = angular;
                vel.linear.x =  linear;
                vel_pub.publish(vel);
                secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
                loop_rate.sleep();
            }
            //Richard's Reactions End

            ROS_FATAL("Emergency! The Building is on fire. You have 20 minutes to evacuate before the building is engulfed in fire. Please stay low and walk in a calm manner to the nearest fire exit and evacuate the building immediately");

            Newvictim = false;
            victimcount++;
            if (victimcount <7){
                ROS_INFO("Searching the Next Victim.......");
            } else{
                return -1;
            }
            goto endofop2;
        }

        if (Laserlist.size() > 100){



         if(checkstuck(Laserlist,Yawlist)){
            explore.clearblacklist();
            ROS_WARN("Frontier Exploration stucked, exerting back-up random motions");
            explore.stop();

            std::chrono::time_point<std::chrono::system_clock> start_stuck;
            start_stuck = std::chrono::system_clock::now();
            uint64_t secondsElapsed = 0;

            float angular = 0.0;
            float linear = 0.0;
            int bumper_check = 0;
            /////////////////////////////code from contest1//////////////////////
            while(ros::ok() && secondsElapsed <= 15 && !Newvictim) {
                ros::spinOnce();
                bool any_bumper_pressed=false;
                for(uint32_t b_idx=0; b_idx<N_BUMPER; ++b_idx) 
                {
                    any_bumper_pressed|=(bumper[b_idx] ==kobuki_msgs::BumperEvent::PRESSED);
                }
                
                if(any_bumper_pressed){           
                    
                    front_bumper_pressed|=(bumper[1] ==kobuki_msgs::BumperEvent::PRESSED);
                
                    left_bumper_pressed|=(bumper[0] ==kobuki_msgs::BumperEvent::PRESSED);
                    
                    right_bumper_pressed|=(bumper[2] ==kobuki_msgs::BumperEvent::PRESSED);
                }
                // ROS_INFO("front:%i, left:%i,right:%i", front_bumper_pressed, left_bumper_pressed,right_bumper_pressed);
                if(PosXList.size()>40){
                    PosXList.clear();
                    PosYList.clear();
                    TurnLeftList.clear();
                }


                if (!turndesired && !turndesiredold){
                    turnleft = dist(mt);
                }

                if (turndesired && !any_bumper_pressed){
                   // ROS_INFO("Finding Optimal Direction...");
                    if (minLaserDist - distprev > 0.05 || minLaserDist <1 ||minLaserDist>6.5){
                        distprev = minLaserDist;
                        linear = 0;
                        if (turnleft){
                            angular = M_PI/3;
                        }else if(!turnleft){
                            angular = - M_PI/3;
                        }
                    // ROS_INFO("Adjusting:%f, prev:%f , angle: current:%f, init:%f", minLaserDist,distprev,yaw,yawinit);
                    }
                    else{
                        linear = 0;
                        angular = 0;
                        turndesired = false;
                        pause3 = true;
                       // ROS_INFO("Adjustment finished:%f", minLaserDist);
                    }
                    
                    goto endofop;}


                if(!turn){
                    yawinit = yaw;
                }

                
                if (turndesiredold && !any_bumper_pressed){
                    //ROS_INFO("Finding Open Space");

                    if(turnleft){
                    while(yaw < yawprev && (fabs(yaw-yawinit) > 0.01)){
                        yaw += 2*M_PI;
                    }}else{

                    while(yaw > yawprev && (fabs(yaw-yawinit) > 0.01)){
                        yaw -= 2*M_PI;
                    }}

                    yawdiff = fabs(yaw - yawinit);
                // ROS_INFO("Adjusting:%f, prev:%f , angle: current:%f, init:%f", minLaserDist,distprev,yaw,yawinit);
                    if ((2*M_PI - yawdiff) > 0.1){
                    if (fabs(M_PI - yawdiff)<M_PI/6 ||  minLaserDist <1 ||minLaserDist>6.5){
                        distprev = minLaserDist;
                        linear = 0;
                        if (turnleft){
                            angular = M_PI/3;
                        }else if(!turnleft){
                            angular = - M_PI/3;
                        }
                        
                        yawprev = yaw;
                        currtime = secondsElapsed;
                    }
                    else{
                        linear = 0;
                        angular = 0;
                        turndesiredold = false;
                        pause3 = true;
                        yawprev = yaw;
                      //  ROS_INFO("Adjustment finished:%f", minLaserDist);
                    }}else{
                        linear = 0;
                        angular = 0;
                        yawprev = yaw;
                        turndesiredold = false;
                        distinit = minLaserDist;
                        distprev =distinit;
                        turndesired = true;
                    }
                    goto endofop;
                }
            
                
                if (front_bumper_pressed){ //bumped go backward
               // ROS_INFO("front_bumper");
                    linear = -0.3;
                    angular = M_PI/3;
                    counting_backward_step++;
                //      ROS_INFO("anguar:%f linear:%f  MinLaserDist:%f  counting_backward:%i BACK BACK ", angular, linear,minLaserDist, counting_backward_step);


                    if (counting_backward_step==14){  // going backward steps
                        counting_backward_step=0;
                        front_bumper_pressed = false;
                        turndesiredold = true;
                        distinit = minLaserDist;
                        distprev =distinit;
                
                        int check = 0;
                        if(PosXList.size()>0){
                            check = Checksimilarity(PosXList,PosYList,posX,posY);
                            if(check < PosXList.size()){
                                turnleft = (turnleft == TurnLeftList[check] ? !turnleft : turnleft);
                               // ROS_INFO("Same Place");
                            }
                        }
                        PosXList.push_back(posX);
                        PosYList.push_back(posY);
                        TurnLeftList.push_back(turnleft);
                        goto endofop;
                    }
                }

                

                else if (left_bumper_pressed){ // bumped turn right
              //  ROS_INFO("left_bumper");
                    linear = -0.3;
                    angular = -M_PI/3;
                    counting_backward_step++;
                //          ROS_INFO("anguar:%f linear:%f  MinLaserDist:%f counting_backward:%i TURNNING RIGHT ", angular, linear,minLaserDist,counting_backward_step);
                //

                    if (counting_backward_step==10){  // going backward steps
                        counting_backward_step=0;
                        left_bumper_pressed = false;
                        
                    }
                }

                else if (right_bumper_pressed){ //bumped turn left
                    linear = -0.3;
                    angular = M_PI/3;
                    counting_backward_step++;
                //      ROS_INFO("anguar:%f linear:%f  MinLaserDist:%f  counting_backward:%i   TURNNING left ", angular, linear,minLaserDist,counting_backward_step);


                    if (counting_backward_step==10){  // going backward steps
                        counting_backward_step=0;
                        right_bumper_pressed = false;
                        
                    }
                }

                else if(!any_bumper_pressed){  // go forward
                  //  ROS_INFO("go forward");

                    if (minLaserDist>0.55 && minLaserDist<6.5){

                        if (minDistabs  < 0.7){
                            linear  = 0.2;
                            angular =  -0.3*LRerror;

                        }else{
                        linear = 0.3;
                        angular = -0.255*LRerror;}
                        
                        if(angular >M_PI/6 || angular< -M_PI/6){
                            angular = 0;
                        }

                        if(linear >0.2 ){
                            linear = 0.3;
                        }
                   

                    }
                    else if ((minLaserDist<0.55 || minLaserDist>6.5) && secondsElapsed >1){
                        turndesiredold = true;
                        //Try

                        int check = 0;
                        if(PosXList.size()>0){
                            check = Checksimilarity(PosXList,PosYList,posX,posY);
                            if(check < PosXList.size()){
                                turnleft = (turnleft == TurnLeftList[check] ? !turnleft : turnleft);
                            //    ROS_INFO("Same Place");
                            }
                        }
                        PosXList.push_back(posX);
                        PosYList.push_back(posY);
                        TurnLeftList.push_back(turnleft);
                         //

                        distinit = minLaserDist;
                        distprev =distinit;
                        lasterror = 0;
                        yawinit = yaw;
                        yawprev = yaw;

                    }
                }

                

                
                endofop: 
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
                if (angular !=0){
                    turn =true;
                }else{
                    turn = false;
                }
                // The last thing to do is to update the timer.
                secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start_stuck).count();
                loop_rate.sleep();
            }  

        ROS_INFO("Searching for Victims....");
        }
    
////////////////////////end of contest1////////////////////////// 
        Laserlist.clear(); 
        Yawlist.clear();
        }
        endofop2:
        explore.start();
        ros::Duration(0.1).sleep();
        secondsElapsed2 = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }
    return 0;
}
