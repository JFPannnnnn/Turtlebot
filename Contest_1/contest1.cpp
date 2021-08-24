#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <cmath>
#include <vector>
#include <chrono>
#include <algorithm>


#include<nav_msgs/Odometry.h>
#include<tf/transform_datatypes.h>

#include <random>
#include <iostream>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) *180./M_PI)
#define DEG2RAD(deg) ((deg) *M_PI /180.)

enum Bumper
{
	Left = 0,
	Center = 1,
	Right = 2,
};

uint8_t bumper[3]={kobuki_msgs::BumperEvent::RELEASED,kobuki_msgs::BumperEvent::RELEASED,kobuki_msgs::BumperEvent::RELEASED};

float minLaserDist=std::numeric_limits<float>::infinity();
float minDistL = 100.0;
float minDistR = 100.0;
float angleadjust = 0.0;
float lavg;
float ravg;
float turning_prob;
float sum;
float lasterror;
float lastavgerror;
float minDistabs = 0;
float prevtime;
std::vector<float> PosXList;
std::vector<float> PosYList;
std::vector<float> Laserlist;
std::vector<bool> TurnLeftList;
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
bool checkstuck(std::vector<float> &Laserlist){
    float errsum = 0;
    float erravg = 0;
    for(uint32_t i = 1;i<(Laserlist.size());++i) {
        errsum += fabs(Laserlist[i]-Laserlist[i-1]);
    }
    erravg = errsum/(Laserlist.size()-1);
    //ROS_INFO("erravg: %f", erravg);
    if(erravg < 0.0001){
        return true;
    }
    return false;
}
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//fill with your code
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] =msg->state;


    //uint8_t leftState=bumper[kobuki_msgs::BumperEvent::LEFT];
   
    //kobuki_msgs::BumperEvent::PRESSED if bumper is pressed,kobuki_msgs::BumperEvent::RELEASED otherwise
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
float sortingangles(std::vector<float> &distmaxYaw, std::vector<float> &distmax, std::vector<float> &preferred_dir, std::vector<float> &preferred_dist, float &yawinit){
    int cnt = 1; 
    if (distmax.size()>0){
        preferred_dir[0] = distmaxYaw[0];
        preferred_dist[0] = distmax[0];
    }
    for (uint32_t i=0; i<3; i++){
        if (cnt == distmax.size()){
            if (fabs(distmaxYaw[cnt-1] - distmaxYaw[cnt-2])>0.72&& preferred_dist[i-1] != distmax[cnt-1]){
                preferred_dist[i] = distmax[cnt-1];
                preferred_dir[i] = distmaxYaw[cnt-1];
            }}
        else if(cnt < distmaxYaw.size()){
            while((fabs(distmaxYaw[cnt] - distmaxYaw[cnt-1])<0.72) && cnt < distmaxYaw.size()){
                if(distmax[cnt]>preferred_dist[i]&& cnt<distmaxYaw.size()){
                    preferred_dist[i] = distmax[cnt];
                    preferred_dir[i] = distmaxYaw[cnt];
                    }
                cnt++;
                }
        cnt++;
        }
    }
   ROS_INFO("preferred_dir 1:%f,2:%f,3:%f, initial: %f",preferred_dir[0],preferred_dir[1],preferred_dir[2],yawinit);
   int back = avoidgoingbackward(preferred_dir , yawinit); // check if any direction is the backward direction 
    if (preferred_dir[0] == 0){
        return 0;
        
    }else if(preferred_dir[1] == 0){    // There is only one open space
        ROS_INFO("Only One Way to Go!");
        return preferred_dir[0];

    }else if(preferred_dir[1]!=0 && preferred_dir[2] == 0) { // There are two open spaces
        if(back!= preferred_dir.size()){
            ROS_INFO("In a Corner"); //corner check, make sure it will not turn back (180 degrees)
            return preferred_dir[!back];
        }else{
            ROS_INFO("Turning Randomly");
            return (turnleft == 1 ? preferred_dir[1] : preferred_dir[0]);
            }
    }else{
        ROS_INFO("3 open spaces");
        if(back!=preferred_dir.size()){  
            ROS_INFO("Eliminate 180 one"); // Not turning back 
            switch (back){
                case 0: return (turnleft == 1 ? preferred_dir[1] : preferred_dir[2]);  // Randomly chosen a direction to turn 
                case 1: return (turnleft == 1 ? preferred_dir[0] : preferred_dir[2]);
                case 2: return (turnleft == 1 ? preferred_dir[0] : preferred_dir[1]);
                ROS_INFO("Switch");
            }
            }else{
            return (turnleft == 1 ? preferred_dir[0] : preferred_dir[2]); // it will randomly turns to first or the thrid direction when none is 180
            }
        }

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
    sum = 0.001;
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
            sum += msg->ranges[laser_idx];
            i++;}
    }
    if(i>0){
    lavg = sum/(i);}
    sum = 0.0, i = 0;
    for(uint32_t laser_idx=nLasers/2;laser_idx<nLasers-20;++laser_idx) {
            minDistR=std::min(minDistR,msg->ranges[laser_idx]);
            if (!std::isnan(msg->ranges[laser_idx])){
            sum += msg->ranges[laser_idx];
            i++;}
    }
    if(i>0){
    ravg = sum/(i);}
    sum = 0.0, i = 0;
    LRerror = minDistL - minDistR;
    LRavgerror = lavg - ravg;
    minDistabs = msg -> ranges[nLasers/2];
    Laserlist.push_back(minDistabs);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr&msg)
{
    //fill code
    posX=msg->pose.pose.position.x;
    posY=msg->pose.pose.position.y;
    yaw=tf::getYaw(msg->pose.pose.orientation);
    tf::getYaw(msg->pose.pose.orientation);
 // ROS_INFO("Position: (%f,%f) Orientation:%frad or%fdegrees.", posX, posY,yaw,RAD2DEG(yaw));  
}

float distcal(float px1, float py1, float px2, float py2){
    return sqrt(pow(fabs(px2-px1),2) + pow(fabs(py2-py1),2)); 
}

//bool any_bumper_pressed()
//{
//	return bumperRight || bumperCenter || bumperLeft;
//}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "maze_explorer");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom=nh.subscribe("odom", 1, &odomCallback); 
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;
    int bumper_check = 0;

    while(ros::ok() && secondsElapsed <= 900) {
        ros::spinOnce();
        //fill with your code
        // Check if any of the bumpers were pressed.
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
        if(Laserlist.size() > 5){
            if(checkstuck(Laserlist)&&!pause3&&!turndesired&&!turndesiredold&&!turn360requested){
                ROS_INFO("stuck");
                front_bumper_pressed = true;
            }
            Laserlist.clear();
        }
        if(pause3){
            ROS_INFO("Pause 3 seconds");
            if(secondsElapsed - currtime < 3){
            angular = 0;
            linear = 0;        }else{
                pause3 = false;
            }
	    Laserlist.clear();
            goto endofop;}
        if (turn360requested){
            while(yaw < yawprev){
                yaw += 2*M_PI;
            }
            yawdiff = fabs(yaw - yawinit);
            ROS_INFO("turn360requested");
            if ((2*M_PI - yawdiff) > 0.1){
                linear = 0;
                angular = M_PI/12;
                yawprev = yaw;
            }
            else{
                currtime = secondsElapsed;
                angular = 0;
                yawprev = 0;
                turn360requested = false;
                pause3 = true;
            }
            goto endofop;
        }

        if (changeangle !=0){

            while(yaw < yawprev){
                yaw += 2*M_PI;
            }
            ROS_INFO("changeangle");
            if(fabs(changeangle - yaw + 2*M_PI )> 0.1 && yaw < 15){
                angular = M_PI/6;
                linear = 0.0;
                yawprev = yaw;
                
            }else{
                changeangle = 0;
                currtime = secondsElapsed;
                pause3 = true;
                angular = 0; 
                linear = 0.0;
                yawprev = 0;
                turn360requested = false;
            }

            goto endofop;
        }
        if (!turndesired && !turndesiredold){
            turnleft = dist(mt);
        }

        if (turndesired && !any_bumper_pressed){
            ROS_INFO("Finding Optimal Direction...");
            if (minLaserDist - distprev > 0.05 || minLaserDist <1 ||minLaserDist>6.5){
                distprev = minLaserDist;
                linear = 0;
                if (turnleft){
                    angular = M_PI/6;
                }else if(!turnleft){
                    angular = - M_PI/6;
                }
               // ROS_INFO("Adjusting:%f, prev:%f , angle: current:%f, init:%f", minLaserDist,distprev,yaw,yawinit);
            }
            else{
                linear = 0;
                angular = 0;
                turndesired = false;
                pause3 = true;
                ROS_INFO("Adjustment finished:%f", minLaserDist);
            }
            
            goto endofop;}


        if(!turn){
            yawinit = yaw;
        }



        if(secondsElapsed % 100 == 0 && secondsElapsed >0 && !turndesired && !turndesiredold){   /// control the time of spining 
    //        ROS_INFO("20 Seconds Passed, Turn 360");
            turn360requested = true;

            goto endofop;
        }
        
        if (turndesiredold && !any_bumper_pressed){
            ROS_INFO("Finding Open Space");

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
                    angular = M_PI/8;
                }else if(!turnleft){
                    angular = - M_PI/8;
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
                ROS_INFO("Adjustment finished:%f", minLaserDist);
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
        ROS_INFO("front_bumper");
            linear = -0.05;
            angular = M_PI/6;
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
                        ROS_INFO("Same Place");
                    }
                }
                PosXList.push_back(posX);
                PosYList.push_back(posY);
                TurnLeftList.push_back(turnleft);
                goto endofop;
            }
        }

        

        else if (left_bumper_pressed){ // bumped turn right
        ROS_INFO("left_bumper");
            linear = -0.05;
            angular = -M_PI/6;
            counting_backward_step++;
  //          ROS_INFO("anguar:%f linear:%f  MinLaserDist:%f counting_backward:%i TURNNING RIGHT ", angular, linear,minLaserDist,counting_backward_step);
//

            if (counting_backward_step==10){  // going backward steps
                counting_backward_step=0;
                left_bumper_pressed = false;
                
            }
        }

        else if (right_bumper_pressed){ //bumped turn left
            linear = -0.05;
            angular = M_PI/6;
            counting_backward_step++;
      //      ROS_INFO("anguar:%f linear:%f  MinLaserDist:%f  counting_backward:%i   TURNNING left ", angular, linear,minLaserDist,counting_backward_step);


            if (counting_backward_step==10){  // going backward steps
                counting_backward_step=0;
                right_bumper_pressed = false;
                
            }
        }

        else if(!any_bumper_pressed){  // go forward
            ROS_INFO("go forward");

            if (minLaserDist>0.55 && minLaserDist<6.5){

                if (minDistabs  < 0.7){
                    linear  = 0.15;
                    angular =  -0.3*LRerror;

                }else{
                linear = 0.17;
                angular = -0.255*LRerror;}
                
                if(angular >M_PI/6 || angular< -M_PI/6){
                    angular = 0;
                }

                if(linear >0.2 ){
                    linear = 0.2;
                }
               // lastavgerror = LRavgerror;
               // prevtime = secondsElapsed;
               // ROS_INFO("anguar:%f linear:%f  LRavgError:%f dError:%f Lavg:%f Ravg: %f ", angular, linear,LRavgerror,dErroravg,lavg,ravg);
                

            }
             else if ((minLaserDist<0.55 || minLaserDist>6.5) && secondsElapsed >1){
                turndesiredold = true;
//Try

                int check = 0;
                if(PosXList.size()>0){
                    check = Checksimilarity(PosXList,PosYList,posX,posY);
                    if(check < PosXList.size()){
                        turnleft = (turnleft == TurnLeftList[check] ? !turnleft : turnleft);
                        ROS_INFO("Same Place");
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
             //   if(yawinit < 0){
               // yawinit = yawinit + 2*M_PI; // start saving the yawinit
                //}
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
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }


    return 0;
}
