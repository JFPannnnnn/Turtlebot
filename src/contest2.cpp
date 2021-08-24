// **********************************************
// MIE 443 Contest2 Code 
// Arthors: Eugene Song, Richard Zhao, Alan Pan
// Date March 24, 2021
// Move Plan Algorithm adapted from:
// https://u.cs.biu.ac.il/~yehoshr1/89-685/Fall2013/demos/lesson7/MakePlan.cpp
// **********************************************

#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <navigation.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <string>
#include <boost/foreach.hpp>
#include <fstream>
#define forEach BOOST_FOREACH
#define defaultboxdist 0.7
#define proxthresh 0.75


double g_GoalTolerance = 0.0;
std::string g_WorldFrame = "map";
float xgoal,ygoal,phigoal, phigoalactual; 
float phigoalorg, xgoalorg, ygoalorg;
float boxdist = 0.6;
bool change_search_dir = false;


enum Proxstate
{
	Tight = 3,
    CW = 2,
	CCW = 1,
	Normal = 0,

};

class boxpos{
    public:
    float phi;
    int quadrant;
    Proxstate prox;
    public:
    boxpos(){
        phi = 0;
        quadrant = 0;
        prox = Proxstate::Normal;
    }
    boxpos(float phi_in){
        phi = phi_in;
        prox = Proxstate::Normal;
        if (phi>=0 && phi<M_PI/2){
        quadrant = 1;
        }else if (phi>M_PI/2 && phi<M_PI){
            quadrant =2;
        }else if (phi<0 && phi<-M_PI/2){
            quadrant = 3;
        }else{
            quadrant = 4;
        }
        phi = fmod(phi,2*M_PI);
    }

};


void fillPathRequest(nav_msgs::GetPlan::Request &request,float xgoal, float ygoal, float phigoal, RobotPose &robotPose);
bool callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv);

float absdist(std::vector<float> Pos1, std::vector<float> Pos2){
    float x = Pos1[0] - Pos2[0]; //calculating number to square in next step
	float y = Pos1[1] - Pos2[1];          
	return sqrt(pow(x, 2) + pow(y, 2));

}

std::vector<int >Nearest_neighbour(std::vector<std::vector<float> > &coords,
                                                    RobotPose &robotPose);

Proxstate checkProx2(boxpos box1, boxpos box2, boxpos box3, bool middle){


if ((box1.phi - box2.phi > 0 && box1.phi - box2.phi < M_PI
    && box3.phi - box1.phi > 0 && box3.phi - box1.phi < M_PI) ||
    (box2.phi - box1.phi > 0 && box2.phi - box1.phi < M_PI
    && box1.phi - box3.phi > 0 && box1.phi - box3.phi < M_PI)){
        return Proxstate::Tight;
    }


if (box2.prox == Proxstate::CW && middle){

     if(box1.quadrant != box3.quadrant){
        if (box1.quadrant - box3.quadrant == -1 || 
            box1.quadrant - box3.quadrant == 3){
            
            return Proxstate::Tight;}
        else{
            
            return Proxstate::CCW;
        }
    } else {
        return box1.phi < box3.phi ? Proxstate::Tight : Proxstate::CCW;
        }

}  else if(box2.prox == Proxstate::CCW && middle){
    
    if(box1.quadrant != box3.quadrant){
        if (box1.quadrant - box3.quadrant == 1 || 
            box1.quadrant - box3.quadrant == -3){
            
            return Proxstate::Tight;}
        else{
            return Proxstate::CW;}
        }
     else {
        return box1.phi < box3.phi ? Proxstate::Tight : Proxstate::CCW;
        }

}  else if(box2.prox == Proxstate::Tight && middle){
    
    if(box1.quadrant != box2.quadrant){
        if (box1.quadrant - box2.quadrant == 1 || 
            box1.quadrant - box2.quadrant == -3){
            
            return Proxstate::CCW;

            } else if (box1.quadrant - box2.quadrant == -1 || 
                    box1.quadrant - box2.quadrant == 3){
            
            return Proxstate::CW;
        }
    } else {
        return box1.phi > box2.phi ? Proxstate::CCW : Proxstate::CW;
        }

}else {
    if(box1.quadrant != box2.quadrant){
        if (box1.quadrant - box2.quadrant == 1 || 
            box1.quadrant - box2.quadrant == -3){
            
            return Proxstate::CCW;

            } else if (box1.quadrant - box2.quadrant == -1 || 
                    box1.quadrant - box2.quadrant == 3){
            
            return Proxstate::CW;
        }
    } else {
        return box1.phi > box2.phi ? Proxstate::CCW : Proxstate::CW;
        }
}
}

void CheckProximity(std::vector<std::vector<float> > &sortedcoords,std::vector<boxpos> &Boxposs){
    for (int i = 0; i< sortedcoords.size(); i++){
    boxpos Boxpos(sortedcoords[i][2]);
    Boxposs.push_back(Boxpos);}
    for (int i = 0; i< sortedcoords.size(); i++){
        std::vector<float> currpos = std::vector<float>(sortedcoords[i].begin(), sortedcoords[i].begin()+2);
        std::vector<float> nextpos;
        std::vector<float> endpos;
        boxpos boxbegin = Boxposs[i];
        boxpos boxnext;
        boxpos boxend;
        bool middle = false;
        if(i == 0)
        {
            nextpos = std::vector<float>(sortedcoords[i+1].begin(), sortedcoords[i+1].begin()+2);
            endpos = std::vector<float>(sortedcoords[i+2].begin(), sortedcoords[i+2].begin()+2);
            boxnext = Boxposs[i+1];
            boxend = Boxposs[i+2];
        } else if ( i == sortedcoords.size()-1)
        {
            nextpos = std::vector<float>(sortedcoords[i-1].begin(), sortedcoords[i-1].begin()+2);
            endpos = std::vector<float>(sortedcoords[i-2].begin(), sortedcoords[i-2].begin()+2);
            boxnext = Boxposs[i-1];
            boxend = Boxposs[i-2];            
        } else{
            nextpos = std::vector<float>(sortedcoords[i-1].begin(), sortedcoords[i-1].begin()+2);
            endpos = std::vector<float>(sortedcoords[i+1].begin(), sortedcoords[i+1].begin()+2); 
            boxnext = Boxposs[i-1];
            boxend = Boxposs[i+1]; 
            middle = true;
        }
        
        if (absdist(currpos,nextpos)<proxthresh || absdist(currpos,endpos)<proxthresh){
                ROS_INFO("Here, %i, absdist: %f", i,absdist(currpos,nextpos));
                ROS_INFO("box1 Quart, %i, box2 Quart, %i box3 Quart, %i", boxbegin.quadrant,
                                                                boxnext.quadrant,boxend.quadrant);
                ROS_INFO("box1 P, %f, box2 P, %f box3 P, %f", boxbegin.phi,
                                                                boxnext.phi,boxend.phi);                                                

                Boxposs[i].prox = checkProx2(boxbegin,boxnext,boxend,middle);
            }else{
                Boxposs[i].prox = Proxstate::Normal;
            }
    }
}



    

int main(int argc, char** argv) {

    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1]  << " z: " 
                  << boxes.coords[i][2] * 180/M_PI<< std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);

    //***************************************
    //my code for small adjustments
    //***************************************
    for (auto i = 0; i < 30; ++i)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    // Get Localization initalized
    for (auto i = 0; i < 5; ++i)
    {
        ros::spinOnce();
        if (!Navigation::moveToGoal(robotPose.x + 0.01, robotPose.y, robotPose.phi))
        {
            return -1;
        }
    }

    int count = 0;
    std::vector<std::vector<float> > coords = boxes.coords;
    std::vector<int> sorted = Nearest_neighbour(coords,robotPose);
    std::vector<std::vector<float> > sortedcoords;
    std::vector<float> initPos {robotPose.x,robotPose.y};
    for(uint8_t i = 0; i<sorted.size();i++){
        sortedcoords.push_back(coords[sorted[i]]);
    }
    initPos.push_back(robotPose.phi);
    sortedcoords.push_back(initPos);
    std::vector<boxpos> Boxposs;
    Boxposs.reserve(sortedcoords.size());
    CheckProximity(sortedcoords,Boxposs);
    std::vector<std::vector<float> > coords_not_went;
    //***************************************
    //my code end
    //***************************************
//rosservice list /move_base

    //****************Make Plan**********************//
    // Init service query for make plan
    phigoal = sortedcoords[count][2];
    xgoal = sortedcoords[count][0]+ 0.55*std::cos(phigoal);
    ygoal = sortedcoords[count][1]+ 0.55*std::sin(phigoal);
	std::string service_name = "move_base/NavfnROS/make_plan";
	while (!ros::service::waitForService(service_name, ros::Duration(3.0))) {
		ROS_INFO("Waiting for service move_base/make_plan to become available");
}

	ros::ServiceClient serviceClient = n.serviceClient<nav_msgs::GetPlan>(service_name, true);
	if (!serviceClient) {
		ROS_FATAL("Could not initialize get plan service from %s", serviceClient.getService().c_str());
		return -1;
	}

	if (!serviceClient) {
		ROS_FATAL("Persistent service connection to %s failed", serviceClient.getService().c_str());
		return -1;
	}
    nav_msgs::GetPlan srv;
    std::ofstream BoxIDs("BoxIDs.txt");
    //*****************Make Plan Finished************//
    

    // Execute strategy.
    while(ros::ok()) {
        ros::spinOnce();

        //ROS_INFO("RobotPose: x:%f, y:%f, phi%f", robotPose.x,robotPose.y, robotPose.phi);
        if (count < sortedcoords.size()-1){
            phigoal = sortedcoords[count][2];
            xgoal = sortedcoords[count][0]+ boxdist*std::cos(phigoal);
            ygoal = sortedcoords[count][1]+ boxdist*std::sin(phigoal);
            phigoalorg = phigoal;
            xgoalorg = xgoal;
            ygoalorg = ygoal;
            fillPathRequest(srv.request,xgoal,ygoal,fmod(phigoal + 3.14, 6.28), robotPose);
            float searchangle = M_PI/3;
            boxdist = defaultboxdist;
            ROS_INFO("Original Coords: x:%f, y:%f, phi:%f", xgoal,ygoal, phigoal);
            while (!callPlanningService(serviceClient, srv)){
                
                xgoal = xgoalorg; ygoal = ygoalorg;
                if (!change_search_dir){
                if(phigoal - phigoalorg< searchangle){
                    phigoal += M_PI/9;
                    ROS_INFO("CCW Adjusting Angle");
                }else {
                    phigoal = phigoalorg;
                    change_search_dir = true;
                    ROS_INFO("CCW way no good");
                }
                } else{
                if(phigoal - phigoalorg > -searchangle){
                    phigoal -= M_PI/9;
                    ROS_INFO("CW Adjusting Angle");
                }else{
                    if(searchangle < M_PI/2){
                        ROS_INFO("Damn, No place to go?? Increase the search range!");
                        phigoal = searchangle;
                        searchangle += M_PI/4;
                        change_search_dir = false;
                        boxdist -= 0.05;
                    } else{
                        ROS_INFO("Damn, I guess I will need to decrease the box dist");
                        phigoal = phigoalorg;
                        if (boxdist > 0){ 
                        boxdist -= 0.05;
                        phigoal = phigoalorg;
                        change_search_dir = false;}
                        else{
                            break;
                        }
                    }
                    }
                }
                
                xgoal = sortedcoords[count][0]+ (boxdist-0.1*fabs(phigoal-phigoalorg)/searchangle) * std::cos(fmod(phigoal, 2*M_PI));
                ygoal = sortedcoords[count][1]+ (boxdist-0.1*fabs(phigoal-phigoalorg)/searchangle) * std::sin(fmod(phigoal, 2*M_PI));
                fillPathRequest(srv.request,xgoal,ygoal,fmod(phigoal + 3.14, 6.28), robotPose);
                }
       }else if (count == sortedcoords.size()-1){
           ROS_INFO("Going Home");
            phigoal = sortedcoords[count][2];
            xgoal = sortedcoords[count][0];
            ygoal = sortedcoords[count][1];
            phigoalorg = phigoal;
            xgoalorg = xgoal;
            ygoalorg = ygoal;
            change_search_dir = false;
            fillPathRequest(srv.request,xgoal,ygoal,fmod(phigoal, 6.28), robotPose);
            while (!callPlanningService(serviceClient, srv)){
                if(!change_search_dir){
                    if (xgoal - xgoalorg < 1){
                    xgoal += 0.01;
                    fillPathRequest(srv.request,xgoal,ygoal,fmod(phigoal, 6.28), robotPose);
                    while (!callPlanningService(serviceClient, srv)){
                        if(!change_search_dir){
                            if (ygoal - ygoalorg < 1)
                            {
                            ygoal += 0.01;}
                            else{
                            change_search_dir = true;
                            ygoal = ygoalorg;
                            }

                        }else{
                        if (ygoal - ygoalorg > -1){
                            ygoal -= 0.01;
                        } else{
                            change_search_dir = false;
                            break;
                        }
                        }
                    }
                }
                }else{
                    if (xgoal - xgoalorg > -1){
                        xgoal -= 0.01;
                        fillPathRequest(srv.request,xgoal,ygoal,fmod(phigoal, 6.28), robotPose);
                        while (!callPlanningService(serviceClient, srv)){
                        if(!change_search_dir){
                            if (ygoal - ygoalorg < 1)
                            {
                            ygoal += 0.01;}
                            else{
                            change_search_dir = true;
                            ygoal = ygoalorg;
                            }

                        }else{
                        if (ygoal - ygoalorg > -1){
                            ygoal -= 0.01;
                        } else{
                            change_search_dir = false;
                            break;
                        }
                        }
                    }

                    }else{
                            change_search_dir = false;
                            break;
                        }

                }
                fillPathRequest(srv.request,xgoal,ygoal,fmod(phigoal, 2* M_PI), robotPose);
            }
            if (!Navigation::moveToGoal(xgoal, ygoal, phigoalactual))
            {
            return -1;
            }
            return -1;
            }
        
        else if (!coords_not_went.empty()){
           ROS_INFO("Going to left_over coords");
           //Code?
           return -1;

       }
       else{
           return -1;
       }
        float phidev = phigoalorg - phigoal;

        ROS_INFO("phi deviated: %f", phidev);
        ROS_INFO("Current Coords: x:%f, y:%f, phi:%f", xgoal,ygoal, phigoal);
        phigoalorg = phigoal;
        float xgoalprev = xgoal; 
        float ygoalprev = ygoal;
        float accumulatedist = 0;
        float minimaldist = 0;
        //Algorithm deal with situations with more than one pic around
        switch(Boxposs[count].prox){
            case CCW:
            phigoal = fmod(phigoal+(M_PI/8*(1-2*phidev/M_PI)), 2*M_PI); 
            minimaldist = 0.25;
            ROS_INFO("Other Objects Very Close, Rotate CCW");
            break;

            case CW:
            phigoal = fmod(phigoal-(M_PI/8*(1-2*phidev/M_PI)), 2*M_PI);
            ROS_INFO("Other Objects Very Close, Rotate CW");
            minimaldist = 0.25;
            break;

            case Normal:
            ROS_INFO("Normal Conditions");
            minimaldist = 0.2;
            break;

            case Tight:
            phigoal = sortedcoords[count][2];
            phigoalorg = phigoal;
            minimaldist = 0.2;
            ROS_INFO("More than two objects around");
            break;
        }

        
        fillPathRequest(srv.request,xgoal-0.01*std::cos(fmod(phigoalorg, 2*M_PI)),
        ygoal-0.01*std::sin(fmod(phigoalorg, 2*M_PI)),fmod(phigoal + 3.14, 6.28), robotPose);
        while(callPlanningService(serviceClient, srv)&& boxdist - accumulatedist >minimaldist){
            ROS_INFO("Im here");
            xgoalprev = xgoal; ygoalprev = ygoal;
            xgoal -= 0.01*std::cos(fmod(phigoalorg, 2*M_PI));
            ygoal -= 0.01*std::sin(fmod(phigoalorg, 2*M_PI));
            accumulatedist += 0.01;
            fillPathRequest(srv.request,xgoal,ygoal,fmod(phigoal + 3.14, 6.28), robotPose);
        } 
        xgoal = xgoalprev;
        ygoal = ygoalprev;
        

        phigoalactual = fmod(phigoal + 3.14, 6.28);
        if (!Navigation::moveToGoal(xgoal, ygoal, phigoalactual))
        {
            coords_not_went.push_back(sortedcoords[count]);
        }else{

        //Imagepipeline Starts Here
        // Space for Image Pipeline
        
    ImagePipeline imagePipeline(n);
    std::vector<int> potential_pic; 
    std::vector<int> list_matches; 
    std::vector<int> list_area; 
    
    int suggested_id = 0;
    int template_id = 0;
    int num_iteration = 0;
    int num_valid = 0;
    int num_matches=0 ;
    int area=0;
    while(ros::ok()) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        std::vector<int> received_value = imagePipeline.getTemplateID(boxes);
        suggested_id = received_value[0];
        num_matches = received_value[1];
        area = received_value[2];

        if (num_iteration<6 && suggested_id !=-2){
        
            ROS_WARN("Richard the god suggested_id: %i",suggested_id+1);
            potential_pic.push_back(suggested_id);
            list_matches.push_back(num_matches);
            list_area.push_back(area);
            num_iteration = num_iteration+1;
            ROS_INFO("THE Jianfei_iiiiiiiiiiiiteration:%i", num_iteration);            

        }

        if (num_iteration==6)
        {   
            // template_id = mostFrequent(potential_pic,potential_pic.size());
            // ROS_INFO("THE PIC NUMBER IS: %i", template_id);
            // potential_pic.clear();  

            for (int i = 0; i <potential_pic.size();i++){
                if (list_matches[i]<70){       // can tune
                    list_area[i] =0;
                }
            }

            list_matches[0]=0;
            list_area[0]=0;

            int maxMatchesIndex = std::max_element(list_matches.begin(),list_matches.end()) - list_matches.begin();
            int maxMatches = *std::max_element(list_matches.begin(), list_matches.end());

            int maxAreaIndex = std::max_element(list_area.begin(),list_area.end()) - list_area.begin();
            int maxArea = *std::max_element(list_area.begin(), list_area.end());
            ROS_INFO("The max area :%i", maxArea);
            ROS_INFO("The max area index :%i", maxAreaIndex);
            ROS_INFO("The max area's Matches number:%i", list_matches[maxAreaIndex]);
            ROS_INFO("##########################################################################");

            ROS_INFO("The max matches :%i", maxMatches);
            ROS_INFO("The max Matches Index :%i", maxMatchesIndex );
            ROS_INFO("The max matches's area :%i", list_area[maxMatchesIndex]);
            ROS_INFO("##########################################################################");

            if (list_matches[maxAreaIndex]>70){
                ROS_FATAL("The picture index number is :%i", potential_pic[maxMatchesIndex]+1);   
                ROS_INFO("##########################################################################");
                if (count<sorted.size()){
                BoxIDs <<"Box No."<<sorted[count]<<" tag_"<<potential_pic[maxMatchesIndex] +1 << " Coordiantes:" << sortedcoords[count][0]<< "," << sortedcoords[count][1] << std::endl;
                }

            }
            else {
                ROS_INFO("It is possible blank");
                if (count<sorted.size()){            
                BoxIDs <<"Box No."<<sorted[count]<<" tag_"<< "Blank" << " Coordiantes:" << sortedcoords[count][0]<< "," << sortedcoords[count][1] <<std::endl;
                }
            }

            potential_pic.clear();
            list_matches.clear();
            list_area.clear();
            break;
        }
        
        // std::cout<<'the selected pic_id:'<<template_id<<std::endl;
          
    }
        }

        
        count ++;
        ros::Duration(0.01).sleep();
        change_search_dir = false;
        ROS_INFO("Desitination %i reach", count);
    }
    return 0;
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
/////////////////////////**Functions**//////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void fillPathRequest(nav_msgs::GetPlan::Request &request,float xgoal, float ygoal, float phigoal, RobotPose &robotPose)
{
	request.start.header.frame_id = g_WorldFrame;
	request.start.pose.position.x = robotPose.x;
	request.start.pose.position.y = robotPose.y;
	request.start.pose.orientation.w = robotPose.phi;

	request.goal.header.frame_id = g_WorldFrame;
	request.goal.pose.position.x = xgoal;
	request.goal.pose.position.y = ygoal;
	request.goal.pose.orientation.w = phigoal;

	request.tolerance = g_GoalTolerance;
}

bool callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv)
{
	 // Perform the actual path planner call
	if (serviceClient.call(srv)) {
		if (!srv.response.plan.poses.empty()) {
            return true;
		}
		else {
            return false;
		}
	}
	else {
		ROS_ERROR("Failed to call service %s - is the robot moving?", serviceClient.getService().c_str());
        return false;
	}
}

std::vector<int >Nearest_neighbour(std::vector<std::vector<float> > &coords,
                                                    RobotPose &robotPose){
   
    std::vector<int> sorted(coords.size(),0);
    std::vector<float> initPos {robotPose.x,robotPose.y}; // save the initial pos
    std::vector<float> nextPos {robotPose.x,robotPose.y}; // the first pos will be the initial pos
    
    std::vector<int> unsorted(coords.size(),0); 
    for(uint8_t i =0; i<unsorted.size(); i++){unsorted[i]=i;} // initialize an vector starts from 0 to 9
    std::vector<float> costlist(unsorted.size(),0); // initialize a list that saves all the eucliden distance
    int index  = 0;
    
    while(unsorted.size()>0){ // continue the loop while there are unfinished coods
    
    for(uint8_t i = 0; i<unsorted.size();i++){ //calculate all the costs(abs distance)
        std::vector<float> currpos = std::vector<float>(coords[unsorted[i]].begin(), coords[unsorted[i]].begin()+2);
       costlist[i] = absdist(currpos,nextPos);
    }
    //find the index of the box that has the minimum cost
    int minElementIndex = std::min_element(costlist.begin(),costlist.end()) - costlist.begin(); 
    // update the next coord
    nextPos = std::vector<float>(coords[unsorted[minElementIndex]].begin(),coords[unsorted[minElementIndex]].begin()+2);
    sorted[index] = unsorted[minElementIndex];
    unsorted.erase(unsorted.begin()+minElementIndex);
    costlist.pop_back();
    index ++;

    }
    return sorted;
    }
