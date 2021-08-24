#include <imagePipeline.h>
#include <tuple>
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <vector>
#include <math.h>


#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

using namespace cv;
using namespace cv::xfeatures2d;

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();  
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

// use the number of 0 in the image in determine if the image is blank 
std::vector<int> ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    std::vector<int> num_matches;
    std::vector<int> num_matches_index;
    std::vector<int> if_parallel_sides;
    std:: vector<int> currentarea;
    std::vector<int> important_info;
    important_info.clear();

    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;

        important_info.push_back(-2);
        important_info.push_back(0);
        important_info.push_back(0);
        return important_info;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
        important_info.push_back(-1);
        important_info.push_back(0);
        important_info.push_back(0);
        return important_info;
    } else {
        cv::imshow("view", img); // used to be at the back 
        cv::waitKey(10);
        Mat img_scene = img;
        Mat input;
        Mat img_object;
        float cos_hori_top;
        float cos_hori_bot;
        float cos_verti_left;
        float cos_verti_right;

        float tan_hori_top;
        float tan_hori_bot;
        float tan_verti_left;
        float tan_verti_right;
        bool notweirdshape = true;

        for (int ind = 0; ind < boxes.templates.size(); ind ++){

            /***YOUR CODE HERE***/
            // Use: boxes.templates

            // Copy the input image to a local variable called input 
            //img.copyTo(input); 
            //img.release();
            //convert the input image to greyscale, save it as img_scene
            //cvtColor(input, img_scene, CV_BGR2GRAY);

            //--Step 1: Detect the keypointsusing SURF Detector, compute the descriptors

            img_object = boxes.templates[ind];
            int minHessian=250;
            Ptr<SURF> detector =SURF::create( minHessian);
            std::vector<KeyPoint> keypoints_object, keypoints_scene;
            Mat descriptors_object, descriptors_scene;
            //Mat img_object = boxes.templates[ind];
            detector->detectAndCompute( img_object, Mat(), keypoints_object, descriptors_object);
            detector->detectAndCompute( img_scene, Mat(), keypoints_scene, descriptors_scene);

            Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
            std::vector< std::vector<DMatch> > knn_matches;
            matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2);
        
        
            //--Filter matches using the Lowe's ratio test 
            const float ratio_thresh=0.76f;
            std::vector<DMatch> good_matches;
            for(size_t i=0; i<knn_matches.size(); i++){
                if(knn_matches[i][0].distance <ratio_thresh*knn_matches[i][1].distance){
                    good_matches.push_back(knn_matches[i][0]);
                    }
                }
            num_matches.push_back(good_matches.size());
            num_matches_index.push_back(ind);

            //-- Localize the object
            std::vector<Point2f> obj;
            std::vector<Point2f> scene;

            for( int i = 0; i < good_matches.size(); i++ )
            {
            //-- Get the keypoints from the good matches
                obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
                scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
            }

            if (scene.size() < 4){
                //std::cout << "Blank Tag scene.size() < 4:" << std::endl;
                template_id = -1;      // if no contour can be found 
                important_info.push_back(template_id);
                important_info.push_back(0);
                important_info.push_back(0);
                return important_info;
            }

            Mat H = findHomography( obj, scene, RANSAC );

            //-- Get the corners from the image_1 ( the object to be "detected" )
            std::vector<Point2f> obj_corners(4);
            obj_corners[0] = cvPoint(0,0); 
            obj_corners[1] = cvPoint( img_object.cols, 0 );
            obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); 
            obj_corners[3] = cvPoint( 0, img_object.rows );
            std::vector<Point2f> scene_corners(4);

            //std::cout <<" transform matrix empty"<< H.empty()<< std::endl;
            if (H.empty()){
                //std::cout << "Blank Tag H.empty:" << std::endl;
                template_id = -1;      // if no contour can be found 
                important_info.push_back(template_id);
                important_info.push_back(0);
                important_info.push_back(0);
                return important_info;
            }

            //std::cout << "scene.size_after:" << obj_corners << std::endl;
            //std::cout << "object.size_after:" << obj.size() << std::endl;
            perspectiveTransform( obj_corners, scene_corners, H);

            float x1 = scene_corners[1].x - scene_corners[0].x;
            float y1 = scene_corners[1].y - scene_corners[0].y;
            float x2 = scene_corners[2].x - scene_corners[3].x;
            float y2 = scene_corners[2].y - scene_corners[3].y;
            float x3 = scene_corners[3].x - scene_corners[0].x;
            float y3 = scene_corners[3].y - scene_corners[0].y;
            float x4 = scene_corners[2].x - scene_corners[1].x;
            float y4 = scene_corners[2].y - scene_corners[1].y;
            cos_hori_top = (abs(x1)) / (pow((x1 * x1 + y1 * y1), 0.5) );
            cos_hori_bot = (abs(x2)) / (pow((x2 * x2 + y2 * y2), 0.5) );
            cos_verti_left = (abs(x3)) / (pow((x3 * x3 + y3 * y3), 0.5) );
            cos_verti_right  = (abs(x4)) / (pow((x4 * x4 + y4 * y4), 0.5) );

            tan_hori_top = std::atan(x1/y1);
            tan_hori_bot = std::atan(x2/y2);
            tan_verti_left = std::atan(x3/y3);
            tan_verti_right  = std::atan(x4/y4);

            if ((abs(tan_hori_top) > 60 ) || (abs(tan_hori_bot) > 60 ) || (abs(tan_verti_left) > 60 ) || (abs(tan_verti_right) > 60 )){
                notweirdshape = false;
            }

            if (abs((cos_hori_top - cos_hori_bot) < 0.5) && (abs(cos_verti_left - cos_verti_right) < 0.5) && notweirdshape){
                if_parallel_sides.push_back(1);
            }else{
                if_parallel_sides.push_back(0);
            }

            float current_area = cv::contourArea(scene_corners);
            currentarea.push_back(current_area);
        }

        img_object.release();
            
        size_t area = 0;
        size_t desired_matches = 0;
        for (size_t match_index = 0; match_index < num_matches.size(); match_index++) // find the maximum number of matches and its corresponding index number
        {
            if ((area < currentarea[match_index]) && (num_matches[match_index] > 80) && (if_parallel_sides[match_index] = 1) && (currentarea[match_index] > 1000)) // Add one more constraint of homography
            {   
                template_id = num_matches_index[match_index];
                desired_matches = num_matches[match_index];
                area = currentarea[match_index];
            }

        }

        std::cout << "number of best matches:" << desired_matches << std::endl;
        std::cout << "template_id" << template_id << std::endl;
        
        if (desired_matches < 30)
        {
            template_id = -1;    // if match number is too small, it is the blank image
            std::cout << "Blank Tag <50:" << std::endl;
            important_info.push_back(template_id);
            important_info.push_back(0);
            important_info.push_back(0);
            return important_info;
           
        }


        img_object = boxes.templates[template_id];

        int minHessian=250;
        Ptr<SURF> detector =SURF::create( minHessian);
        std::vector<KeyPoint> keypoints_object, keypoints_scene;
        Mat descriptors_object, descriptors_scene;
        //Mat img_object = boxes.templates[ind];
        detector->detectAndCompute( img_object, Mat(), keypoints_object, descriptors_object);
        detector->detectAndCompute( img_scene, Mat(), keypoints_scene, descriptors_scene);

        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
        std::vector< std::vector<DMatch> > knn_matches;
        matcher->knnMatch( descriptors_object, descriptors_scene, knn_matches, 2);
    
    
        //--Filter matches using the Lowe's ratio test 
        const float ratio_thresh=0.75f;
        std::vector<DMatch> good_matches;
        for(size_t i=0; i<knn_matches.size(); i++){
            if(knn_matches[i][0].distance <ratio_thresh*knn_matches[i][1].distance){
                good_matches.push_back(knn_matches[i][0]);
                }
            }

        //--Draw matches
        Mat img_matches;
        drawMatches( img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches, Scalar::all(-1),Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        //-- Localize the object
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;

        for( int i = 0; i < good_matches.size(); i++ )
        {
            //-- Get the keypoints from the good matches
            obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
            scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
        }


        Mat H = findHomography( obj, scene, RANSAC );

        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
        obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
        std::vector<Point2f> scene_corners(4);

        perspectiveTransform( obj_corners, scene_corners, H);

        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
        line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
        line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );

        //std::cout << "scene_corners[0]" << scene_corners[0] << std::endl;
        //std::cout << "scene_corners" << scene_corners << std::endl;
        //-- Show detected matches
        imshow( "Good Matches & Object detection", img_matches );
        float x1 = scene_corners[1].x - scene_corners[0].x;
        float y1 = scene_corners[1].y - scene_corners[0].y;
        float x2 = scene_corners[2].x - scene_corners[3].x;
        float y2 = scene_corners[2].y - scene_corners[3].y;
        float x3 = scene_corners[3].x - scene_corners[0].x;
        float y3 = scene_corners[3].y - scene_corners[0].y;
        float x4 = scene_corners[2].x - scene_corners[1].x;
        float y4 = scene_corners[2].y - scene_corners[1].y;
        cos_hori_top = (abs(x1)) / (pow((x1 * x1 + y1 * y1), 0.5) );
        cos_hori_bot = (abs(x2)) / (pow((x2 * x2 + y2 * y2), 0.5) );
        cos_verti_left = (abs(x3)) / (pow((x3 * x3 + y3 * y3), 0.5) );
        cos_verti_right  = (abs(x4)) / (pow((x4 * x4 + y4 * y4), 0.5) );

        float current_area = cv::contourArea(scene_corners);

        //std::cout << "cos_hori_top" << cos_hori_top << std::endl;
        //std::cout << "cos_hori_bot" << cos_hori_bot << std::endl;
        //std::cout << "cos_verti_left" << cos_verti_left << std::endl;
        //std::cout << "cos_verti_right" << cos_verti_right << std::endl;
        std::cout << "final_current_area" << current_area << std::endl;



        waitKey(10);

        descriptors_object.release();
        descriptors_scene.release();
        H.release();
        img_matches.release();
        img_object.release();
        img_scene.release();
        good_matches.clear();
        if_parallel_sides.clear();
        currentarea.clear();
        obj_corners.clear();
        scene_corners.clear();
        obj.clear();
        scene.clear();
        num_matches.clear();
        num_matches_index.clear();

        int convert_desired_matches = static_cast<int>(desired_matches);
        int convert_area = static_cast<int>(area);
        important_info.push_back(template_id);
        important_info.push_back(convert_desired_matches);
        important_info.push_back(convert_area);
        return important_info;
    }  
    
   
}
