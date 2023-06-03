//
// Created by ou on 2021/3/14.
//
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros_yolo/yolo.h>
#include <ros_yolo/yoloAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <ros/package.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
using namespace std;
cv::Mat frame;
void callbackRGB(const sensor_msgs::ImageConstPtr& msg)
{
    cout<<"start call back"<<endl;
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr=cv_bridge::toCvCopy(msg,msg->encoding);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception;%s",e.what());
        return;
    }
    cv_ptr->image.copyTo(frame);
    cout<<"end"<<endl;
    cv::imwrite("/home/robot/ros_ws/img/draw.png",frame);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "client_node");
    ros::NodeHandle n;
    // cv::VideoCapture cap(1);
    ros::Subscriber sub = n.subscribe("/kinect2/hd/image_color",1,callbackRGB);
    bool isAction=false;
        n.getParam("yolov5/action", isAction);
    if (isAction) {
        ROS_ERROR("no in service mode, please modify the .yaml file and reload");
        return 0;
    }
    //start yolo client
    ros::ServiceClient client = n.serviceClient<ros_yolo::yolo>("yolo_service");
    client.waitForExistence(ros::Duration(30e-3));

    ros_yolo::yolo srv;

        //start capture
        ros::Rate loop_rate(10);
        while(ros::ok()){

            ros::spinOnce();
            frame=cv::imread("/home/robot/ros_ws/img/draw.png");
            srv.request.image = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            if (client.call(srv)) {
                cout << "----------------" << endl;
                cout << "detect: " << srv.response.results.size() << " objects" << endl;
                vector<string> vstr;
                
                for (auto &result:srv.response.results) {
                    int flag=0;
                    for(int i=0;i<vstr.size();i++)
                    {
                        if(vstr[i].substr(0,3)==result.label.substr(0,3))
                        {
                            flag=1;
                            break;
                        }
                    }
                    if(flag==1)
                    {
                        continue;
                    }
                    vstr.push_back(result.label);
                    auto xyxy = result.bbox.xyxy;
                    cv::Point p1(xyxy[0], xyxy[1]), p2(xyxy[2], xyxy[3]), wh = p2 - p1;
                    auto thickness = cv::min(wh.x, wh.y);
                    cv::rectangle(frame, p1, p2, cv::Scalar(255, 0, 0), thickness / 40 + 1);
                    cv::putText(frame, result.label, p1, cv::FONT_HERSHEY_COMPLEX,
                                1, cv::Scalar(0, 0, 255),
                                1, 0);
                    cout << result.label << endl;
                }
                //}
                cout<<"kaishi1"<<endl;
                cv::imshow("img", frame);
                cout<<"kaishi2"<<endl;
                cv::waitKey(10);
                loop_rate.sleep();

            }
    }
    

    return 0;
}
/*
//
// Created by ou on 2021/3/14.
//
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros_yolo/yolo.h>
#include <ros_yolo/yoloAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <ros/package.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
using namespace std;
cv::Mat frame;
bool bCaptureOneFrame=true;
void callbackRGB(const sensor_msgs::ImageConstPtr& msg)
{
    if(bCaptureOneFrame==true)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        //cv_ptr
        frame=cv_ptr->image;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "client_node");
    ros::NodeHandle n;
    // cv::VideoCapture cap(1);
    n.subscribe("/kinect2/hd/image_color",1,callbackRGB);
    std::string im_pth = "/home/robot/test_ws/recordData/RGBD/catch.png";

    // if(!cap.isOpened())
    // {
    //     printf("capture not open");
    //     return -1;
    // }
    // namedWindow("myCamera",CV_WINDOW_AUTOSIZE);
    bool isAction=false;
        n.getParam("yolov5/action", isAction);
    if (isAction) {
        ROS_ERROR("no in service mode, please modify the .yaml file and reload");
        return 0;
    }

    //cv::VideoCapture capture;
    //capture.open(ros::package::getPath("ros_yolo") + "/resource/video.mp4");

    //request once
    //ros_yolo::yolo srv;
    //srv.request.image = imread(ros::package::getPath("ros_yolo") + "/resource/2019_0001.jpg");
    // cv::Mat frame;
    // bool isAction = false;

    //while (capture.isOpened()) {
     //   capture.read(frame);


    ros::ServiceClient client = n.serviceClient<ros_yolo::yolo>("yolo_service");
    client.waitForExistence(ros::Duration(30e-3));

    ros_yolo::yolo srv;
    frame = cv::imread("/home/robot/屏幕截图2.png",1);
    // ros_yolo::yolo srv;
    cv::Mat edges;
    bool stop=false;

        cout<<"read"<<endl;
        cv::imshow("img", frame);
        // cv::cvtColor(frame,edges,CV_BGR2GRAY);
        srv.request.image = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        vector<string> strv;
        if (client.call(srv)) {
            cout << "----------------" << endl;
            cout << "detect: " << srv.response.results.size() << " objects" << endl;

            for (auto &result:srv.response.results) {
                int flag=0;
                for(int i=0;i<strv.size();i++)
                {
                    if(result.label.substr(0,4) == strv[i].substr(0,4)){
                        flag=1;
                        break;
                    }
                }
                if(flag==1)
                    continue;
                strv.push_back(result.label.substr(0,4));
                auto xyxy = result.bbox.xyxy;
                cv::Point p1(xyxy[0], xyxy[1]), p2(xyxy[2], xyxy[3]), wh = p2 - p1;
                auto thickness = cv::min(wh.x, wh.y);
                cv::rectangle(frame, p1, p2, cv::Scalar(255, 0, 0), thickness / 40 + 1);
                cv::putText(frame, result.label, p1, cv::FONT_HERSHEY_COMPLEX,
                            1, cv::Scalar(0, 0, 255),
                            0.5, 0);
                cout << result.label << endl;
            }
            //}

            cv::imshow("img", frame);
            cv::imwrite("/home/robot/save2.jpg",frame);
            ros::spinOnce();
            cv::waitKey(1);
        }

    // while(!stop)
    // {
    //     // cap>>frame;
    //     cout<<"read"<<endl;
    //     cv::imshow("img", frame);
    //     // cv::cvtColor(frame,edges,CV_BGR2GRAY);
    //     srv.request.image = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    //     if (client.call(srv)) {
    //         cout << "----------------" << endl;
    //         cout << "detect: " << srv.response.results.size() << " objects" << endl;
    //         for (auto &result:srv.response.results) {
    //             auto xyxy = result.bbox.xyxy;
    //             cv::Point p1(xyxy[0], xyxy[1]), p2(xyxy[2], xyxy[3]), wh = p2 - p1;
    //             auto thickness = cv::min(wh.x, wh.y);
    //             cv::rectangle(frame, p1, p2, cv::Scalar(255, 0, 0), thickness / 40 + 1);
    //             cv::putText(frame, result.label, p1, cv::FONT_HERSHEY_COMPLEX,
    //                         1, cv::Scalar(0, 0, 255),
    //                         1, 0);
    //             cout << result.label << endl;
    //         }
    //         //}

    //         cv::imshow("img", frame);
    //         ros::spinOnce();
    //         cv::waitKey(1);
    //     }
    // }

    
    return 0;
}
*/
