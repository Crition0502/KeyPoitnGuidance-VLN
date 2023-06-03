/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/* @author Zhang Wanjie                                             */

#include "action_manager.h"
#include"Depth_TO_PointCloud.h"



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
static ros::Publisher spk_pub;
static ros::Publisher vel_pub;
static string strToSpeak = "";
static string strKeyWord = "";
static ros::ServiceClient clientIAT;
static xfyun_waterplus::IATSwitch srvIAT;
static ros::ServiceClient cliGetWPName;
static waterplus_map_tools::GetWaypointByName srvName;
static ros::ServiceClient follow_start;
static ros::ServiceClient follow_stop;
static wpb_home_tutorials::Follow srvFlw;
static ros::Publisher behaviors_pub;
static std_msgs::String behavior_msg;
static ros::Publisher add_waypoint_pub;
static int nToRecoFrame = 100;
string target_place="kitchen_observation";
string target="drinks";
static ros::Publisher cmd_vel_pub_;
static ros::ServiceClient client;
static ros::Subscriber ColorImage;
static ros::Subscriber DepthImage;

CActionManager::CActionManager()
{
    nCurActIndex = 0;
    nCurActCode = -1;
    strListen = "";
    bGrabDone = false;
    bPassDone = false;
    nVideoFrameCount = 0;
    pVW = NULL;
}

CActionManager::~CActionManager()
{

}

void CActionManager::ProcColorCB(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("[ProcColorCB - ]...");
    if(nCurActCode != ACT_REC_VIDEO)
        return;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image = cv_ptr->image;

    if(pVW != NULL)
    {
        ROS_INFO("[rec video - %d]...",nVideoFrameCount);
        *pVW << image;
        nVideoFrameCount ++;
    }

    // //image_pub.publish(cv_ptr->toImageMsg());
 
    // imwrite("/home/robot/objects.jpg",cv_ptr->image);
    // ROS_INFO("Save the image of object recognition!!");
    // spk_pub.publish("OK,I have recognize the objects.");
}

void CActionManager::Init()
{
    ros::NodeHandle n;
    cliGetWPName = n.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    spk_pub = n.advertise<std_msgs::String>("/xfyun/tts", 20);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    clientIAT = n.serviceClient<xfyun_waterplus::IATSwitch>("xfyun_waterplus/IATSwitch");
    follow_start = n.serviceClient<wpb_home_tutorials::Follow>("wpb_home_follow/start");   //连接跟随开始的服务
    follow_stop = n.serviceClient<wpb_home_tutorials::Follow>("wpb_home_follow/stop");      //连接跟随停止的服务
    behaviors_pub = n.advertise<std_msgs::String>("/wpb_home/behaviors", 30);
    add_waypoint_pub = n.advertise<waterplus_map_tools::Waypoint>( "/waterplus/add_waypoint", 1);
    grab_result_sub = n.subscribe<std_msgs::String>("/wpb_home/grab_result",30,&CActionManager::GrabResultCallback,this);
    pass_result_sub = n.subscribe<std_msgs::String>("/wpb_home/pass_result",30,&CActionManager::PassResultCallback,this);
    cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    client = n.serviceClient<ros_yolo::yolo>("yolo_service");
    
}

static void FollowSwitch(bool inActive, float inDist)
{
    if(inActive == true)
    {
        srvFlw.request.thredhold = inDist;
        if (!follow_start.call(srvFlw))
        {
            ROS_WARN("[CActionManager] - follow start failed...");
        }
    }
    else
    {
        if (!follow_stop.call(srvFlw))
        {
            ROS_WARN("[CActionManager] - failed to stop following...");
        }
    }
}

static void GrabSwitch(bool inActive)
{
    if(inActive == true)
    {
        behavior_msg.data = "grab start";
        behaviors_pub.publish(behavior_msg);
    }
    else
    {
        behavior_msg.data = "grab stop";
        behaviors_pub.publish(behavior_msg);
    }
}

static void PassSwitch(bool inActive)
{
    if(inActive == true)
    {
        behavior_msg.data = "pass start";
        behaviors_pub.publish(behavior_msg);
    }
    else
    {
        behavior_msg.data = "pass stop";
        behaviors_pub.publish(behavior_msg);
    }
}

static void AddNewWaypoint(string inStr)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.waitForTransform("/map","/base_footprint",  ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/map","/base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("[lookupTransform] %s",ex.what());
        return;
    }

    float tx = transform.getOrigin().x();
    float ty = transform.getOrigin().y();
    tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(transform.getRotation() , tf::Point(tx, ty, 0.0)), ros::Time::now(), "map");
    geometry_msgs::PoseStamped new_pos;
    tf::poseStampedTFToMsg(p, new_pos);

    waterplus_map_tools::Waypoint new_waypoint;
    new_waypoint.name = inStr;
    new_waypoint.pose = new_pos.pose;
    add_waypoint_pub.publish(new_waypoint);

    ROS_WARN("[New Waypoint] %s ( %.2f , %.2f )" , new_waypoint.name.c_str(), tx, ty);
}


/*
这里是新加 的
*/

float current_x;
float current_y;
float target_x;
float target_y;
float theta[4]={0,0,0,0};
// double Yaw=0;
double pos_Yaw=0;
bool incallback=0;
float pos_target_x;
float pos_target_y;
bool getSuccess=0;

Mat rgb, depth;
Mat realdepth;
struct timeval tv;
string topic1_name = "/kinect2/qhd/image_color"; //topic 名称
string topic2_name = "/kinect2/qhd/image_depth_rect";

string save_imagedata = "/home/robot/catkin_ws/recordData/RGBD/";
string rgbPath,depthPath;
void GProcColorCB(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr=cv_bridge::toCvCopy(msg,msg->encoding);//BGR8
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }

    bool cg=imwrite("/home/robot/test_ws/recordData/RGBD/catch.png",cv_ptr->image);
    if(cg)
    {
        cout<<"succcess1"<<endl;
    }
    ROS_INFO("Save the image of object recognition!!");
}
void GProcDepthCB(const sensor_msgs::ImageConstPtr& msg)
{
    
    // cv_bridge::CvImageConstPtr pCvImage;

    cv_bridge::CvImageConstPtr cv_ptr;
    try{
        cv_ptr=cv_bridge::toCvShare(msg,msg->encoding);
        cout<<"encoding way: "<<msg->encoding<<endl;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }

    bool cg=imwrite("/home/robot/test_ws/recordData/RGBD/catchDepth.png",cv_ptr->image);
    cv_ptr->image.copyTo(realdepth);
    if(cg)
    {
        cout<<"succcess2"<<endl;
    }
    ROS_INFO("Save the image of object recognition!!");
}
static void toEulerAngle(const double x,const double y,const double z,const double w, double& roll, double& pitch, double& yaw)
{
// roll (x-axis rotation)
    double sinr_cosp = +2.0 * (w * x + y * z);
    double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
    double sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

// yaw (z-axis rotation)
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
//    return yaw;
}

int StatisticalDepthInfo(cv::Mat depthImage,float x1,float x2,float y1,float y2)
{   
    float real_depth=0;
    vector<double> pointd;
    int hist[4001]={0};
    int valid_pic=0;
    for(int i=int(y1);i<int(y2);i++)
    {
        for(int j=int(x1);j<int(x2);j++)
        {
            int tmp_depth=int (depthImage.at<ushort>(i, j) );
            if(tmp_depth>150&&tmp_depth<4000)//前面150mm和后面150mm都去掉
            {
                hist[ tmp_depth ] ++;
                valid_pic++;
            }
        }
        cout<<endl;
    }
    int tmpnum=0;
    for(int i=1;i<4000;i++)
    {
        if(tmpnum<hist[i])
        {
            real_depth=i;
            tmpnum=hist[i];
        }
    }
    // int tmpnum=0;
    // for(int i=0;i<4000;i++)
    // {
    //     if(tmpnum==valid_pic/2)
    //     {
    //         real_depth=i;
    //     }
    //     if(hist[i]!=0)
    //     {
    //         tmpnum++;
    //     }
    // }
    return real_depth;
}

int avaliable_radius_max=1.4*20;//1.5米 30
int avaliable_radius_min=0.25*20;//0.5米 10
vector<int> cycle_point;

void getAvaliableGoalCB(const nav_msgs::OccupancyGridConstPtr& map)
{
    incallback=1;
    int height=map->info.height;
    int width=map->info.width;
    float resolution=map->info.resolution;
    cout<<"高度宽度"<<height<<"================="<<width<<endl<<endl<<endl;
    cout<<resolution<<endl;
    for(int i=0;i<avaliable_radius_max*2;i++)
    {
        for(int j=0;j<avaliable_radius_max*2;j++)
        {
            int l=abs(i-avaliable_radius_max)*abs(i-avaliable_radius_max)+abs(j-avaliable_radius_max)*abs(j-avaliable_radius_max);
            int delta_radius=avaliable_radius_max-avaliable_radius_min;
            if( abs(l-delta_radius*delta_radius) < 3)
            {
                cycle_point.push_back(i);
                cycle_point.push_back(j);
            }
        }
    }
    std::vector<int> target_area;
    int pixcel_x=2000 - int( target_x*20);//cout<<"pixcelx"<<pixcel_x<<endl;//174
    int pixcel_y=2000 + int( target_y*20);//cout<<pixcel_y<<endl;//100
    for(int i=pixcel_x-avaliable_radius_max;i<pixcel_x+avaliable_radius_max;i++)
    {
        for(int j=pixcel_y-avaliable_radius_max;j<pixcel_y+avaliable_radius_max;j++)
        {
            if(i<0 || i>10800 || j<0 || j>19200)
            {
                target_area.push_back('-1');
            }
            else
            {
                target_area.push_back(int( map->data[i*width+j] ) );
            }
        }
    }

    int avaliable_flag[2]={0};
    for(int i=0;i<cycle_point.size();i++)
    {
        cout<<cycle_point[i]<<"-------------------"<<endl;;
    }
    for(int i=0;i<cycle_point.size();i=i+2)//对于每个点
    {
        int flag=0;
        cout<<cycle_point[i]<<" "<<cycle_point[i+1]<<"="<<avaliable_radius_min<<endl;
        for(int j=cycle_point[i]-avaliable_radius_min;j<cycle_point[i]+avaliable_radius_min;j++)
        {
            for(int k=cycle_point[i+1]-avaliable_radius_min;k<cycle_point[i+1]+avaliable_radius_min;k++)
            {
                if(target_area[j * (avaliable_radius_max*2) + k] != '\0')
                {
                    flag=1;//有不可达的地方
                }
            }
        }
        cout<<endl;
        if(flag==0)//all avaliable
        {
            avaliable_flag[0]=cycle_point[i];
            avaliable_flag[1]=cycle_point[i+1];
            break;
        }
    }
    cout<<avaliable_flag[0]<<"=================================="<<avaliable_flag[1]<<endl<<endl;
    cout<<"******pixcel_x**********pixcel_y*****************"<<pixcel_x<<"*"<<pixcel_y<<endl;
    pos_target_x=0 - float( pixcel_x - 2000 + (avaliable_flag[0] - avaliable_radius_max) ) / 20;
    pos_target_y=float( pixcel_y - 2000 + (avaliable_flag[1] - avaliable_radius_max) ) / 20;
    cout<<"******pos_target_x**********pos_target_y*****************"<<pos_target_x<<"*"<<pos_target_y<<endl;

    float vec_x=target_x-pos_target_x;
    float vec_y=target_y-pos_target_y;
    pos_Yaw=0;
    if(vec_x>0 )
    {
        pos_Yaw=atan( vec_x/vec_y ) ;
    }
    else if(vec_x<0 )
    {
        pos_Yaw=atan( vec_x/vec_y ) + 3.1415936 ;
    }
    
    cout<<"*****************pos_Yaw*********************"<<pos_Yaw<<endl;

}



// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参,待测试 kinect 2.0 学习笔记_深度相机内参
const double camera_factor = 1000;
const double camera_cx =  945.587;//623.827;// 325.5;
const double camera_cy =520.070;//354.457;// 253.5;
const double camera_fx = 1068.871;//699.751;// 518.0;
const double camera_fy = 1066.012;//699.751;// 519.0;


static int nLastActCode = -1;
static geometry_msgs::Twist vel_cmd;
bool CActionManager::Main()
{
    int nNumOfAct = arAct.size();
    map<string , string> mapT;mapT.insert(pair<string , string >("可乐" , "kitchen_observation"));mapT.insert(pair<string , string >("book" , "study"));
    map<string , string>::iterator iter;
    int isFind=-1;
    if(nCurActIndex >= nNumOfAct)
    {
        return false;
    }
    int nKeyWord = -1;
    nCurActCode = arAct[nCurActIndex].nAct;
    // navigation_point=0;
    switch (nCurActCode)
	{
	case ACT_GOTO:
		if (nLastActCode != ACT_GOTO)
		{
            FollowSwitch(false, 0);
			// string strGoto = arAct[nCurActIndex].strTarget;
            cout<<target_place<<endl;
            string strGoto = target_place;
            cout<<strGoto<<endl;
            printf("[ActMgr] %d - Goto %s",nCurActIndex,strGoto.c_str());
            srvName.request.name = strGoto;
            cout<<srvName.request.name<<endl;
            if (cliGetWPName.call(srvName))
            {
                cout<<srvName.request.name<<endl;
                cout<<strGoto<<endl;
                std::string name = srvName.response.name;
                float x = srvName.response.pose.position.x;current_x=x;
                float y = srvName.response.pose.position.y;current_y=y;
                theta[0]=srvName.response.pose.orientation.x;
                theta[1]=srvName.response.pose.orientation.y;
                theta[2]=srvName.response.pose.orientation.z;
                theta[3]=srvName.response.pose.orientation.w;cout<<theta[0]<<"___"<<theta[1]<<"___"<<theta[2]<<"___"<<theta[3]<<"-----------------------------------"<<endl;
                ROS_INFO("fristGet_wp_name: name = %s (%.2f,%.2f)", strGoto.c_str(),x,y);//地点位置和坐标

                MoveBaseClient ac("move_base", true);
                if(!ac.waitForServer(ros::Duration(5.0)))
                {
                    ROS_INFO("The move_base action server is no running. action abort...");
                }
                else
                {
                    move_base_msgs::MoveBaseGoal goal;
                    goal.target_pose.header.frame_id="map";
                    goal.target_pose.header.stamp=ros::Time::now();
                    goal.target_pose.pose = srvName.response.pose;
                    ac.sendGoal(goal);
                    ac.waitForResult();
                    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                        ROS_INFO("Arrived at %s!",strGoto.c_str());
                    else
                        ROS_INFO("Failed to get to %s ...",strGoto.c_str() );
                }
                
            }
            else
            {
                ROS_ERROR("Failed to call service GetWaypointByName");
            }
            nCurActIndex ++;
        }
		break;

	case ACT_FIND_OBJ:
		if (nLastActCode != ACT_FIND_OBJ)
		{
            printf("[ActMgr] %d - Find %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            ros::NodeHandle nh;
            int navigation_point=0;
            while(true)
            {
                cout<<"sleep"<<endl;
                int de=2;
                while(de--)//wait a minite to make sure that the camera catch pictures
                {
                    cout<<"sleep"<<de<<"s"<<endl;
                    sleep(1);  
                }
                cout<<"start photo"<<endl;
                
                remove("/home/robot/test_ws/recordData/RGBD/catch.png");
                remove("/home/robot/test_ws/recordData/RGBD/catchDepth.png");

                ColorImage = nh.subscribe("/kinect2/hd/image_color",1,GProcColorCB);
                std::string im_pth = "/home/robot/test_ws/recordData/RGBD/catch.png";
                cv::Mat frame;
                while(true)
                {
                    ros::spinOnce();
                    frame=cv::imread(im_pth);
                    if(!frame.data)
                    {
                        cout<<"get color failed"<<endl;
                    }
                    else{
                        cout<<"get the color image"<<endl;
                        break;
                    }
                }
                ColorImage.shutdown();
                DepthImage = nh.subscribe("/kinect2/hd/image_depth_rect",1,GProcDepthCB);
                im_pth = "/home/robot/test_ws/recordData/RGBD/catchDepth.png";
                cv::Mat frame_depth;
                while(true)
                {
                    ros::spinOnce();
                    frame_depth=cv::imread(im_pth);
                    if(!frame_depth.data)
                    {
                        cout<<"get depth failed"<<endl;
                    }
                    else{
                        cout<<"get the deep image"<<endl;
                        break;
                    }
                }
                DepthImage.shutdown();
                client.waitForExistence(ros::Duration(30e-3));
                
                
                ros_yolo::yolo srv;
                srv.request.image = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

                float x1=0,y1=0,x2=0,y2=0;
                int thickness;
                double score=0.0;
                target="bottle";
                string label="";
                if (client.call(srv)) {
                    cout<<"detect"<<srv.response.results.size()<<"objects"<<endl;
                    for (auto &result:srv.response.results) {
                        #include <typeinfo> 
                        cout<<(result.label).substr(0,6)<<endl;
                        int label_len=(result.label).length();
                        // score=atof((result.label).c_str());
                        if(label_len>target.length() && (result.label).substr(0,target.length())==target)
                        {
                            string sc=(result.label).substr(label_len-4,4);
                            cout<<atof(sc.c_str())<<endl;
                            if(score<atof(sc.c_str()))
                            {
                                score=atof(sc.c_str());
                                auto xyxy = result.bbox.xyxy;
                                x1=xyxy[0];y1=xyxy[1];x2=xyxy[2];y2=xyxy[3];
                                cout<<"=====#########====="<<xyxy[0]<<" "<<xyxy[1]<<" "<<xyxy[2]<<" "<<xyxy[3]<<" "<<endl;
                                label=result.label;
                            }
                        }
                    }
                }
                //get the max score target
                cv::Point p1(x1, y1), p2(x2, y2), wh = p2 - p1;
                thickness = cv::min(wh.x, wh.y);
                cv::rectangle(frame, p1, p2, cv::Scalar(255, 0, 0), thickness / 40 + 1);
                cv::rectangle(frame_depth, p1, p2, cv::Scalar(255, 0, 0), thickness / 40 + 1);//realdepth
                // cv::rectangle(realdepth, p1, p2, cv::Scalar(25dep_obj5, 0, 0), thickness / 40 + 1);
                cv::putText(frame, label, p1, cv::FONT_HERSHEY_COMPLEX,
                            1, cv::Scalar(0, 0, 255),
                            1, 0);
                cout << label << endl;

                cv::imwrite("/home/robot/test_ws/recordData/draw2.png",frame_depth);
                cout<<score<<endl;

                if(score>=0.4)
                {
                    const int LowDepthThreshold = 800;
                    const int HighDepthThreshold = 4000;

                    int center_x=int((x1+x2)/2);
                    int center_y=int((y1+y2)/2);
                    cout<<center_x<<"      "<<center_y<<endl;
                    cv::Point p(center_x, center_y);//初始化点坐标为(20,20)
                    
                    //遍历深度像素数组，并获取深度值.并统计每一个深度值出现的次数


                    //前13位为深度值，单位毫米
                    int depth;
                    bool validDepth=0;
                    // depth = depth_obj >> DepthImageFrame.PlayerIndexBitmaskWidth;

                    //有效视距内的统计
                    if (depth >= LowDepthThreshold && depth <= HighDepthThreshold)
                    {
                        validDepth=1;
                    }
                    #include <bitset>
                    ushort depth_obj=realdepth.at<ushort>(center_y, center_x);//需要注意的是，这里的x和y需要调换位置
                    cout<<"compu"<<endl;
                    int dep_obj = StatisticalDepthInfo(realdepth,x1,x2,y1,y2);
                    cout<<"compuend"<<endl;
                    // for(int ii=int(y1)-20;ii<int(y2)+20;ii++)
                    // {
                    //     for(int jj=int(x1)-20;jj<int(x2)+20;jj++)
                    //     {
                    //         ushort depth_obj2=realdepth.at<ushort>(ii, jj);//需要注意的是，这里的x和y需要调换位置
                    //         cout<<setw(4)<<setfill('0')<<depth_obj2<<" ";
                    //     }
                    //     cout<<endl;
                    // }
                    cout << bitset<sizeof(ushort) * 8>(depth_obj) << endl;
                    cout<<depth_obj<<endl<<endl;
                    cout<<dep_obj<<endl;

                    string imageRGBPath="/home/robot/test_ws/recordData/draw.png";
                    string imageDepthPath="/home/robot/test_ws/recordData/drawDepth.png";

                    // Depth_TO_PointCloud(imageRGBPath,imageDepthPath,x1,x2,y1,y2);
                    // PointT p;
                    // 计算这个点的空间坐标
                    double pz = double(depth_obj) / camera_factor;cout<<pz<<endl;
                    double px = (center_x - camera_cx) * pz / camera_fx;cout<<px<<endl;
                    double py = (center_y - camera_cy) * pz / camera_fy;cout<<py<<endl;

                    double alpha=30;
                    double pi=3.1415926;
                    double cosalpha=cos((alpha/180)*pi);cout<<cosalpha<<endl;
                    double sinalpha=sin((alpha/180)*pi);cout<<sinalpha<<endl;
                    double dXw=px;cout<<dXw<<endl;
                    double dYw=pz;cout<<dYw<<endl;
                    double dZw=py;cout<<dZw<<endl;

                    double Pitch=0,Roll=0,Yaw=0;
                    toEulerAngle(theta[0],theta[1],theta[2],theta[3],Pitch,Roll,Yaw);
                    cout<<theta[0]<<theta[1]<<theta[2]<<theta[3]<<endl;
                    cout <<Yaw<< endl;

                    //使用旋转矩阵的算法，计算出坐标变化后的目标物体的坐标
                    target_x=current_x+dXw*sin(Yaw)+dYw*cos(Yaw);
                    target_y=0-current_y+(dXw*cos(Yaw)-dYw*sin(Yaw) );
                    cout<<"**************************************"<<current_x<<"*"<<current_y<<endl;
                    
                    cout<<"**************************************"<<target_x<<"*"<<target_y<<endl;
                    

                    // cv::imwrite("/home/robot/test_ws/recordData/drawDepth.png",frame_depth);
                    // cv::imwrite("/home/robot/test_ws/recordData/draw.png",frame);
                    

                    ros::Subscriber sub = nh.subscribe("/map",10000,getAvaliableGoalCB);
                    cout<<"do"<<endl;
                    while(!incallback)
                        ros::spinOnce();
                    cout<<"end"<<endl;
                    incallback=0;


                    MoveBaseClient ac("move_base", true);
                    if(!ac.waitForServer(ros::Duration(5.0)))
                    {
                        ROS_INFO("The move_base action server is no running. action abort...");
                    }
                    else
                    {
                        move_base_msgs::MoveBaseGoal goal ;
                        goal.target_pose.header.frame_id = "map";
                        goal.target_pose.header.stamp = ros::Time::now();
                        goal.target_pose.pose = srvName.response.pose;
                        goal.target_pose.pose.position.x=pos_target_x;
                        goal.target_pose.pose.position.y=pos_target_y;
                        geometry_msgs::Quaternion geo_q = tf::createQuaternionMsgFromYaw(pos_Yaw);
                        goal.target_pose.pose.orientation=geo_q;
                        ac.sendGoal(goal);
                        ac.waitForResult();
                        break;
                        // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                        //     ROS_INFO("Arrived at %s!",strGoto.c_str());
                        // else
                        //     ROS_INFO("Failed to get to %s ...",strGoto.c_str() );
                    }

                    nCurActIndex ++;
                }
                else if(score<0.4)
                {
                    navigation_point++;
                    string tmp_str="";
                    for(int i=0;i<target_place.length();i++)
                    {
                        if(target_place[i]!='_')
                        {
                            tmp_str+=target_place[i];
                        }
                        else
                        {
                            break;
                        }
                        
                    }
                    srvName.request.name = tmp_str + "_" +to_string(navigation_point) + "_" +"observation" ;
                    cout<<"gowhere"<< srvName.request.name<<endl;
                    if (cliGetWPName.call(srvName))
                    {
                        std::string name = srvName.response.name;
                        float x = srvName.response.pose.position.x;current_x=x;
                        float y = srvName.response.pose.position.y;current_y=y;
                        theta[0]=srvName.response.pose.orientation.x;
                        theta[1]=srvName.response.pose.orientation.y;
                        theta[2]=srvName.response.pose.orientation.z;
                        theta[3]=srvName.response.pose.orientation.w;cout<<theta[0]<<"___"<<theta[1]<<"___"<<theta[2]<<"___"<<theta[3]<<"-----------------------------------"<<endl;
                        ROS_INFO("hereGet_wp_name: name = %s (%.2f,%.2f)", srvName.request.name,x,y);//地点位置和坐标

                        MoveBaseClient ac("move_base", true);
                        if(!ac.waitForServer(ros::Duration(5.0)))
                        {
                            ROS_INFO("The move_base action server is no running. action abort...");
                        }
                        else
                        {
                            move_base_msgs::MoveBaseGoal goal;
                            goal.target_pose.header.frame_id="map";
                            goal.target_pose.header.stamp=ros::Time::now();
                            goal.target_pose.pose = srvName.response.pose;
                            ac.sendGoal(goal);
                            ac.waitForResult();
                            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                                ROS_INFO("Arrived at %s!",srvName.request.name);
                            else
                                ROS_INFO("Failed to get to %s ...",srvName.request.name );
                        }
                        
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service GetWaypointByName");
                        cout<<"can not find the "<<target<<endl;
                        navigation_point=0;
                    }
                }
            }
		}
		break;

	case ACT_GRAB:
		if (nLastActCode != ACT_GRAB)
		{
            printf("[ActMgr] %d - Grab %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            bGrabDone = false;
            GrabSwitch(true);
		}
        if(bGrabDone == true)
        {
            printf("[ActMgr] %d - Grab %s done!\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            GrabSwitch(false);
            nCurActIndex ++;
        }
		break;

	case ACT_PASS:
		if (nLastActCode != ACT_PASS)
		{
            printf("[ActMgr] %d - Pass %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            bPassDone = false;
            PassSwitch(true);
		}
        if(bPassDone == true)
        {
            printf("[ActMgr] %d - Pass %s done!\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            PassSwitch(false);
            nCurActIndex ++;
        }
		break;

	case ACT_SPEAK:
		if (nLastActCode != ACT_SPEAK)
		{
            printf("[ActMgr] %d - Speak %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            strToSpeak = arAct[nCurActIndex].strTarget;
            std_msgs::String rosSpeak;
            rosSpeak.data = strToSpeak;
            spk_pub.publish(rosSpeak);
            strToSpeak = "";
            usleep(arAct[nCurActIndex].nDuration*1000*1000);
            nCurActIndex ++;
		}
		break;

	case ACT_LISTEN:
		if (nLastActCode != ACT_LISTEN)
		{
            printf("[ActMgr] %d - Listen %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            strListen = "";
            strKeyWord = arAct[nCurActIndex].strTarget;
            int nDur = arAct[nCurActIndex].nDuration;
            if(nDur < 3)
            {
                nDur = 3;
            }
            //开始语音识别
            srvIAT.request.active = true;
            srvIAT.request.duration = nDur;
            clientIAT.call(srvIAT);

		}
        for(iter = mapT.begin(); iter != mapT.end();iter++)//找物品
        {
            isFind=strListen.find( iter->first );
            if( isFind>= 0 )
            {
                target=iter->first;
                target_place=iter->second;cout<<target_place<<"listen"<<endl;
                nKeyWord=0;
            }
        }
        for(iter = mapT.begin(); iter != mapT.end();iter++)//找地方
        {
            isFind=strListen.find( iter->second );
            if(isFind >= 0 )
            {
                target_place=iter->second;
            }
        }

        isFind=strListen.find( "机器人" );
        if(isFind>=0)
        {
            nKeyWord=0;
            cout<<"jqren"<<endl;
        }
        // nKeyWord = strListen.find(strKeyWord);
        if(nKeyWord >= 0)
        {
            //识别完毕,关闭语音识别
            srvIAT.request.active = false;
            clientIAT.call(srvIAT);
            nCurActIndex ++;
            nKeyWord=-1;//重置状态
        }
		break;

    case ACT_MOVE:
        FollowSwitch(false, 0);
        printf("[ActMgr] %d - Move ( %.2f , %.2f ) - %.2f\n",nCurActIndex,arAct[nCurActIndex].fLinear_x,arAct[nCurActIndex].fLinear_y,arAct[nCurActIndex].fAngular_z);
        vel_cmd.linear.x = arAct[nCurActIndex].fLinear_x;
        vel_cmd.linear.y = arAct[nCurActIndex].fLinear_y;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = arAct[nCurActIndex].fAngular_z;
        vel_pub.publish(vel_cmd);

        usleep(arAct[nCurActIndex].nDuration*1000*1000);
        nCurActIndex ++;
		break;

	case ACT_FOLLOW:
		if (nLastActCode != ACT_FOLLOW)
		{
            printf("[ActMgr] %d - Follow dist = %.2f \n", nCurActIndex, arAct[nCurActIndex].fFollowDist);
            FollowSwitch(true, arAct[nCurActIndex].fFollowDist);
            nCurActIndex ++;
		}
		break;

    case ACT_ADD_WAYPOINT:
		if (nLastActCode != ACT_ADD_WAYPOINT)
		{
            printf("[ActMgr] %d - Add waypoint %s \n", nCurActIndex, arAct[nCurActIndex].strTarget.c_str());
            AddNewWaypoint(arAct[nCurActIndex].strTarget);
            nCurActIndex ++;
		}
		break;

    case ACT_REC_VIDEO:
		if (nLastActCode != ACT_REC_VIDEO)
		{
            printf("[ActMgr] %d - rec video start \n", nCurActIndex);
            nVideoFrameCount = 0;
            if(pVW != NULL)
            {
                delete pVW;
                pVW = NULL;
            }
            pVW = new VideoWriter(arAct[nCurActIndex].strTarget, CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(960, 540)); 
            nToRecoFrame = arAct[nCurActIndex].nDuration *25;
		}
        if ( nVideoFrameCount >= nToRecoFrame )
        {
            printf("[ActMgr] %d - rec done \n", nCurActIndex);
            nCurActIndex ++;
        }
		break;

    case ACT_PLAY_VIDEO:
		if (nLastActCode != ACT_PLAY_VIDEO)
		{
            printf("[ActMgr] %d - play video \n", nCurActIndex);
            std::stringstream ss;
            ss << "/home/robot/catkin_ws/src/wpb_home_apps/tools/ffplay -v 0 -loop " << arAct[nCurActIndex].nLoopPlay << " -i " << arAct[nCurActIndex].strTarget;
            system(ss.str().c_str());

            usleep(arAct[nCurActIndex].nDuration*1000*1000);
            nCurActIndex ++;
		}
        break;

    case ACT_CAP_IMAGE:
		if (nLastActCode != ACT_CAP_IMAGE)
		{
            printf("[ActMgr] %d - capture image \n", nCurActIndex);
            nVideoFrameCount = 0;
            strImage = arAct[nCurActIndex].strTarget;
		}
        if ( nVideoFrameCount > 0)
        {
            printf("[ActMgr] %d - capture image done \n", nCurActIndex);
            nCurActIndex ++;
        }
		break;


	default:
		break;
	}
	nLastActCode = nCurActCode;
    return true;
}

void CActionManager::Reset()
{
    strToSpeak = "";
    nCurActIndex = 0;
	nLastActCode = 0;
    arAct.clear();
}

string CActionManager::GetToSpeak()
{
    string strRet = strToSpeak;
    strToSpeak = "";
    return strRet;
}

string ActionText(stAct* inAct)
{
    string ActText = "";
    if(inAct->nAct == ACT_GOTO)
    {
        ActText = "去往地点 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_FIND_OBJ)
    {
        ActText = "搜索物品 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_GRAB)
    {
        ActText = "抓取物品 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_PASS)
    {
        ActText = "把物品递给 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_SPEAK)
    {
        ActText = "说话 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_LISTEN)
    {
        ActText = "听取关键词 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_MOVE)
    {
        ActText = "移动 ( ";
        std::ostringstream stringStream;
        stringStream << inAct->fLinear_x << " , " << inAct->fLinear_y << " ) - " << inAct->fAngular_z;
        std::string retStr = stringStream.str();
        ActText += retStr;
    }
    if(inAct->nAct == ACT_FOLLOW)
    {
        ActText = "跟随 距离为 ";
        std::ostringstream stringStream;
        stringStream << inAct->fFollowDist;
        std::string retStr = stringStream.str();
        ActText += retStr;
    }
    if(inAct->nAct == ACT_ADD_WAYPOINT)
    {
        ActText = "添加航点 ";
        std::ostringstream stringStream;
        stringStream << inAct->fFollowDist;
        std::string retStr = stringStream.str();
        ActText += retStr;
    }
    return ActText;
}

void CActionManager::ShowActs()
{
    printf("\n*********************************************\n");
    printf("显示行为队列:\n");
    int nNumOfAct = arAct.size();
    stAct tmpAct;
    for(int i=0;i<nNumOfAct;i++)
    {
        tmpAct = arAct[i];
        string act_txt = ActionText(&tmpAct);
        printf("行为 %d : %s\n",i+1,act_txt.c_str());
    }
    printf("*********************************************\n\n");
}

void CActionManager::GrabResultCallback(const std_msgs::String::ConstPtr& res)
{
    int nFindIndex = 0;
    nFindIndex = res->data.find("done");
    if( nFindIndex >= 0 )
    {
        bGrabDone = true;
    }
}

void CActionManager::PassResultCallback(const std_msgs::String::ConstPtr& res)
{
    int nFindIndex = 0;
    nFindIndex = res->data.find("done");
    if( nFindIndex >= 0 )
    {
        bPassDone = true;
    }
}