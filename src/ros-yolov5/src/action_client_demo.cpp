//
// Created by ou on 2021/3/18.
//
#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>

#include <cv_bridge/cv_bridge.h>
#include <ros_yolo/yolo.h>
#include <ros_yolo/yoloAction.h>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

void actionDoneCallBack(const actionlib::SimpleClientGoalState &state,
                        const ros_yolo::yoloResultConstPtr &result);

void spinfunc(void);

bool exportBbox(const ros_yolo::yoloResultConstPtr &result, const std::string &filename);

bool exportBboxWithConfidence(const ros_yolo::yoloResultConstPtr &result, const std::string &filename);

void splitImage(const std::string& txt_file, const std::string& image_file);


int main(int argc, char **argv) {
    ros::init(argc, argv, "client_node");
    ros::NodeHandle n;

    bool isAction = false;
    n.getParam("yolov5/action", isAction);
    if (!isAction) {
        ROS_ERROR("no in action mode, please modify the .yaml file and reload");
        return 0;
    }
    boost::thread thread_spin(spinfunc);
    // start client after spin_thread was up
    actionlib::SimpleActionClient<ros_yolo::yoloAction> client("yolo_action", true);
    client.waitForServer();
    ROS_INFO("action 'yolo_action' server exist ");


    cv::VideoCapture capture;
    capture.open(ros::package::getPath("ros_yolo") + "/resource/video1.mp4");
    cv::Mat frame;
    while (capture.isOpened()) {
        capture.read(frame);

        ros_yolo::yoloGoal goal;
        goal.image = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        client.sendGoal(goal, boost::bind(actionDoneCallBack, _1, _2));
        client.waitForResult();  // comment this line  when blocking is not needed
    }
    // exportBbox(result, "/home/robot/test_ws/src/ros-yolov5/res/bbox.txt");
    exportBboxWithConfidence(result, "/home/robot/test_ws/src/ros-yolov5/res/bbox.txt");
    // 定义txt文件名
    std::string txt_file = "/home/robot/test_ws/src/ros-yolov5/res/bbox.txt";
    // 定义原始图片文件名
    std::string image_file = "/home/robot/test_ws/src/ros-yolov5/resource/2019_0001.jpg";
    // 调用splitImage函数，根据txt数据中的坐标数据将原始图片进行分割
    splitImage(txt_file, image_file);
    thread_spin.join();
    return 0;
}

void spinfunc(void) {
    while (ros::ok()) {
        ros::spinOnce();
    }
}

void actionDoneCallBack(const actionlib::SimpleClientGoalState &state,
                        const ros_yolo::yoloResultConstPtr &result) {
    cout << "----------------" << endl;
    cout << "detect: " << result->results.size() << " objects" << endl;
    cv::Mat frame;
    frame = cv_bridge::toCvCopy(result->image, "bgr8")->image;
    for (auto &result:result->results) {
        auto xyxy = result.bbox.xyxy;
        cv::Point p1(xyxy[0], xyxy[1]), p2(xyxy[2], xyxy[3]), wh = p2 - p1;
        auto thickness = cv::min(wh.x, wh.y);
        cv::rectangle(frame, p1, p2, cv::Scalar(255, 0, 0), thickness / 40 + 1);
        cv::putText(frame, result.label, p1, cv::FONT_HERSHEY_COMPLEX,
                    1, cv::Scalar(0, 0, 255),
                    1, 0);
        cout << result.label << endl;
    }
    cv::imshow("img", frame);
    cv::waitKey(1);
}

// 将bbox中的xyxy数组导出到txt文档中
bool exportBbox(const ros_yolo::yoloResultConstPtr &result, const std::string &filename) {
    // 创建一个ofstream对象，用于写入文档
    std::ofstream ofs(filename);
    // 检查是否打开成功
    if (!ofs.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return false;
    }
    // 遍历result中的results数组
    for (auto &result:result.results) {
        auto xyxy = result.bbox.xyxy;
        ofs << result.label << " " << xyxy[0] << " " << xyxy[1] << " " << xyxy[2] << " " << xyxy[3] << std::endl;
    }
    
    ofs.close();
    return true;
}

// 将bbox中的xyxy数组导出到txt文档中,并根据置信度水平进行排序
void exportBboxWithConfidence(const ros_yolo::yoloResult& result, const std::string& filename) {
    std::vector<std::string> bbox_info;
    for (int i = 0; i < result.bounding_boxes.size(); i++) {
        std::string class_name = result.bounding_boxes[i].Class;
        double confidence = result.bounding_boxes[i].probability;
        int x1 = result.bounding_boxes[i].xmin;
        int y1 = result.bounding_boxes[i].ymin;
        int x2 = result.bounding_boxes[i].xmax;
        int y2 = result.bounding_boxes[i].ymax;
        std::string info = class_name + " " + std::to_string(confidence) + " " + std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(x2) + " " + std::to_string(y2);
        bbox_info.push_back(info);
    }
    // 对vector中的字符串按照置信度从大到小排序
    std::sort(bbox_info.begin(), bbox_info.end(), [](const std::string& a, const std::string& b) {
        std::vector<std::string> a_split = split(a, ' ');
        std::vector<std::string> b_split = split(b, ' ');
        double a_confidence = std::stod(a_split[1]);
        double b_confidence = std::stod(b_split[1]);
        // 比较置信度，返回true表示a排在b前面
        return a_confidence > b_confidence;
    });

    // 打开文件，如果不存在则创建
    std::ofstream file(filename);
    if (file.is_open()) {
        for (const auto& info : bbox_info) {
            file << info << "\n";
        }
        file.close();
    } else {
        std::cerr << "Failed to open file: " << filename << "\n";
    }
}

// 定义一个函数，用于根据txt数据中的坐标数据将原始图片进行分割
void splitImage(const std::string& txt_file, const std::string& image_file) {
    std::ifstream file(txt_file);
    if (file.is_open()) {
        std::vector<std::string> bbox_info;
        std::string line;
        while (std::getline(file, line)) {
            bbox_info.push_back(line);
        }
        file.close();
    } else {
        std::cerr << "Failed to open file: " << txt_file << "\n";
        return;
    }
    cv::Mat image = cv::imread(image_file);
    if (image.empty()) {
        std::cerr << "Failed to read image: " << image_file << "\n";
        return;
    }

    for (int i = 0; i < bbox_info.size(); i++) {
        std::vector<std::string> info_split = split(bbox_info[i], ' ');
        std::string class_name = info_split[0];
        double confidence = std::stod(info_split[1]);
        int x1 = std::stoi(info_split[2]);
        int y1 = std::stoi(info_split[3]);
        int x2 = std::stoi(info_split[4]);
        int y2 = std::stoi(info_split[5]);

        cv::Rect roi(x1, y1, x2 - x1, y2 - y1);
        cv::Mat cropped = image(roi);
        std::string filename = class_name + "_" + std::to_string(confidence) + "_" + std::to_string(i) + ".jpg";
        cv::imwrite(filename, cropped);
    }
}
