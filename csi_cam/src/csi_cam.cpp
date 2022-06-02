#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>


using namespace std;
using namespace cv;
const int lowh = 168;
const int lows = 0;
const int lowv = 0;
const int highh = 181;
const int highs = 255;
const int highv = 255;
ros::Publisher centerPointPub;



string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method)
{
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + to_string(capture_width) + ", height=(int)" +
           to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + to_string(flip_method) + " ! video/x-raw, width=(int)" + to_string(display_width) + ", height=(int)" +
           to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}




int main( int argc, char** argv )
{
    ros::init(argc,argv,"csi_camera");
    ros::NodeHandle nh;
    ros::Time::init();

    centerPointPub = nh.advertise<std_msgs::Float32MultiArray>("/tracker/pos_image",1);

    int capture_width = 1280 ;
    int capture_height = 720 ;
    int display_width = 1280 ;
    int display_height = 720 ;
    int framerate = 60 ;
    int flip_method = 0 ;

    //创建管道
    string pipeline = gstreamer_pipeline(capture_width,
    capture_height,
    display_width,
    display_height,
    framerate,
    flip_method);
    std::cout << "使用gstreamer管道: \n\t" << pipeline << "\n";

    //管道与视频流绑定
    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if(!cap.isOpened())
    {
        std::cout<<"打开摄像头失败."<<std::endl;
        return (-1);
    }

    //创建显示窗口
    // namedWindow("CSI Camera", WINDOW_AUTOSIZE);
    Mat img;

    //逐帧显示
    while(ros::ok())
    {
        if (!cap.read(img))
        {
            std::cout<<"捕获失败"<<std::endl;
            break;
        }
	flip(img,img,0);
	flip(img,img,1);

        imshow("CSI Camera",img);


        int keycode = cv::waitKey(30) & 0xff ; //ESC键退出
        if (keycode == 27) 
            break ;
        ros::spinOnce();
    }
    

    cap.release();
    destroyAllWindows() ;
}
