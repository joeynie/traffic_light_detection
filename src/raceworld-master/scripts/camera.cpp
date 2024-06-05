#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// ros::Subscriber camera_sub;

cv::Mat image;
int roix,roiy,rois;
bool checkLight(cv::Mat mat, cv::Scalar color)
{
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mat, mat, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mat, mat, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mat, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    // int maxi=-1;
    // double maxs=0;
    bool lightOn=false;;
    for(int i=0;i<contours.size();++i){
        double area = cv::contourArea(contours[i]);
        // printf("area/rois:%f %d\n",area,mat.cols *mat.rows);
        if (area*10 < rois/2 || area*10> rois*2)  continue;

        double perimeter = cv::arcLength(contours[i], true);
        
        // similarity to circle
        double circularity = 4 * CV_PI * (area / (perimeter * perimeter));
        
        if(circularity > 0.8 ){
            // if(area>maxs){  maxi=i; maxs=area;}
            lightOn = true;
            cv::Rect boundRect = cv::boundingRect(contours[i]);
            // cv::drawContours(image,contours,i,color,1);
            cv::Rect originalRect(boundRect.x + roix, boundRect.y + roiy, boundRect.width, boundRect.height);
            cv::rectangle(image, originalRect, color, 2);
        }
    }
    return lightOn;

}
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        ros::NodeHandle nh;
        image = cv_bridge::toCvShare(msg, "bgr8")->image;

        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::Mat binary;
        cv::threshold(gray, binary, 10, 255, cv::THRESH_BINARY_INV);
        // reduce noise
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
        // find contours
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        // get most possible frame
        std::vector<cv::Rect> roiList;
        for (size_t i = 0; i < contours.size(); i++)
        {
            //outer rectangle
            if(hierarchy[i][3]!=-1)continue;
            double area = cv::contourArea(contours[i]);
            //restrict area
            if (area < 800.0 || area > 10000.0)
                continue;
            cv::Rect boundRect = cv::boundingRect(contours[i]);
            //rectangle
            if(boundRect.height>1.5*boundRect.width && boundRect.height<2.5*boundRect.width){
                roiList.push_back(boundRect);
                cv::rectangle(image, boundRect.tl(), boundRect.br(), cv::Scalar(255, 255, 0), 2);
            }
        }
        //look for light colors
        for (const auto& roi : roiList)
        {
            cv::Mat hsv;
            cv::cvtColor(image(roi),hsv,cv::COLOR_BGR2HSV);
            cv::Mat red1,red2,yellow,green;
            inRange(hsv,cv::Scalar(0,43,180),cv::Scalar(25,255,255),red1);
            inRange(hsv,cv::Scalar(156,43,180),cv::Scalar(180,255,255),red2);
            inRange(hsv,cv::Scalar(26,43,180),cv::Scalar(34,255,255),yellow);
            inRange(hsv,cv::Scalar(35,43,180),cv::Scalar(77,255,255),green);

            roix=roi.x;
            roiy=roi.y;
            rois=roi.width * roi.height;
            bool isRed=(checkLight(red1|red2,cv::Scalar(0,0,255)));
            bool isGreen=(checkLight(green,cv::Scalar(0,255,0)));
            bool isYellow=(checkLight(yellow,cv::Scalar(0,255,255)));
            
            if(isRed) nh.setParam("LightOn",3);
            else if(isYellow) nh.setParam("LightOn",2);
            else if(isGreen) nh.setParam("LightOn",1);
            else nh.setParam("LightOn",0);
        }
        cv::imshow("Image", image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Failed to convert ROS image to OpenCV image: %s", e.what());
    }
}
int main(int argc,char **argv)
{
    ros::init(argc, argv, "image_sub");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/deepracer1/camera/zed_left/image_rect_color_left", 1, imageCallback);

    cv::namedWindow("Image",cv::WINDOW_NORMAL);
    ros::spin();

    cv::destroyAllWindows();
    return 0;
}
