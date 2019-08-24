//#include <ros/ros.h>
//#include <std_msgs/UInt8.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;


const int GREEN_1 = 70;
const int GREEN_2 = 100;
const int GREEN_3 = 180;
const int GREEN_4 = 250;

const int YELLOW_1 = 10;
const int YELLOW_2 = 50;
const int YELLOW_3 = 80;
const int YELLOW_4 = 250;

const int RED_1 = 160;
const int RED_2 = 180;
const int RED_3 = 180;
const int RED_4 = 250;

const Vec3b HSV_RED_LOWER = Vec3b(0, 100, 100);
const Vec3b HSV_RED_UPPER = Vec3b(10, 255, 255);
const Vec3b HSV_RED_LOWER1 = Vec3b(160, 100, 100);
const Vec3b HSV_RED_UPPER1 = Vec3b(190, 255, 255);

const Vec3b HSV_GREEN_LOWER = Vec3b(40, 200, 200);
const Vec3b HSV_GREEN_UPPER = Vec3b(120, 255, 255);

const Vec3b HSV_YELLOW_LOWER = Vec3b(10, 70, 130);
const Vec3b HSV_YELLOW_UPPER = Vec3b(50, 255, 255);

const Vec3b HSV_BLACK_LOWER = Vec3b(0, 0, 0);
const Vec3b HSV_BLACK_UPPER = Vec3b(180, 255, 50);

const int MAX_SIZE = 230;
const int MIN_SIZE = 30;
const int MAX_HEIGHT = 50;
const int MIN_HEIGHT = 10;

bool use_roi = true;

struct TrafficLight {
    int left;
    int top;
    int width;
    int height;
    bool red_on = false;
    bool yellow_on = false;
    bool left_on = false;
    bool green_on = false;
    int state = 0;
    TrafficLight() {

    }
    TrafficLight(int l, int t, int w, int h, int s) : left(l), top(t), width(w), height(h), state(s) {

    }
};

//ros::NodeHandler nh;

void detectHScolor(const cv::Mat& image, double minHue, double maxHue, double minSat, double maxSat, cv::Mat& mask){

    cv::Mat hsv;

    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> channels;

    cv::split(hsv, channels); //HSV 채널 분리

    //Hue 마스크 (0~255)

    cv::Mat mask1;

    cv::threshold(channels[0], mask1, maxHue, 255, cv::THRESH_BINARY_INV);

    cv::Mat mask2;

    cv::threshold(channels[0], mask2, minHue, 255, cv::THRESH_BINARY);

    cv::Mat hueMask;

    if(minHue<maxHue) hueMask = mask1 & mask2;

    else hueMask = mask1 | mask2;

    cv::threshold(channels[1], mask1, maxSat, 255, cv::THRESH_BINARY_INV);

    cv::threshold(channels[1], mask2, minSat, 255, cv::THRESH_BINARY);



    cv::Mat satMask;

    satMask = mask1 & mask2;


    mask = hueMask & satMask;

}

bool isTrafficLight(TrafficLight t) {

    return false;
}

void light_off(Mat img, Mat &dst) {
    Mat hscolor;
    Mat img_labels, stats, centroids;
    int num;

    // yellow light off
    Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    inRange(hsv, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, hscolor);
    // detectHScolor(img, YELLOW_1, YELLOW_2, YELLOW_3, YELLOW_4, hscolor);
    num = connectedComponentsWithStats(hscolor, img_labels, stats, centroids, 8, CV_32S);
    for(int i = 1; i < num; i++) {
        int area = stats.at<int>(i, CC_STAT_AREA);
        int left = stats.at<int>(i, CC_STAT_LEFT);
        int top = stats.at<int>(i, CC_STAT_TOP);
        int width = stats.at<int>(i, CC_STAT_WIDTH);
        int height = stats.at<int>(i, CC_STAT_HEIGHT);
        if(width> height * 1.5 || height > width * 1.5)
            continue;
        if(width < 10)
            continue;
        width += 20;
        left -= 10;

        Rect rc(left,top,width,height);
        rectangle(dst, rc, Scalar(0, 0, 0), FILLED);
//        dst = img;
    }

    //green light off
    inRange(hsv, HSV_GREEN_LOWER, HSV_GREEN_UPPER, hscolor);
    imshow("imddg", hscolor);

    // detectHScolor(img, GREEN_1, GREEN_2, GREEN_3, GREEN_4, hscolor);
    num = connectedComponentsWithStats(hscolor, img_labels, stats, centroids, 8, CV_32S);
    for(int i = 1; i < num; i++) {
        int area = stats.at<int>(i, CC_STAT_AREA);
        int left = stats.at<int>(i, CC_STAT_LEFT);
        int top = stats.at<int>(i, CC_STAT_TOP);
        int width = stats.at<int>(i, CC_STAT_WIDTH);
        int height = stats.at<int>(i, CC_STAT_HEIGHT);
        if(width> height * 1.5 || height > width * 1.5)
            continue;
//        if(width < 10)
//            continue;
        left -= 10;
        width += 10;
        Rect rc(left,top,width,height);
        rectangle(dst, rc, Scalar(0, 0, 0), FILLED);
//        dst = img;
    }

    //red light off
    Mat binaryImg1;
    Mat binaryImg2;

    inRange(hsv, HSV_RED_LOWER, HSV_RED_UPPER, binaryImg1);
    inRange(hsv, HSV_RED_LOWER1, HSV_RED_UPPER1, binaryImg2);

    hscolor = binaryImg1 | binaryImg2;

    // detectHScolor(img, RED_1, RED_2, RED_3, RED_4, hscolor);
    num = connectedComponentsWithStats(hscolor, img_labels, stats, centroids, 8, CV_32S);
    for(int i = 1; i < num; i++) {
        int area = stats.at<int>(i, CC_STAT_AREA);
        int left = stats.at<int>(i, CC_STAT_LEFT);
        int top = stats.at<int>(i, CC_STAT_TOP);
        int width = stats.at<int>(i, CC_STAT_WIDTH);
        int height = stats.at<int>(i, CC_STAT_HEIGHT);
        if(width> height * 1.5 || height > width * 1.5)
            continue;
        if(width < 10)
            continue;
        width += 20;
        Rect rc(left,top,width,height);
        rectangle(dst, rc, Scalar(0, 0, 0), FILLED);
//        dst = img;dfn A
    }

}

int getGrayColor(Mat &img, int x, int y) {
    int r = img.at<Vec3b>(y, x)[2];
    int g = img.at<Vec3b>(y, x)[1];
    int b = img.at<Vec3b>(y, x)[0];
    return 0.2126f * r + 0.7152f * g + 0.0722f * b;
}

int main(int argc, char** argv) {
//    ros::init(argc, argv, "traffic_light_node");
    VideoCapture cap("yellow.mov");
    if (!cap.isOpened()) return -1;


    while(1) {
        vector<TrafficLight> v;
//        Mat img = imread("green.png");
        Mat img;
        cap >> img;

        // 관심영역 설정 (set ROI (X, Y, W, H)).

        Rect rect((1920-1920/2)/2, 0, 1920/2, 600);
        // 관심영역 자르기 (Crop ROI).

        Mat subImage = img(rect);

        imshow("roi", subImage);


        // add contrast -> slow. skip.


        subImage.copyTo(img);
        Mat new_image = Mat::zeros( img.size(), img.type() );

        subImage.copyTo(new_image);


//        double alpha = 2;
//        int beta = 5;
//
//        for( int y = 0; y < img.rows; y++ ) {
//            for( int x = 0; x < img.cols; x++ ) {
//                for( int c = 0; c < 3; c++ ) {
//                    new_image.at<Vec3b>( y, x )[c] = saturate_cast<uchar>( alpha*( img.at<Vec3b>( y, x )[c] ) + beta );
//                }
//            }
//        }

        Mat hsv, res;
        Mat hsv1;

        // add blur
//        medianBlur(img, hsv, 7);
        img.copyTo(hsv);
        hsv = img;

        light_off(hsv, new_image);

        imshow("new_image", new_image);

        //    imshow("lightoff", new_image);

        // add binary
        Mat img_binary;
        cvtColor(new_image, img_binary, COLOR_BGR2GRAY);
        threshold(img_binary, img_binary, 125, 255, THRESH_BINARY_INV);


        imshow("binary", img_binary);

        cvtColor(hsv, res, COLOR_BGR2HSV);
        imshow("hsv", res);
        Mat hscolor;

        Mat img_labels, stats, centroids;
        int num;

        // inRange(hsvImg, HSV_RED_LOWER, HSV_RED_UPPER, binaryImg);
        Mat binaryImg1;
        Mat binaryImg2;
        inRange(res, HSV_RED_LOWER, HSV_RED_UPPER, binaryImg1);
        inRange(res, HSV_RED_LOWER1, HSV_RED_UPPER1, binaryImg2);

        hscolor = binaryImg1 | binaryImg2;


        // detectHScolor(img, RED_1, RED_2, RED_3, RED_4, hscolor);
//        imshow("red hscolor", hscolor);

        num = connectedComponentsWithStats(hscolor, img_labels, stats, centroids, 8, CV_32S);
        // idx 0은 전체 이미지
        for(int j = 1; j < num; j++) {
            int area = stats.at<int>(j, CC_STAT_AREA);
            int left = stats.at<int>(j, CC_STAT_LEFT);
            int top = stats.at<int>(j, CC_STAT_TOP);
            int width = stats.at<int>(j, CC_STAT_WIDTH);
            int height = stats.at<int>(j, CC_STAT_HEIGHT);

            if(use_roi) {
                if(width > height * 2 || height > width * 2)
                    continue;
                if(width > MAX_SIZE || width < MIN_SIZE || height < MIN_HEIGHT || height > MAX_HEIGHT)
                    continue;
            }

            for(int start = left; start < left + width * 6; start ++) {
                if(start == img_binary.cols)
                    break;
                if(img_binary.at<uchar>(top+height/2, start) == 0) {
                    width = start - left;
                    break;
                }
            }

            // 신호등 선택 후 ROI
            if(use_roi) {
                if(width < height * 2.5)
                    continue;
                if(width < 10)
                    continue;
//                float cnt = 0;
//                float max = width * height;
//
//                for(int i = top; i < top + height; i++) {
//                    for(int j = left; j < left + width; j++) {
//                        uchar r = new_image.at<Vec3b>(i, j)[2];
//                        uchar g = new_image.at<Vec3b>(i, j)[1];
//                        uchar b = new_image.at<Vec3b>(i, j)[0];
//                        if( r < 70 && g < 70 && b < 70) {
//                            cnt ++;
//                        }
//                    }
//                }
//                if(cnt / max < 0.8)
//                    continue;
            }


            v.push_back(TrafficLight(left, top, width, height, 0));

            rectangle(img, Point(left, top), Point(left+width, top+height), Scalar(0,0,255), 3);

        }

        inRange(res, HSV_YELLOW_LOWER, HSV_YELLOW_UPPER, hscolor);

        // detectHScolor(img, YELLOW_1, YELLOW_2, YELLOW_3, YELLOW_4, hscolor);

//        imshow("yellow hscolor", hscolor);

        num = connectedComponentsWithStats(hscolor, img_labels, stats, centroids, 8, CV_32S);
        // idx 0은 전체 이미지
        for(int j = 1; j < num; j++) {
            int area = stats.at<int>(j, CC_STAT_AREA);
            int left = stats.at<int>(j, CC_STAT_LEFT);
            int top = stats.at<int>(j, CC_STAT_TOP);
            int width = stats.at<int>(j, CC_STAT_WIDTH);
            int height = stats.at<int>(j, CC_STAT_HEIGHT);

            if(use_roi) {
                if(width > height * 2 || height > width * 2)
                    continue;
                if(width > MAX_SIZE || width < MIN_SIZE || height < MIN_HEIGHT || height > MAX_HEIGHT)
                    continue;
            }

            for(int start = left; start < left + width * 4; start ++) {
                if(start == img_binary.cols)
                    break;
                if(img_binary.at<uchar>(top+height/2, start) == 0) {
                    width = start - left;
                    break;
                }
            }

            for(int start = left; start > left - width * 2; start--) {
                if(start == 0)
                    break;
                if(img_binary.at<uchar>(top+height / 2, start) == 0) {
                    width += left - start;
                    left = start;
                    break;
                }
            }

            // 신호등 선택 후 ROI
            if(use_roi) {
                if(width < height * 2.5)
                    continue;
                if(width < 10)
                    continue;
//                float cnt = 0;
//                float max = width * height;
//
//                for(int i = top; i < top + height; i++) {
//                    for(int j = left; j < left + width; j++) {
//                        uchar r = new_image.at<Vec3b>(i, j)[2];
//                        uchar g = new_image.at<Vec3b>(i, j)[1];
//                        uchar b = new_image.at<Vec3b>(i, j)[0];
//                        if( r < 70 && g < 70 && b < 70) {
//                            cnt ++;
//                        }
//                    }
//                }
//                if(cnt / max < 0.5)
//                    continue;
            }


            v.push_back(TrafficLight(left, top, width, height, 1));

            rectangle(img, Point(left, top), Point(left+width, top+height), Scalar(0,255,255),3);

        }


        imshow("imgorigin", hsv);


        inRange(res, HSV_GREEN_LOWER, HSV_GREEN_UPPER, hscolor);


        // detectHScolor(img, GREEN_1, GREEN_2, GREEN_3, GREEN_4, hscolor);
       imshow("green_hscolor", hscolor);

        num = connectedComponentsWithStats(hscolor, img_labels, stats, centroids, 8, CV_32S);
        // idx 0은 전체 이미지
        for(int j = 1; j < num; j++) {
            int area = stats.at<int>(j, CC_STAT_AREA);
            int left = stats.at<int>(j, CC_STAT_LEFT);
            int top = stats.at<int>(j, CC_STAT_TOP);
            int width = stats.at<int>(j, CC_STAT_WIDTH);
            int height = stats.at<int>(j, CC_STAT_HEIGHT);

            if(use_roi) {
                if(width > height * 2 || height > width * 2)
                    continue;
                if(width > MAX_SIZE || width < MIN_SIZE || height < MIN_HEIGHT || height > MAX_HEIGHT)
                    continue;
            }

            for(int start = left; start > left - width * 5; start --) {
                if(start == 0)
                    break;
                if(img_binary.at<uchar>(top+height / 2, start) == 0) {
                    width += left - start;
                    left = start;
                    break;
                }
            }

            // 신호등 선택 후 ROI
            if(use_roi) {
                if(width < height * 2.5)
                    continue;

                if(width < 30)
                    continue;
//                float cnt = 0;
//                float max = width * height;
//
//                for(int i = top; i < top + height; i++) {
//                    for(int j = left; j < left + width; j++) {
//                        uchar r = new_image.at<Vec3b>(i, j)[2];
//                        uchar g = new_image.at<Vec3b>(i, j)[1];
//                        uchar b = new_image.at<Vec3b>(i, j)[0];
//                        if( r < 70 && g < 70 && b < 70) {
//                            cnt ++;
//                        }
//                    }
//                }
//                if(cnt / max < 0.8)
//                    continue;
            }

            v.push_back(TrafficLight(left, top, width, height, 2));
            rectangle(img, Point(left, top), Point(left+width, top+height), Scalar(0,255,0), 3);
        }

        // 신호등 판별 필요

        int max_size = 0;
        int max_idx = 0;
        for(int i = 0; i < v.size(); i ++) {
            int tmp = v[i].width * v[i].height;
            if(max_size < tmp) {
                max_size = tmp;
                max_idx = i;
            }
        }

        if(v.size() > 0) {
            cout << v[0].width << " " << v[1].height << endl;
            int data = 0;
            if(v[max_idx].red_on) data += 8;
            if(v[max_idx].yellow_on) data += 4;
            if(v[max_idx].left_on) data += 2;
            if(v[max_idx].green_on) data += 1;

//            ros::Publisher traffic_light_pub = nh.advertise<std_msgs::UInt8>("traffic_light", 1);
//            UInt8 msg;
//            msg.data = data;
        }

        imshow("img", img);
        waitKey(0);
        // if (waitKey(5) >= 0)
        //     break;
    }

}
