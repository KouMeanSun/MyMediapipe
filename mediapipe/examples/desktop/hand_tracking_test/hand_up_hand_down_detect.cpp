#include "hand_up_hand_down_detect.h"

GoogleMediapipeHandTrackingDetect::HandUpHandDownDetect::HandUpHandDownDetect()
{

}


GoogleMediapipeHandTrackingDetect::HandUpHandDownDetect::~HandUpHandDownDetect()
{

}


int GoogleMediapipeHandTrackingDetect::HandUpHandDownDetect::DetectHandUpOrHandDown(int point_list[][2], int origin_image_height,int upDownthresholdValue)
{
    int result = -1;
    int thresholdValue = upDownthresholdValue;
    int y0 = point_list[0][1];
    int y1 = point_list[1][1];
    int y2 = point_list[2][1];
    // int y3 = point_list[3][1];
    // int y4 = point_list[4][1];

    // if ((y1 - y0 > thresholdValue) && (y2 - y1 > thresholdValue) && (
    //         y3 - y2 > thresholdValue) && (y4 - y3 > thresholdValue)){
    //     result =  HandUp_HandDown::HandDown;
    // }else if((y0 - y1 > thresholdValue) && (y1 - y2 > thresholdValue) && (
    //         y2 - y3 > thresholdValue) && (y3 - y4 > thresholdValue)){
    //     result = HandUp_HandDown::HandUp;
    // }else {
    //     result = HandUp_HandDown::NoHand;
    // }

    if ((y1 - y0 > thresholdValue) && (y2 - y1 > thresholdValue)){
        result =  HandUp_HandDown::HandDown;
    }else if((y0 - y1 > thresholdValue) && (y1 - y2 > thresholdValue)){
        result = HandUp_HandDown::HandUp;
    }else {
        result = HandUp_HandDown::NoHand;
    }


    return result;
}