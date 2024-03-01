//
// Created by GMY on 2022/2/17.
//

#include "hand_left_hand_right_detect.h"


GoogleMediapipeHandTrackingDetect::HandLeftHandRightDetect::HandLeftHandRightDetect()
{

}

GoogleMediapipeHandTrackingDetect::HandLeftHandRightDetect::~HandLeftHandRightDetect()
{

}

int GoogleMediapipeHandTrackingDetect::HandLeftHandRightDetect::DetectHandLeftOrHandRight(int point_list[][2], int origin_image_height,int leftRightthresholdValue)
{
    int result = -1;

    int thresholdValue = leftRightthresholdValue;
    int x0 = point_list[0][0];
    int x1 = point_list[1][0];
    int x2 = point_list[2][0];
    // int x3 = point_list[3][0];
    // int x4 = point_list[4][0];

    // if ((x1 - x0 > thresholdValue) && (x2 - x1 > thresholdValue) && (
    //         x3 - x2 > thresholdValue) && (x4 - x3 > thresholdValue)){
    //     result =  HandLeft_HandRight::HandRight;
    // }else if((x0 - x1 > thresholdValue) && (x1 - x2 > thresholdValue) && (
    //         x2 - x3 > thresholdValue) && (x3 - x4 > thresholdValue)){
    //     result = HandLeft_HandRight::HandLeft;
    // }else {
    //     result = HandLeft_HandRight::Other;
    // }

    if ((x1 - x0 > thresholdValue) && (x2 - x1 > thresholdValue) ){
        result =  HandLeft_HandRight::HandRight;
    }else if((x0 - x1 > thresholdValue) && (x1 - x2 > thresholdValue) ){
        result = HandLeft_HandRight::HandLeft;
    }else {
        result = HandLeft_HandRight::Other;
    }

    return result;
}




