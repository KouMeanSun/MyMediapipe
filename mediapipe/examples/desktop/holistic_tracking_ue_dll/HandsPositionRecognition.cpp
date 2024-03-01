//
// Created by gaomingyang on 2022/11/25.
//

#include "HandsPositionRecognition.h"
#include <iostream>


GoogleMediapipeDetect::HandsPositionRecognition::HandsPositionRecognition()
{

}

GoogleMediapipeDetect::HandsPositionRecognition::~HandsPositionRecognition()
{

}


bool GoogleMediapipeDetect::HandsPositionRecognition::PositionRecognizeProcess(const std::vector<Point2D>& posePoints,
                              int& left_hand_position_result,
                              int& right_hand_position_result){
    Point2D left_shoulder = posePoints[11];
    Point2D right_shoulder = posePoints[12];

    Point2D left_eye = posePoints[2];
    Point2D right_eye = posePoints[5];

    Point2D mouth_left = posePoints[9];
    Point2D left_hip = posePoints[23];


    Point2D left_wrist = posePoints[15];
    Point2D right_wrist = posePoints[16];

    //1.先判断左手手掌x坐标位置，是否在，左，右肩膀之间,在这个区间内判断上下
    if(left_wrist.x > right_shoulder.x && left_wrist.x < left_shoulder.x){
        if(left_wrist.y < left_hip.y){//判断手在臀部上方
            if(left_wrist.y < mouth_left.y){// 左手在嘴巴上面，则判断为上
                left_hand_position_result = HandPosition::HandPosition_top;
            }else if(left_wrist.y > mouth_left.y) {//左手在嘴巴下面，则判断伟下
                left_hand_position_result = HandPosition::HandPosition_bottom;
            }
        }
    }

    if(left_wrist.x > left_shoulder.x && left_wrist.y < left_eye.y){//在左肩的左侧，眼睛的上方，则判断为左
        left_hand_position_result = HandPosition::HandPosition_left;
    }

    if(left_wrist.x < right_shoulder.x && left_wrist.y < left_eye.y){//在右肩的右侧，眼睛的上方，则判断为右
        left_hand_position_result = HandPosition::HandPosition_right;
    }


    //2.再判断右手
    if(right_wrist.x > right_shoulder.x && right_wrist.x < left_shoulder.x ){
        if(right_wrist.y < left_hip.y){//判断手在臀部上方
            if(right_wrist.y < mouth_left.y){//左手在嘴巴上面，则判断为上
                right_hand_position_result = HandPosition::HandPosition_top;
            }else if(right_wrist.y > mouth_left.y){
                right_hand_position_result = HandPosition::HandPosition_bottom;
            }
        }
    }

    if(right_wrist.x > left_shoulder.x && right_wrist.y < left_eye.y){//在左肩的左侧，眼睛的上方，则判断为左
        right_hand_position_result = HandPosition::HandPosition_left;
    }

    if(right_wrist.x < right_shoulder.x && right_wrist.y < left_eye.y){//在左肩的左侧，眼睛的上方，则判断为左
        right_hand_position_result = HandPosition::HandPosition_right;
    }

    return true;
}