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

    //1.���ж���������x����λ�ã��Ƿ��ڣ����Ҽ��֮��,������������ж�����
    if(left_wrist.x > right_shoulder.x && left_wrist.x < left_shoulder.x){
        if(left_wrist.y < left_hip.y){//�ж������β��Ϸ�
            if(left_wrist.y < mouth_left.y){// ������������棬���ж�Ϊ��
                left_hand_position_result = HandPosition::HandPosition_top;
            }else if(left_wrist.y > mouth_left.y) {//������������棬���ж�ΰ��
                left_hand_position_result = HandPosition::HandPosition_bottom;
            }
        }
    }

    if(left_wrist.x > left_shoulder.x && left_wrist.y < left_eye.y){//��������࣬�۾����Ϸ������ж�Ϊ��
        left_hand_position_result = HandPosition::HandPosition_left;
    }

    if(left_wrist.x < right_shoulder.x && left_wrist.y < left_eye.y){//���Ҽ���Ҳ࣬�۾����Ϸ������ж�Ϊ��
        left_hand_position_result = HandPosition::HandPosition_right;
    }


    //2.���ж�����
    if(right_wrist.x > right_shoulder.x && right_wrist.x < left_shoulder.x ){
        if(right_wrist.y < left_hip.y){//�ж������β��Ϸ�
            if(right_wrist.y < mouth_left.y){//������������棬���ж�Ϊ��
                right_hand_position_result = HandPosition::HandPosition_top;
            }else if(right_wrist.y > mouth_left.y){
                right_hand_position_result = HandPosition::HandPosition_bottom;
            }
        }
    }

    if(right_wrist.x > left_shoulder.x && right_wrist.y < left_eye.y){//��������࣬�۾����Ϸ������ж�Ϊ��
        right_hand_position_result = HandPosition::HandPosition_left;
    }

    if(right_wrist.x < right_shoulder.x && right_wrist.y < left_eye.y){//��������࣬�۾����Ϸ������ж�Ϊ��
        right_hand_position_result = HandPosition::HandPosition_right;
    }

    return true;
}