//
// Created by gaomingyang on 2022/11/30.
//

#include "LovePoseRecognition.h"


GoogleMediapipeDetect::LovePoseRecognition::LovePoseRecognition()
{

}

GoogleMediapipeDetect::LovePoseRecognition::~LovePoseRecognition()
{

}
bool GoogleMediapipeDetect::LovePoseRecognition::PoseRecognizeProcess(const std::vector<Point2D>& posePoints,
                                                                               int& pose_result) {
    Point2D left_eye_inner = posePoints[1];

    Point2D left_shoulder = posePoints[11];
    Point2D right_shoulder = posePoints[12];

    Point2D left_elbow = posePoints[13];
    Point2D right_elbow = posePoints[14];

    Point2D left_wrist = posePoints[15];
    Point2D right_wrist = posePoints[16];

    if(left_elbow.y < left_shoulder.y && right_elbow.y < right_shoulder.y && left_elbow.x > left_shoulder.x && right_elbow.x < right_shoulder.x && left_wrist.x < left_shoulder.x && right_wrist.x > right_shoulder.x && left_wrist.y < left_eye_inner.y && right_wrist.y < left_eye_inner.y){
        pose_result = PoseDisplay ::PoseDisplay_Love;
    }else {
        pose_result = PoseDisplay ::PoseDisplay_No;
    }

    return true;
}