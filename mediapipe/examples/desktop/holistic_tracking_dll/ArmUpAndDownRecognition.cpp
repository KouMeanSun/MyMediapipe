#include "ArmUpAndDownRecognition.h"
#include <iostream>

GoogleMediapipeDetect::ArmUpAndDownRecognition::ArmUpAndDownRecognition()
{

}

GoogleMediapipeDetect::ArmUpAndDownRecognition::~ArmUpAndDownRecognition()
{

}

bool GoogleMediapipeDetect::ArmUpAndDownRecognition::RecognizeProcess(const std::vector<Point2D>& pose_joint_points,
                                                                      int& left_arm_result,
                                                                      int& right_arm_result,
                                                                      int& left_arm_leftRight_result,
                                                                      int& right_arm_leftRight_result,
                                                                      int& left_arm_up_down_result_DIFF,
                                                                      int& right_arm_up_down_result_DIFF,
                                                                      int& left_arm_left_right_result_DIFF,
                                                                      int& right_arm_left_right_result_DIFF)
{
    if (pose_joint_points.size() != 33)
        return false;

    Point2D left_elbow = pose_joint_points[2];
    Point2D right_elbow = pose_joint_points[5];

    Point2D left_wrist = pose_joint_points[15];
    Point2D right_wrist = pose_joint_points[16];

    Point2D left_shoulder = pose_joint_points[11];
    Point2D right_shoulder = pose_joint_points[12];

    Point2D nose   = pose_joint_points[0];
//     std::cout << "ArmUpAndDownRecognition left_elbow x:" <<left_elbow.x<<",y:"<<left_elbow.y<< std::endl;
//     std::cout << "ArmUpAndDownRecognition left_wrist x:" <<left_wrist.x<<",y:"<<left_wrist.y<< std::endl;
    // ¼ì²â×óÊÖ
    if (left_wrist.y > left_shoulder.y)
    {
        left_arm_result = (int)ArmUpDown::ArmDown;
        left_arm_up_down_result_DIFF = left_wrist.y - left_shoulder.y;
    }
    else if (left_shoulder.y > left_wrist.y)
    {
        left_arm_result = (int)ArmUpDown::ArmUp;
        left_arm_up_down_result_DIFF = left_shoulder.y - left_wrist.y;
    }
    else
    {
        left_arm_result = (int)ArmUpDown::NoResult;
    }
    //std::cout << "ArmUpAndDownRecognition left_arm_result:" <<left_arm_result<< std::endl;
    if(left_wrist.x > nose.x){
        left_arm_leftRight_result = (int)ArmLeftRight::ArmLeft;
        left_arm_left_right_result_DIFF = left_wrist.x - left_shoulder.x;
    } else {
        left_arm_leftRight_result = (int)ArmLeftRight::ArmRight;
        left_arm_left_right_result_DIFF = left_shoulder.x - left_wrist.x ;
    }
    // else{
    //     left_arm_leftRight_result = (int)ArmLeftRight::NoLeftRightResult;
    // }
    //std::cout << "ArmUpAndDownRecognition left_arm_leftRight_result:" <<left_arm_leftRight_result<< std::endl;
    // ¼ì²âÓÒÊÖ
    if (right_wrist.y > right_shoulder.y)
    {
        right_arm_result = ArmUpDown::ArmDown;
        right_arm_up_down_result_DIFF = right_wrist.y - right_shoulder.y;
    }
    else if ( right_shoulder.y > right_wrist.y)
    {
        right_arm_result = ArmUpDown::ArmUp;
        right_arm_up_down_result_DIFF = right_shoulder.y - right_wrist.y;
    }
    else
    {
        right_arm_result = ArmUpDown::NoResult;
    }
    //std::cout << "ArmUpAndDownRecognition right_arm_result:" <<right_arm_result<< std::endl;
    if(right_wrist.x > nose.x){
        right_arm_leftRight_result = (int)ArmLeftRight::ArmLeft;
        right_arm_left_right_result_DIFF = right_wrist.x - left_shoulder.x;
    }else {
        right_arm_leftRight_result = (int)ArmLeftRight::ArmRight;
        right_arm_left_right_result_DIFF = left_shoulder.x - right_wrist.x;
    }
    // else {
    //     right_arm_leftRight_result = (int)ArmLeftRight::NoLeftRightResult;
    // }
    //std::cout << "ArmUpAndDownRecognition right_arm_leftRight_result:" <<right_arm_leftRight_result<< std::endl;
	return true;
}