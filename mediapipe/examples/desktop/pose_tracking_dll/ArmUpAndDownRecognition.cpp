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

    int upDownThrealdValue = 1;


	Point2D left_elbow = pose_joint_points[2];
	Point2D right_elbow = pose_joint_points[5];

	Point2D left_wrist = pose_joint_points[15];
	Point2D right_wrist = pose_joint_points[16];

	Point2D left_shoulder = pose_joint_points[11];
	Point2D right_shoulder = pose_joint_points[12];
    // std::cout << "ArmUpAndDownRecognition left_elbow x:" <<left_elbow.x<<",y:"<<left_elbow.y<< std::endl;
    // std::cout << "ArmUpAndDownRecognition left_wrist x:" <<left_wrist.x<<",y:"<<left_wrist.y<< std::endl;
	// ¼ì²â×óÊÖ
	if ((left_wrist.y - left_elbow.y) > upDownThrealdValue)
	{
		left_arm_result = (int)ArmUpDown::ArmDown;
        left_arm_up_down_result_DIFF = left_wrist.y - left_elbow.y;
	}
	else if ((left_elbow.y - left_wrist.y) > upDownThrealdValue)
	{
		left_arm_result = (int)ArmUpDown::ArmUp;
        left_arm_up_down_result_DIFF = left_elbow.y - left_wrist.y;
	}
	else
	{
		left_arm_result = (int)ArmUpDown::NoResult;
	}

	if(left_wrist.x > left_shoulder.x){
        left_arm_leftRight_result = (int)ArmLeftRight::ArmLeft;
        left_arm_left_right_result_DIFF = left_wrist.x - left_shoulder.x;
	} else if(left_wrist.x < right_shoulder.x){
        left_arm_leftRight_result = (int)ArmLeftRight::ArmRight;
        left_arm_left_right_result_DIFF = left_shoulder.x - left_wrist.x ;
	}else{
        left_arm_leftRight_result = (int)ArmLeftRight::NoLeftRightResult;
	}

	// ¼ì²âÓÒÊÖ
	if ((right_wrist.y - right_elbow.y) > upDownThrealdValue)
	{
		right_arm_result = ArmUpDown::ArmDown;
        right_arm_up_down_result_DIFF = right_wrist.y - right_elbow.y;
	}
	else if (( right_elbow.y - right_wrist.y) > upDownThrealdValue )
	{
		right_arm_result = ArmUpDown::ArmUp;
        right_arm_up_down_result_DIFF = right_elbow.y - right_wrist.y;
	}
	else
	{
		right_arm_result = ArmUpDown::NoResult;
	}

	if(right_wrist.x > left_shoulder.x){
	    right_arm_leftRight_result = (int)ArmLeftRight::ArmLeft;
        right_arm_left_right_result_DIFF = right_wrist.x - left_shoulder.x;
	}else if(right_wrist.x < right_shoulder.x ){
        right_arm_leftRight_result = (int)ArmLeftRight::ArmRight;
        right_arm_left_right_result_DIFF = left_shoulder.x - right_wrist.x;
	}else {
        right_arm_leftRight_result = (int)ArmLeftRight::NoLeftRightResult;
	}

	return true;
}