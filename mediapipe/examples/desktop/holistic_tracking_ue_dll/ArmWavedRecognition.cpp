//
// Created by GMY on 2022/3/29.
//

#include "ArmWavedRecognition.h"
#include <iostream>

GoogleMediapipeDetect::ArmWavedRecognition::ArmWavedRecognition()
{

}

GoogleMediapipeDetect::ArmWavedRecognition::~ArmWavedRecognition()
{

}

bool GoogleMediapipeDetect::ArmWavedRecognition::WavedRecognizeProcess(int *left_arm_updown_result_list,
                                                                       int *right_arm_updown_result_list,
                                                                       int *left_arm_leftright_result_list,
                                                                       int *right_arm_leftright_result_list,
                                                                       int &left_arm_waved_updown_result,
                                                                       int &right_arm_waved_updown_result,
                                                                       int &left_arm_waved_leftright_result,
                                                                       int &right_arm_waved_leftright_result) {

    //拿到差值
    int left_arm_up_down_result_DIFF = left_arm_updown_result_list[2];
    int right_arm_up_down_result_DIFF =  right_arm_updown_result_list[2];

    int left_arm_left_right_result_DIFF = left_arm_leftright_result_list[2];
    int right_arm_left_right_result_DIFF = right_arm_leftright_result_list[2];

    //先判断左臂上下挥动
    int left_arm_updown_first = left_arm_updown_result_list[0];
    int left_arm_updown_second = left_arm_updown_result_list[1];

//    if(left_arm_updown_first == ArmUpDown::ArmDown && left_arm_updown_second == ArmUpDown::ArmUp ){
//        left_arm_waved_updown_result = ArmWaved::ArmWavedUp;
//    }else if(left_arm_updown_first == ArmUpDown::ArmUp && left_arm_updown_second == ArmUpDown::ArmDown ){
//        left_arm_waved_updown_result = ArmWaved::ArmWavedDown;
//    }else {
//        left_arm_waved_updown_result = ArmWaved::NoArmWavedResult;
//    }
    if(left_arm_updown_first == (int)ArmUpDown::NoResult && left_arm_updown_second == (int)ArmUpDown::NoResult){
        left_arm_waved_updown_result = ArmWaved::NoArmWavedResult;
    }else if(left_arm_updown_first != left_arm_updown_second){
        if(left_arm_updown_first == ArmUpDown::ArmDown && left_arm_updown_second == ArmUpDown::ArmUp){
            left_arm_waved_updown_result = ArmWaved::ArmWavedUp;
        }else if(left_arm_updown_first == ArmUpDown::ArmUp && left_arm_updown_second == ArmUpDown::ArmDown){
            left_arm_waved_updown_result = ArmWaved::ArmWavedDown;
        }else {
            left_arm_waved_updown_result = ArmWaved::NoArmWavedResult;
        }
    }else {
        left_arm_waved_updown_result = ArmWaved::NoArmWavedResult;
    }

    //std::cout << "gmy left_arm_waved_updown_result:" << left_arm_waved_updown_result << std::endl;

    //再判断右臂上下挥动
    int right_arm_updown_first = right_arm_updown_result_list[0];
    int right_arm_updown_second = right_arm_updown_result_list[1];

    if(right_arm_updown_first == (int)ArmUpDown::NoResult && right_arm_updown_second == (int)ArmUpDown::NoResult){
        right_arm_waved_updown_result = ArmWaved::NoArmWavedResult;
    }else if(right_arm_updown_first != right_arm_updown_second){
        if(right_arm_updown_first == ArmUpDown::ArmDown && right_arm_updown_second == ArmUpDown::ArmUp){
            right_arm_waved_updown_result = ArmWaved::ArmWavedUp;
        }else if(right_arm_updown_first == ArmUpDown::ArmUp && right_arm_updown_second == ArmUpDown::ArmDown){
            right_arm_waved_updown_result = ArmWaved::ArmWavedDown;
        }else {
            right_arm_waved_updown_result = ArmWaved::NoArmWavedResult;
        }
    }else {
        right_arm_waved_updown_result = ArmWaved::NoArmWavedResult;
    }

    //判断左臂左右挥动
    //如果左臂是 ArmUpDown::Down 状态 才检测左右挥动
    if((left_arm_updown_first == ArmUpDown::ArmUp) && (left_arm_updown_second == ArmUpDown::ArmUp) && (left_arm_waved_updown_result == ArmWaved::NoArmWavedResult)){
        int left_arm_leftright_first = left_arm_leftright_result_list[0];
        int left_arm_leftright_second = left_arm_leftright_result_list[1];
       // std::cout << "gmy left_arm_waved_leftright_result 1111111111111" << std::endl;
        if(left_arm_leftright_first == ArmLeftRight::NoLeftRightResult && left_arm_leftright_second == ArmLeftRight::NoLeftRightResult){
            left_arm_waved_leftright_result =  ArmWaved::NoArmWavedResult;
           // std::cout << "gmy left_arm_waved_leftright_result 22222222222" << std::endl;
        }else if(left_arm_leftright_first != left_arm_leftright_second){
            //std::cout << "gmy left_arm_waved_leftright_result left_arm_leftright_first != left_arm_leftright_second" << std::endl;
            if(left_arm_leftright_first == ArmLeftRight::ArmLeft && left_arm_leftright_second == ArmLeftRight::ArmRight){
                //std::cout << "gmy left_arm_waved_leftright_result 333333333333333" << std::endl;
                left_arm_waved_leftright_result = ArmWaved::ArmWavedRight;
                left_arm_waved_updown_result = ArmWaved::NoArmWavedResult;
            }else if(left_arm_leftright_first == ArmLeftRight::ArmRight &&  left_arm_leftright_second == ArmLeftRight::ArmLeft){
                //std::cout << "gmy left_arm_waved_leftright_result 444444444444444" << std::endl;
                left_arm_waved_leftright_result = ArmWaved::ArmWavedLeft;
                left_arm_waved_updown_result = ArmWaved::NoArmWavedResult;
            }else {
                left_arm_waved_leftright_result =  ArmWaved::NoArmWavedResult;
               // std::cout << "gmy left_arm_waved_leftright_result 555555555555555" << std::endl;
            }
        }else {
            left_arm_waved_leftright_result =  ArmWaved::NoArmWavedResult;
           // std::cout << "gmy left_arm_waved_leftright_result 66666666666666666" << std::endl;
        }
    }else {
        left_arm_waved_leftright_result =  ArmWaved::NoArmWavedResult;
       // std::cout << "gmy left_arm_waved_leftright_result 77777777777777777" << std::endl;
    }

    //判断右臂左右挥动
    //如果右臂是 ArmUpDown::Down 状态 才检测左右挥动
    if( (right_arm_updown_first == ArmUpDown::ArmUp) && (right_arm_updown_second == ArmUpDown::ArmUp) && (right_arm_waved_updown_result = ArmWaved::NoArmWavedResult)){
        int right_arm_leftright_firt = right_arm_leftright_result_list[0];
        int right_arm_leftright_second = right_arm_leftright_result_list[1];

        if(right_arm_leftright_firt == ArmLeftRight::NoLeftRightResult && right_arm_leftright_second == ArmLeftRight::NoLeftRightResult){
            right_arm_waved_leftright_result =  ArmWaved::NoArmWavedResult;
        }else if(right_arm_leftright_firt != right_arm_leftright_second){
            if(right_arm_leftright_firt == ArmLeftRight::ArmLeft  && right_arm_leftright_second == ArmLeftRight::ArmRight){
                right_arm_waved_leftright_result = ArmWaved::ArmWavedRight;
                right_arm_waved_updown_result = ArmWaved::NoArmWavedResult;
            }else if(right_arm_leftright_firt == ArmLeftRight::ArmRight  && right_arm_leftright_second == ArmLeftRight::ArmLeft){
                right_arm_waved_leftright_result = ArmWaved::ArmWavedLeft;
                right_arm_waved_updown_result = ArmWaved::NoArmWavedResult;
            }else {
                right_arm_waved_leftright_result =  ArmWaved::NoArmWavedResult;
            }
        }else {
            right_arm_waved_leftright_result =  ArmWaved::NoArmWavedResult;
        }
    }else {
        right_arm_waved_leftright_result =  ArmWaved::NoArmWavedResult;
    }

    //最后通过差值，同一个手臂只取上下或左右挥动一个结果。
//    if((left_arm_waved_updown_result != ArmWaved::NoArmWavedResult) && (left_arm_waved_leftright_result =  ArmWaved::NoArmWavedResult)){
//        if(left_arm_up_down_result_DIFF >= left_arm_left_right_result_DIFF){
//            //只取左臂上下挥动的结果，清除左臂左右挥动的结果
//            left_arm_waved_leftright_result =  ArmWaved::NoArmWavedResult;
//            //std::cout<<"1111111111111111"<<std::endl;
//        }else{
//            left_arm_waved_updown_result = ArmWaved::NoArmWavedResult;
//            //std::cout<<"222222222222222"<<std::endl;
//        }
//    }
//
//    if((right_arm_waved_updown_result != ArmWaved::NoArmWavedResult) && (right_arm_waved_leftright_result !=  ArmWaved::NoArmWavedResult)){
//        if(right_arm_up_down_result_DIFF >= right_arm_left_right_result_DIFF){
//            //只取右臂上下挥动的结果，清除右臂左右挥动的结果
//            right_arm_waved_leftright_result =  ArmWaved::NoArmWavedResult;
//            //std::cout<<"333333333333"<<std::endl;
//        }else {
//            right_arm_waved_updown_result = ArmWaved::NoArmWavedResult;
//            //std::cout<<"4444444444444444"<<std::endl;
//        }
//    }

    return true;
}