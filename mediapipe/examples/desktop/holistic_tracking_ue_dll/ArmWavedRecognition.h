//
// Created by GMY on 2022/3/29.
//

#ifndef MEDIAPIPE_MASTER_ARMWAVEDRECOGNITION_H
#define MEDIAPIPE_MASTER_ARMWAVEDRECOGNITION_H
#include <vector>
#include "TrackingDataStructure.h"

namespace GoogleMediapipeDetect{
    class ArmWavedRecognition {
         public:
                ArmWavedRecognition();
                 virtual~ArmWavedRecognition();

         public:
            bool WavedRecognizeProcess(int* left_arm_updown_result_list,
                    int* right_arm_updown_result_list,
                    int* left_arm_leftright_result_list,
                    int* right_arm_leftright_result_list,
                    int& left_arm_waved_updown_result,
                    int& right_arm_waved_updown_result,
                    int& left_arm_waved_leftright_result,
                    int& right_arm_waved_leftright_result);
    };
}



#endif //MEDIAPIPE_MASTER_ARMWAVEDRECOGNITION_H
