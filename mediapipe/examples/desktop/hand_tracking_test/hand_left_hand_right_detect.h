//
// Created by GMY on 2022/2/17.
//

#ifndef MEDIAPIPE_MASTER_HAND_LEFT_HAND_RIGHTDETECT_H
#define MEDIAPIPE_MASTER_HAND_LEFT_HAND_RIGHTDETECT_H

#include "hand_tracking_data.h"
#include <vector>

namespace GoogleMediapipeHandTrackingDetect {
    class HandLeftHandRightDetect
    {
    public:
        HandLeftHandRightDetect();
        virtual~HandLeftHandRightDetect();

    public:
        int DetectHandLeftOrHandRight(int point_list[][2],int origin_image_height,int leftRightthresholdValue);
    };
}


#endif //MEDIAPIPE_MASTER_HAND_LEFT_HAND_RIGHTDETECT_H
