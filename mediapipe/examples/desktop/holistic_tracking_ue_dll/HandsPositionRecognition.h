//
// Created by gaomingyang on 2022/11/25.
//

#ifndef MEDIAPIPE_MASTER_HANDSPOSITIONRECOGNITION_H
#define MEDIAPIPE_MASTER_HANDSPOSITIONRECOGNITION_H

#include <vector>
#include "TrackingDataStructure.h"

namespace GoogleMediapipeDetect {
    class HandsPositionRecognition {
    public:
        HandsPositionRecognition();
        virtual~HandsPositionRecognition();

    public:
        bool PositionRecognizeProcess(const std::vector<Point2D>& posePoints,
                                   int& left_hand_position_result,
                                   int& right_hand_position_result
                                   );
    };
}

#endif //MEDIAPIPE_MASTER_HANDSPOSITIONRECOGNITION_H
