//
// Created by gaomingyang on 2022/11/30.
//

#ifndef MEDIAPIPE_MASTER_LOVEPOSERECOGNITION_H
#define MEDIAPIPE_MASTER_LOVEPOSERECOGNITION_H
#include <vector>
#include "TrackingDataStructure.h"

namespace GoogleMediapipeDetect {
    class LovePoseRecognition {
    public:
        LovePoseRecognition();
        virtual~LovePoseRecognition();

    public:
        bool PoseRecognizeProcess(const std::vector<Point2D>& posePoints,
                                      int& );
    };

}
#endif //MEDIAPIPE_MASTER_LOVEPOSERECOGNITION_H
