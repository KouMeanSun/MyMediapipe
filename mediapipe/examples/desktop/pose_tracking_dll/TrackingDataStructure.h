//
// Created by GMY on 2022/4/1.
//

#ifndef MEDIAPIPE_MASTER_TRACKINGDATASTRUCTURE_H
#define MEDIAPIPE_MASTER_TRACKINGDATASTRUCTURE_H

struct Point2D
{
    float x;
    float y;
};

namespace GoogleMediapipeDetect{

    typedef Point2D Vector2D;

    enum Gesture
    {
        NoGesture = -1,
        One = 1,
        Two = 2,
        Three = 3,
        Four = 4,
        Five = 5,
        Six = 6,
        ThumbUp = 7,
        Ok = 8,
        Fist = 9,
        gun = 10,
        love = 11,
        fuck = 12,
        ringUp = 13
    };

    enum ArmUpDown
    {
        NoResult = -1,
        ArmUp = 1,
        ArmDown = 2
    };

    enum ArmLeftRight
    {
        NoLeftRightResult = -2,
        ArmLeft = 3,
        ArmRight = 4
    };

    enum ArmWaved
    {
        NoArmWavedResult = -3,
        ArmWavedUp  = 5,
        ArmWavedDown = 6,
        ArmWavedLeft = 7,
        ArmWavedRight = 8
    };
}
#endif //MEDIAPIPE_MASTER_TRACKINGDATASTRUCTURE_H
