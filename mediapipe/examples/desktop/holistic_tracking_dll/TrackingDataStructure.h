#ifndef TRACKING_DATA_STRUCTURE_H
#define TRACKING_DATA_STRUCTURE_H

struct Point2D
{
    float x;
    float y;
};

namespace GoogleMediapipeDetect {

//	struct Point2D
//	{
//		float x;
//		float y;
//	};

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

    enum  HandPosition{
        HandPosition_No = -4,
        HandPosition_left = 9,
        HandPosition_right = 10,
        HandPosition_top = 11,
        HandPosition_bottom = 12
    };

    enum  PoseDisplay{
        PoseDisplay_No = -5,
        PoseDisplay_Love = 13
    };
}

#endif // !TRACKING_DATA_STRUCTURE_H
