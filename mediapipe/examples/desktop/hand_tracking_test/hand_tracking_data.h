#ifndef HAND_TRACKING_DATA_H
#define HAND_TRACKING_DATA_H

struct PoseInfo {
	float x;
	float y;
};

typedef PoseInfo Point2D;
typedef PoseInfo Vector2D;


struct GestureRecognitionResult
{
	int m_Gesture_Recognition_Result[2] = {-1,-1};
	int m_HandUp_HandDown_Detect_Result[2] = {-1,-1};
	int m_HandLeft_HandRight_Detect_Result[2] = {-2,-2};
};

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

enum HandUp_HandDown
{
	NoHand = -1,
	HandUp = 1,
	HandDown = 2
};

enum HandLeft_HandRight{
	Other = -2,
	HandLeft = 3,
	HandRight = 4
};

enum DetectFrame_Direct_Status{
	success = 1,
	not_Init = 2,
	run_status_not_ok = 3
};

#endif // !HAND_TRACKING_DATA_H
