#ifndef ARM_UP_AND_DOWN_RECOGNITION_H
#define ARM_UP_AND_DOWN_RECOGNITION_H

#include <vector>

#include "TrackingDataStructure.h"

namespace GoogleMediapipeDetect {
	class ArmUpAndDownRecognition 
	{
	public:
		ArmUpAndDownRecognition();
		virtual~ArmUpAndDownRecognition();

	public:
		bool RecognizeProcess(const std::vector<Point2D>& pose_joint_points,
		        int& left_arm_result,
		        int& right_arm_result,
		        int& left_arm_leftRight_result,
		        int& right_arm_leftRight_result,
                              int& left_arm_up_down_result_DIFF,
                              int& right_arm_up_down_result_DIFF,
                              int& left_arm_left_right_result_DIFF,
                              int& right_arm_left_right_result_DIFF);
	};
}

#endif // !ARM_UP_AND_DOWN_RECOGNITION_H
