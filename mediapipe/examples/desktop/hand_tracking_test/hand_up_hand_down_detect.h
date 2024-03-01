#ifndef HAND_UP_HAND_DOWN_DETECT_H
#define HAND_UP_HAND_DOWN_DETECT_H

#include "hand_tracking_data.h"

#include <vector>

namespace GoogleMediapipeHandTrackingDetect {
	class HandUpHandDownDetect
	{
	public:
		HandUpHandDownDetect();
		virtual~HandUpHandDownDetect();

	public:
		int DetectHandUpOrHandDown(int point_list[][2],int origin_image_height,int upDownthresholdValue);
	};
}


#endif // !HAND_UP_HAND_DOWN_DETECT_H
