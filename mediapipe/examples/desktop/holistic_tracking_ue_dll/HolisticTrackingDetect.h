#ifndef HOLISTIC_TRACKING_DETECT_H
#define HOLISTIC_TRACKING_DETECT_H

#include <cstdlib>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/opencv_highgui_inc.h"
#include "mediapipe/framework/port/opencv_imgproc_inc.h"
#include "mediapipe/framework/port/opencv_video_inc.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"

#include "mediapipe/framework/formats/detection.pb.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/formats/rect.pb.h"
#include "TrackingDataStructure.h"

namespace GoogleMediapipeDetect {

	class HolisticTrackingDetect
	{
	public:
		HolisticTrackingDetect();
		virtual~HolisticTrackingDetect();

	public:
		int InitModel(const char* model_path);
		int DetectImageDirect(int image_width, int image_height, void* image_data ,int* detect_result,Point2D* PoseLandmarks, Point2D* LeftHandLandmarks, Point2D* RightHandLandmarks, bool& isGestureOK);
		int DetectCamera(bool show_image = false);

        int BoxRegister(int hotBoxLeftTopX,int hotBoxLeftTopY,int hotBoxRightBottomX,int hotBoxRightBottomY ,int okGester);
        void DrawPoseLandmarksTest(int image_width, int image_height, void* image_data,Point2D* PoseLandmarks,const char* writeimageFullPath);

		int Release();

	private:
		absl::Status Mediapipe_InitGraph(const char* model_path);
		absl::Status Mediapipe_RunMPPGraph_Direct(int image_width, int image_height, void* image_data, int* detect_result,Point2D* PoseLandmarks, Point2D* LeftHandLandmarks, Point2D* RightHandLandmarks, bool& isGestureOK);
		absl::Status Mediapipe_RunMPPGraph_Camera(bool show_image = false);
		absl::Status Mediapipe_ReleaseGraph();

        int WriteLog(absl::Status msg);

	private:
		bool m_bIsInit;
		bool m_bIsRelease;

		mediapipe::CalculatorGraph m_Graph;

		const char* m_Video_InputStreamName;

		const char* m_Video_OutputStreamName;
		const char* m_PoseLandmarks_OutputStreamName;
		const char* m_LeftHandLandmarks_OutputStreamName;
		const char* m_RightHandLandmarks_OutputStreamName;
		const char* m_FaceLandmarks_OutputStreamName;

        int m_frameIndex;
        int* m_left_arm_result_list = new int[5];
        int* m_right_arm_result_list = new int[5];
        int* m_left_arm_leftRight_result_list = new int[5];
        int* m_right_arm_leftRight_result_list = new int[5];


        int m_left_arm_pre_result = (int)ArmUpDown::NoResult;
        int m_right_arm_pre_result = (int)ArmUpDown::NoResult;
        int m_left_arm_pre_leftRight_result = (int)ArmLeftRight::NoLeftRightResult;
        int m_right_arm_pre_leftRight_result = (int)ArmLeftRight::NoLeftRightResult;

        int m_left_arm_current_result = (int)ArmUpDown::NoResult;
        int m_right_arm_current_result = (int)ArmUpDown::NoResult;
        int m_left_arm_current_leftRight_result = (int)ArmLeftRight::NoLeftRightResult;
        int m_right_arm_current_leftRight_result = (int)ArmLeftRight::NoLeftRightResult;

        int m_OKGesture;

        int m_hotBoxLeftTopX;
        int m_hotBoxLeftTopY;

        int m_hotBoxRightBottomX;
        int m_hotBoxRightBottomY;

		std::unique_ptr<mediapipe::OutputStreamPoller> m_pVideoPoller;
		std::unique_ptr<mediapipe::OutputStreamPoller> m_pPoseLandmarksPoller;
		std::unique_ptr<mediapipe::OutputStreamPoller> m_pLeftHandLandmarksPoller;
		std::unique_ptr<mediapipe::OutputStreamPoller> m_pRightHandLandmarksPoller;
		std::unique_ptr<mediapipe::OutputStreamPoller> m_pFaceLandmarksPoller;
	};
}

#endif // !HOLISTIC_TRACKING_DETECT_H
