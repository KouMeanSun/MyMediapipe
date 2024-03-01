#include <vector>

#include "hand_tracking_detect.h"
#include "hand_gesture_recognition.h"
#include "hand_up_hand_down_detect.h"
#include "hand_left_hand_right_detect.h"

GoogleMediapipeHandTrackingDetect::HandTrackingDetect::HandTrackingDetect()
{
	m_bIsInit = false;
	m_bIsRelease = false;
	m_kInputStream = "input_video";
	m_kOutputStream = "output_video";
	m_kWindowName = "MediaPipe";
	m_kOutputLandmarks = "landmarks";
	m_LandmarksCallBackFunc = nullptr;
	m_GestureResultCallBackFunc = nullptr;

    m_frameIndex = 0;

    m_OKGesture = -1;

}

GoogleMediapipeHandTrackingDetect::HandTrackingDetect::~HandTrackingDetect()
{
	if (!m_bIsRelease)
	{
		Release();
	}
}

int GoogleMediapipeHandTrackingDetect::HandTrackingDetect::InitGraph(const char* model_path)
{
	absl::Status run_status = Mediapipe_InitGraph(model_path);
	if (!run_status.ok()) 
	{
		return 0;
	}
	m_bIsInit = true;
	return  1;
}

int GoogleMediapipeHandTrackingDetect::HandTrackingDetect::BoxRegister(int hotBoxLeftTopX,int hotBoxLeftTopY,int hotBoxRightBottomX,int hotBoxRightBottomY ,int okGester){
	m_OKGesture = okGester;
	m_hotBoxLeftTopX = hotBoxLeftTopX;
	m_hotBoxLeftTopY = hotBoxLeftTopY;
	m_hotBoxRightBottomX = hotBoxRightBottomX;
	m_hotBoxRightBottomY = hotBoxRightBottomY;

	return 1;
}

int GoogleMediapipeHandTrackingDetect::HandTrackingDetect::RegisterLandmarksCallback(LandmarksCallBack func)
{
	if (func != nullptr)
	{
		m_LandmarksCallBackFunc = func;
		return 1;
	}

	return 0;
}

int GoogleMediapipeHandTrackingDetect::HandTrackingDetect::RegisterGestureResultCallBack(GestureResultCallBack func)
{
	if (func != nullptr)
	{
		m_GestureResultCallBackFunc = func;
		return 1;
	}

	return 0;
}


int GoogleMediapipeHandTrackingDetect::HandTrackingDetect::DetectFrame(int image_index, int image_width, int image_height, void* image_data)
{
	if (!m_bIsInit)
		return 0;

	absl::Status run_status = Mediapipe_RunMPPGraph(image_index,image_width,image_height,image_data);
	if (!run_status.ok()) {
		return 0;
	}
	return 1;
}

int GoogleMediapipeHandTrackingDetect::HandTrackingDetect::DetectFrame_Direct(int image_width, int image_height, void* image_data, GestureRecognitionResult& gesture_result,int isHorizonFlip,int upDownthresholdValue,int leftRightthresholdValue, PoseInfo* firstInfos,PoseInfo* secondInfos,bool& isGestureOK)
{
	if (!m_bIsInit)
		return DetectFrame_Direct_Status::not_Init;

	absl::Status run_status = Mediapipe_RunMPPGraph_Direct(image_width, image_height, image_data, gesture_result,isHorizonFlip,upDownthresholdValue, leftRightthresholdValue,firstInfos,secondInfos, isGestureOK);
	if (!run_status.ok()) {
		return DetectFrame_Direct_Status::run_status_not_ok;
	}
	return DetectFrame_Direct_Status::success;
}

int GoogleMediapipeHandTrackingDetect::HandTrackingDetect::DetectVideo(const char* video_path, int show_image)
{
	if (!m_bIsInit)
		return 0;

	absl::Status run_status = Mediapipe_RunMPPGraph(video_path, show_image);
	if (!run_status.ok()) {
		return 0;
	}
	return 1;
}


int GoogleMediapipeHandTrackingDetect::HandTrackingDetect::Release()
{
	absl::Status run_status = Mediapipe_ReleaseGraph();
	if (!run_status.ok()) {
		return 0;
	}
	m_bIsRelease = true;
	return 1;
}


absl::Status GoogleMediapipeHandTrackingDetect::HandTrackingDetect::Mediapipe_InitGraph(const char* model_path)
{
	std::string calculator_graph_config_contents;
	MP_RETURN_IF_ERROR(mediapipe::file::GetContents(model_path, &calculator_graph_config_contents));

	mediapipe::CalculatorGraphConfig config =
		mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
			calculator_graph_config_contents);

	MP_RETURN_IF_ERROR(m_Graph.Initialize(config));

	// 添加video输出流
	auto sop = m_Graph.AddOutputStreamPoller(m_kOutputStream);
	assert(sop.ok());
	m_pPoller = std::make_unique<mediapipe::OutputStreamPoller>(std::move(sop.value()));

	// 添加landmarks输出流
	mediapipe::StatusOrPoller sop_landmark = m_Graph.AddOutputStreamPoller(m_kOutputLandmarks);
	assert(sop_landmark.ok());
	m_pPoller_landmarks = std::make_unique<mediapipe::OutputStreamPoller>(std::move(sop_landmark.value()));

	MP_RETURN_IF_ERROR(m_Graph.StartRun({}));

	return absl::OkStatus();
}

absl::Status GoogleMediapipeHandTrackingDetect::HandTrackingDetect::Mediapipe_RunMPPGraph(int image_index, int image_width, int image_height, void* image_data)
{
	// 构造cv::Mat对象
	cv::Mat camera_frame(cv::Size(image_width, image_height), CV_8UC3,(uchar*)image_data);
	cv::cvtColor(camera_frame, camera_frame, cv::COLOR_BGR2RGB);
	cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*/ 1);
	//std::cout << "图片构建完成" << std::endl;

	// Wrap Mat into an ImageFrame.
	auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
		mediapipe::ImageFormat::SRGB, camera_frame.cols, camera_frame.rows,
		mediapipe::ImageFrame::kDefaultAlignmentBoundary);
	cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
	camera_frame.copyTo(input_frame_mat);
	//std::cout << "Wrap Mat into an ImageFrame." << std::endl;

	// Send image packet into the graph.
	size_t frame_timestamp_us =
		(double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;

	MP_RETURN_IF_ERROR(m_Graph.AddPacketToInputStream(
		m_kInputStream, mediapipe::Adopt(input_frame.release())
		.At(mediapipe::Timestamp(frame_timestamp_us))));
	//std::cout << "Send image packet into the graph." << std::endl;


	// Get the graph result packet, or stop if that fails.
	mediapipe::Packet packet;
	mediapipe::Packet packet_landmarks;
	if (!m_pPoller->Next(&packet)) 
		return absl::OkStatus();

	if (m_pPoller_landmarks->QueueSize() > 0) 
	{
		if (m_pPoller_landmarks->Next(&packet_landmarks))
		{

			std::vector<mediapipe::NormalizedLandmarkList> output_landmarks = packet_landmarks.Get<std::vector<mediapipe::NormalizedLandmarkList>>();

			int* hand_gesture_recognition_result = new int[output_landmarks.size()];
			std::vector<PoseInfo> hand_landmarks;
			hand_landmarks.clear();


			for (int m = 0; m < output_landmarks.size(); ++m)
			{
				mediapipe::NormalizedLandmarkList single_hand_NormalizedLandmarkList = output_landmarks[m];

				std::vector<PoseInfo> singleHandGestureInfo;
				singleHandGestureInfo.clear();

				for (int i = 0; i < single_hand_NormalizedLandmarkList.landmark_size(); ++i)
				{
					PoseInfo info;
					const mediapipe::NormalizedLandmark landmark = single_hand_NormalizedLandmarkList.landmark(i);
					info.x = landmark.x() * camera_frame.cols;
					info.y = landmark.y() * camera_frame.rows;
					singleHandGestureInfo.push_back(info);
					hand_landmarks.push_back(info);
				}

				HandGestureRecognition handGestureRecognition;
				int result = handGestureRecognition.GestureRecognition(singleHandGestureInfo);
//                std::cout << "gmy CLion GetGestureResult result = " << result << std::endl;
				hand_gesture_recognition_result[m] = result;

			}



			// 回调坐标点
			if (m_GestureResultCallBackFunc)
			{
				PoseInfo* hand_landmarks_pose_infos = new PoseInfo[hand_landmarks.size()];
				for (int i = 0; i < hand_landmarks.size(); ++i)
				{
					hand_landmarks_pose_infos[i].x = hand_landmarks[i].x;
					hand_landmarks_pose_infos[i].y = hand_landmarks[i].y;
				}

				if (m_LandmarksCallBackFunc)
				{
					m_LandmarksCallBackFunc(image_index, hand_landmarks_pose_infos, hand_landmarks.size());
				}

				delete[] hand_landmarks_pose_infos;
			}


			// 回调识别结果
			if (m_GestureResultCallBackFunc)
			{
				m_GestureResultCallBackFunc(image_index, hand_gesture_recognition_result, output_landmarks.size());
			}

			delete[] hand_gesture_recognition_result;

		}
	}
//    std::cout << "gmy CLion GetGestureResult result = " << result << std::endl;
	return absl::OkStatus();
}


absl::Status GoogleMediapipeHandTrackingDetect::HandTrackingDetect::Mediapipe_RunMPPGraph_Direct(int image_width, int image_height, void* image_data, GestureRecognitionResult& gesture_result,int isHorizonFlip,int upDownthresholdValue,int leftRightthresholdValue,PoseInfo* firstInfos,PoseInfo* secondInfos, bool& isGestureOK)
{
    m_frameIndex++;
	// 构造cv::Mat对象
	cv::Mat camera_frame(cv::Size(image_width, image_height), CV_8UC4, (uchar*)image_data);
	cv::cvtColor(camera_frame, camera_frame, cv::COLOR_RGBA2RGB);
	//cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*/ isHorizonFlip);
//	std::cout << "gmy 图片构建完成" << std::endl;

	// Wrap Mat into an ImageFrame.
	auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
		mediapipe::ImageFormat::SRGB, camera_frame.cols, camera_frame.rows,
		mediapipe::ImageFrame::kDefaultAlignmentBoundary);
	cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
	camera_frame.copyTo(input_frame_mat);
//	std::cout << "gmy Wrap Mat into an ImageFrame." << std::endl;
	// Send image packet into the graph.
	size_t frame_timestamp_us =
		(double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;

	MP_RETURN_IF_ERROR(m_Graph.AddPacketToInputStream(
		m_kInputStream, mediapipe::Adopt(input_frame.release())
		.At(mediapipe::Timestamp(frame_timestamp_us))));
	//std::cout << "gmy Send image packet into the graph." << std::endl;
	// Get the graph result packet, or stop if that fails.
	mediapipe::Packet packet;
	mediapipe::Packet packet_landmarks;
	if (!m_pPoller->Next(&packet))
		return absl::OkStatus();

	if (m_pPoller_landmarks->QueueSize() > 0)
	{
		if (m_pPoller_landmarks->Next(&packet_landmarks))
		{

			std::vector<mediapipe::NormalizedLandmarkList> output_landmarks = packet_landmarks.Get<std::vector<mediapipe::NormalizedLandmarkList>>();

			GestureRecognitionResult tempGestureResult;

			for (int m = 0; m < output_landmarks.size(); ++m)
			{
//			    std::cout<<"gaomingyang m="<<m<<std::endl;
				mediapipe::NormalizedLandmarkList single_hand_NormalizedLandmarkList = output_landmarks[m];

				std::vector<PoseInfo> singleHandGestureInfo;
				singleHandGestureInfo.clear();

				for (int i = 0; i < single_hand_NormalizedLandmarkList.landmark_size(); ++i)
				{
					PoseInfo info;
					const mediapipe::NormalizedLandmark landmark = single_hand_NormalizedLandmarkList.landmark(i);
					info.x = landmark.x() * camera_frame.cols;
					info.y = landmark.y() * camera_frame.rows;
//					std::cout<<"gaomingyang landmark.x() :"<<landmark.x()<<",landmark.y() :"<<landmark.y()<<std::endl;
                    singleHandGestureInfo.push_back(info);
                    if(m==0){
                        firstInfos[i] = info;
                    }else if(m==1){
                        secondInfos[i] = info;
                    }
                    if (i == 0){
                        int pointIndex = m_frameIndex%3;
                        m_points_list[pointIndex][0] = (int)info.x;
                        m_points_list[pointIndex][1] = (int)info.y;
                    }
				}

				// 检测手势
				HandGestureRecognition handGestureRecognition;
				int gesture_recognition_result = handGestureRecognition.GestureRecognition(singleHandGestureInfo);
				tempGestureResult.m_Gesture_Recognition_Result[m] = gesture_recognition_result;

				//如果 检测到的手势是 Five，则继续 检测上划和下划
				if(gesture_recognition_result == Gesture::Five || gesture_recognition_result == Gesture::Four){
                    HandUpHandDownDetect handupHandDownDetect;
                    HandLeftHandRightDetect handLeftHandRightDetect;
                    //检测上下滑动手势
                    int handuphanddown_result = handupHandDownDetect.DetectHandUpOrHandDown(m_points_list, image_height,upDownthresholdValue);

                    tempGestureResult.m_HandUp_HandDown_Detect_Result[m] = handuphanddown_result;
                    //检测左右滑动手势
                    int handLeftRight_result = handLeftHandRightDetect.DetectHandLeftOrHandRight(m_points_list,image_height,leftRightthresholdValue);
                    tempGestureResult.m_HandLeft_HandRight_Detect_Result[m] = handLeftRight_result;
				}else{
                    tempGestureResult.m_HandUp_HandDown_Detect_Result[m] = Gesture::NoGesture;
				}
				if(m == 0){
					if(gesture_recognition_result == m_OKGesture){
						int x = firstInfos[0].x;
						int y = firstInfos[0].y;
						if( (x > m_hotBoxLeftTopX) && (x < m_hotBoxRightBottomY) && (y > m_hotBoxLeftTopY) && (y < m_hotBoxRightBottomY)){
							 isGestureOK =  true;
						}else{
							isGestureOK =  false;
						}
					}else{
						isGestureOK =  false;
					}
				}else{
					isGestureOK =  false;
				}
			}
			gesture_result = tempGestureResult;
		}
	}
	cv::cvtColor(camera_frame, camera_frame, cv::COLOR_RGB2RGBA);
	return absl::OkStatus();
}



absl::Status GoogleMediapipeHandTrackingDetect::HandTrackingDetect::Mediapipe_RunMPPGraph(const char* video_path, int show_image)
{
	cv::VideoCapture capture;
	capture.open(video_path);
	
	RET_CHECK(capture.isOpened());
	
	std::cout << "capture.isOpened()" << std::endl;

	if (show_image) {
		cv::namedWindow(m_kWindowName, /*flags=WINDOW_AUTOSIZE*/ 1);
	}
#if (CV_MAJOR_VERSION >= 3) && (CV_MINOR_VERSION >= 2)
	capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	capture.set(cv::CAP_PROP_FPS, 30);
#endif

	bool grab_frames = true;
	int image_index = 0;
	while (grab_frames) {
		// Capture opencv camera or video frame.
		cv::Mat camera_frame_raw;
		capture >> camera_frame_raw;
		if (camera_frame_raw.empty())
			break;

		cv::Mat camera_frame;
		cv::cvtColor(camera_frame_raw, camera_frame, cv::COLOR_BGR2RGB);
		cv::flip(camera_frame, camera_frame, /*flipcode=HORIZONTAL*/ 1);

		// Wrap Mat into an ImageFrame.
		auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
			mediapipe::ImageFormat::SRGB, camera_frame.cols, camera_frame.rows,
			mediapipe::ImageFrame::kDefaultAlignmentBoundary);
		cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
		camera_frame.copyTo(input_frame_mat);

		// Send image packet into the graph.
		size_t frame_timestamp_us =
			(double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;

		MP_RETURN_IF_ERROR(m_Graph.AddPacketToInputStream(
			m_kInputStream, mediapipe::Adopt(input_frame.release())
			.At(mediapipe::Timestamp(frame_timestamp_us))));

		// Get the graph result packet, or stop if that fails.
		mediapipe::Packet packet;
		mediapipe::Packet packet_landmarks;
		if (!m_pPoller->Next(&packet)) break;
		if (m_pPoller_landmarks->QueueSize() > 0) {
			if (m_pPoller_landmarks->Next(&packet_landmarks))
			{

				std::vector<mediapipe::NormalizedLandmarkList> output_landmarks = packet_landmarks.Get<std::vector<mediapipe::NormalizedLandmarkList>>();

				int* hand_gesture_recognition_result = new int[output_landmarks.size()];
				std::vector<PoseInfo> hand_landmarks;
				hand_landmarks.clear();


				for (int m = 0; m < output_landmarks.size(); ++m)
				{
					mediapipe::NormalizedLandmarkList single_hand_NormalizedLandmarkList = output_landmarks[m];

					std::vector<PoseInfo> singleHandGestureInfo;
					singleHandGestureInfo.clear();

					for (int i = 0; i < single_hand_NormalizedLandmarkList.landmark_size(); ++i)
					{
						PoseInfo info;
						const mediapipe::NormalizedLandmark landmark = single_hand_NormalizedLandmarkList.landmark(i);
						info.x = landmark.x() * camera_frame.cols;
						info.y = landmark.y() * camera_frame.rows;
						singleHandGestureInfo.push_back(info);
						hand_landmarks.push_back(info);
					}

					HandGestureRecognition handGestureRecognition;
					int result = handGestureRecognition.GestureRecognition(singleHandGestureInfo);

					hand_gesture_recognition_result[m] = result;

				}

				// 回调坐标点
				if (m_GestureResultCallBackFunc)
				{
					PoseInfo* hand_landmarks_pose_infos = new PoseInfo[hand_landmarks.size()];
					for (int i = 0; i < hand_landmarks.size(); ++i)
					{
						hand_landmarks_pose_infos[i].x = hand_landmarks[i].x;
						hand_landmarks_pose_infos[i].y = hand_landmarks[i].y;
					}

					if (m_LandmarksCallBackFunc)
					{
						m_LandmarksCallBackFunc(image_index, hand_landmarks_pose_infos, hand_landmarks.size());
					}

					delete[] hand_landmarks_pose_infos;
				}


				// 回调识别结果
				if (m_GestureResultCallBackFunc)
				{
					m_GestureResultCallBackFunc(image_index, hand_gesture_recognition_result, output_landmarks.size());
				}

				delete[] hand_gesture_recognition_result;

			}
		}
		if (show_image) 
		{
			auto& output_frame = packet.Get<mediapipe::ImageFrame>();
			// Convert back to opencv for display or saving.
			cv::Mat output_frame_mat = mediapipe::formats::MatView(&output_frame);
			cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGB2BGR);
			cv::Mat dst;
			//cv::resize(output_frame_mat, dst, cv::Size(output_frame_mat.cols / 2, output_frame_mat.rows / 2));
			cv::imshow(m_kWindowName, dst);
			cv::waitKey(1);
		}

		image_index += 1;
	}
	if (show_image)
		capture.release();
		cv::destroyWindow(m_kWindowName);
	return absl::OkStatus();
}


absl::Status GoogleMediapipeHandTrackingDetect::HandTrackingDetect::Mediapipe_ReleaseGraph()
{
	MP_RETURN_IF_ERROR(m_Graph.CloseInputStream(m_kInputStream));
	return m_Graph.WaitUntilDone();
}
