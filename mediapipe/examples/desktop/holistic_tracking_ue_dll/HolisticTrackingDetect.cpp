#include <vector>
#include <iostream>
#include <fstream>

#include "HolisticTrackingDetect.h"
#include "GestureRecognition.h"
#include "ArmUpAndDownRecognition.h"
#include "ArmWavedRecognition.h"
#include "HandsPositionRecognition.h"
#include "LovePoseRecognition.h"

using namespace std;

GoogleMediapipeDetect::HolisticTrackingDetect::HolisticTrackingDetect()
{
	m_bIsInit = false;
	m_bIsRelease = false;

	m_Video_InputStreamName = "input_video";

	m_Video_OutputStreamName = "output_video";
	m_PoseLandmarks_OutputStreamName = "pose_landmarks";
	m_LeftHandLandmarks_OutputStreamName = "left_hand_landmarks";
	m_RightHandLandmarks_OutputStreamName = "right_hand_landmarks";
	m_FaceLandmarks_OutputStreamName = "face_landmarks";

	m_pVideoPoller = nullptr;
	m_pPoseLandmarksPoller = nullptr;
	m_pLeftHandLandmarksPoller = nullptr;
	m_pRightHandLandmarksPoller = nullptr;
	m_pFaceLandmarksPoller = nullptr;
}

GoogleMediapipeDetect::HolisticTrackingDetect::~HolisticTrackingDetect()
{
	if (m_bIsInit && !m_bIsRelease)
	{
		Release();
	}
}

int GoogleMediapipeDetect::HolisticTrackingDetect::InitModel(const char* model_path)
{
	absl::Status run_status = Mediapipe_InitGraph(model_path);
	if (!run_status.ok()){
        std::cout<<"init run_status != ok !!"<<std::endl;
        std::cout<<run_status<<std::endl;
        WriteLog(run_status);
        return 0;
    }
		
	m_bIsInit = true;
	return  1;
}

int GoogleMediapipeDetect::HolisticTrackingDetect::BoxRegister(int hotBoxLeftTopX,int hotBoxLeftTopY,int hotBoxRightBottomX,int hotBoxRightBottomY ,int okGester){
    m_OKGesture = okGester;
    m_hotBoxLeftTopX = hotBoxLeftTopX;
    m_hotBoxLeftTopY = hotBoxLeftTopY;
    m_hotBoxRightBottomX = hotBoxRightBottomX;
    m_hotBoxRightBottomY = hotBoxRightBottomY;

    return 1;
}
void GoogleMediapipeDetect::HolisticTrackingDetect::DrawPoseLandmarksTest(int image_width, int image_height, void* image_data,Point2D* posePoints,const char* writeimageFullPath){
    cv::Mat frame(cv::Size(image_width, image_height), CV_8UC4, (uchar*)image_data);
    Point2D point0 = posePoints[0];
    cv::Scalar color(255, 0, 0); // BGR 格式的颜色，这里是蓝色
    cv::Scalar linecolor(0, 0, 255); // BGR 格式的颜色
    int lineType = cv::LINE_8; // 8 连通线
    // 定义线的类型，这里使用实线，线宽为 2
    int linethickness = 2;
    int thickness = -1; // 如果厚度设置为负数（如 -1），则绘制一个填充的圆，类似于点
    int radius = 2; // 点的半径，可以根据需要调整大小
    // 在图像上绘制点
    cv::circle(frame, cv::Point(point0.x*frame.cols, point0.y*frame.rows), radius, color, thickness);

    Point2D point1 = posePoints[1];
    cv::line(frame, cv::Point(point0.x * frame.cols, point0.y * frame.rows), cv::Point(point1.x * frame.cols, point1.y * frame.rows), linecolor, linethickness, lineType);
    cv::circle(frame, cv::Point(point1.x * frame.cols, point1.y), radius, color, thickness);
    Point2D point2 = posePoints[2];
    cv::line(frame, cv::Point(point1.x * frame.cols, point1.y * frame.rows), cv::Point(point2.x * frame.cols, point2.y * frame.rows), linecolor, linethickness, lineType);

    cv::circle(frame, cv::Point(point2.x * frame.cols, point2.y * frame.rows), radius, color, thickness);
    Point2D point3 = posePoints[3];
    cv::line(frame, cv::Point(point2.x * frame.cols, point2.y * frame.rows), cv::Point(point3.x * frame.cols, point3.y * frame.rows), linecolor, linethickness, lineType);
    cv::circle(frame, cv::Point(point3.x * frame.cols, point3.y * frame.rows), radius, color, thickness);
    Point2D point4 = posePoints[4];
    cv::circle(frame, cv::Point(point4.x * frame.cols, point4.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point0.x * frame.cols, point0.y * frame.rows), cv::Point(point4.x * frame.cols, point4.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point5 = posePoints[5];
    cv::circle(frame, cv::Point(point5.x * frame.cols, point5.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point4.x * frame.cols, point4.y * frame.rows), cv::Point(point5.x * frame.cols, point5.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point6 = posePoints[6];
    cv::circle(frame, cv::Point(point6.x * frame.cols, point6.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point5.x * frame.cols, point5.y * frame.rows), cv::Point(point6.x * frame.cols, point6.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point7 = posePoints[7];
    cv::circle(frame, cv::Point(point7.x * frame.cols, point7.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point3.x * frame.cols, point3.y * frame.rows), cv::Point(point7.x * frame.cols, point7.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point8 = posePoints[8];
    cv::circle(frame, cv::Point(point8.x * frame.cols, point8.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point6.x * frame.cols, point6.y * frame.rows), cv::Point(point8.x * frame.cols, point8.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point9 = posePoints[9];
    cv::circle(frame, cv::Point(point9.x * frame.cols, point9.y * frame.rows), radius, color, thickness);
    Point2D point10 = posePoints[10];
    cv::circle(frame, cv::Point(point10.x * frame.cols, point10.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point9.x * frame.cols, point9.y * frame.rows), cv::Point(point10.x * frame.cols, point10.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point11 = posePoints[11];
    cv::circle(frame, cv::Point(point11.x * frame.cols, point11.y * frame.rows), radius, color, thickness);
    Point2D point12 = posePoints[12];
    cv::circle(frame, cv::Point(point12.x * frame.cols, point12.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point11.x * frame.cols, point11.y * frame.rows), cv::Point(point12.x * frame.cols, point12.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point13 = posePoints[13];
    cv::circle(frame, cv::Point(point13.x * frame.cols, point13.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point11.x * frame.cols, point11.y * frame.rows), cv::Point(point13.x * frame.cols, point13.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point14 = posePoints[14];
    cv::circle(frame, cv::Point(point14.x * frame.cols, point14.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point12.x * frame.cols, point12.y * frame.rows), cv::Point(point14.x * frame.cols, point14.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point15 = posePoints[15];
    cv::circle(frame, cv::Point(point15.x * frame.cols, point15.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point13.x * frame.cols, point13.y * frame.rows), cv::Point(point15.x * frame.cols, point15.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point16 = posePoints[16];
    cv::circle(frame, cv::Point(point16.x * frame.cols, point16.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point14.x * frame.cols, point14.y * frame.rows), cv::Point(point16.x * frame.cols, point16.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point17 = posePoints[17];
    cv::circle(frame, cv::Point(point17.x * frame.cols, point17.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point15.x * frame.cols, point15.y * frame.rows), cv::Point(point17.x * frame.cols, point17.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point18 = posePoints[18];
    cv::circle(frame, cv::Point(point18.x * frame.cols, point18.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point16.x * frame.cols, point16.y * frame.rows), cv::Point(point18.x * frame.cols, point18.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point19 = posePoints[19];
    cv::circle(frame, cv::Point(point19.x * frame.cols, point19.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point17.x * frame.cols, point17.y * frame.rows), cv::Point(point19.x * frame.cols, point19.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point20 = posePoints[20];
    cv::circle(frame, cv::Point(point20.x * frame.cols, point20.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point18.x * frame.cols, point18.y * frame.rows), cv::Point(point20.x * frame.cols, point20.y * frame.rows), linecolor, linethickness, lineType);
    cv::line(frame, cv::Point(point20.x * frame.cols, point20.y * frame.rows), cv::Point(point16.x * frame.cols, point16.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point21 = posePoints[21];
    cv::circle(frame, cv::Point(point21.x * frame.cols, point21.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point19.x * frame.cols, point19.y * frame.rows), cv::Point(point21.x * frame.cols, point21.y * frame.rows), linecolor, linethickness, lineType);
    cv::line(frame, cv::Point(point15.x * frame.cols, point15.y * frame.rows), cv::Point(point21.x * frame.cols, point21.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point22 = posePoints[22];
    cv::circle(frame, cv::Point(point22.x * frame.cols, point22.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point16.x * frame.cols, point16.y * frame.rows), cv::Point(point22.x * frame.cols, point22.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point23 = posePoints[23];
    cv::circle(frame, cv::Point(point23.x * frame.cols, point23.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point11.x * frame.cols, point11.y * frame.rows), cv::Point(point23.x * frame.cols, point23.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point24 = posePoints[24];
    cv::circle(frame, cv::Point(point24.x * frame.cols, point24.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point12.x * frame.cols, point12.y * frame.rows), cv::Point(point24.x * frame.cols, point24.y * frame.rows), linecolor, linethickness, lineType);
    cv::line(frame, cv::Point(point23.x * frame.cols, point23.y * frame.rows), cv::Point(point24.x * frame.cols, point24.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point25 = posePoints[25];
    cv::circle(frame, cv::Point(point25.x * frame.cols, point25.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point23.x * frame.cols, point23.y * frame.rows), cv::Point(point25.x * frame.cols, point25.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point26 = posePoints[26];
    cv::circle(frame, cv::Point(point26.x * frame.cols, point26.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point24.x * frame.cols, point24.y * frame.rows), cv::Point(point26.x * frame.cols, point26.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point27 = posePoints[27];
    cv::circle(frame, cv::Point(point27.x * frame.cols, point27.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point25.x * frame.cols, point25.y * frame.rows), cv::Point(point27.x * frame.cols, point27.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point28 = posePoints[28];
    cv::circle(frame, cv::Point(point28.x * frame.cols, point28.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point26.x * frame.cols, point26.y * frame.rows), cv::Point(point28.x * frame.cols, point28.y * frame.rows), linecolor, linethickness, lineType);
    Point2D point29 = posePoints[29];
    cv::circle(frame, cv::Point(point29.x * frame.cols, point29.y * frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point27.x* frame.cols, point27.y* frame.rows), cv::Point(point29.x* frame.cols, point29.y* frame.rows), linecolor, linethickness, lineType);
    Point2D point30 = posePoints[30];
    cv::circle(frame, cv::Point(point30.x* frame.cols, point30.y* frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point28.x* frame.cols, point28.y* frame.rows), cv::Point(point30.x* frame.cols, point30.y* frame.rows), linecolor, linethickness, lineType);
    Point2D point31 = posePoints[31];
    cv::circle(frame, cv::Point(point31.x* frame.cols, point31.y* frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point29.x* frame.cols, point29.y* frame.rows), cv::Point(point31.x* frame.cols, point31.y* frame.rows), linecolor, linethickness, lineType);
    cv::line(frame, cv::Point(point31.x* frame.cols, point31.y* frame.rows), cv::Point(point27.x* frame.cols, point27.y* frame.rows), linecolor, linethickness, lineType);
    Point2D point32 = posePoints[32];
    cv::circle(frame, cv::Point(point32.x* frame.cols, point32.y* frame.rows), radius, color, thickness);
    cv::line(frame, cv::Point(point30.x* frame.cols, point30.y* frame.rows), cv::Point(point32.x* frame.cols, point32.y* frame.rows), linecolor, linethickness, lineType);
    cv::line(frame, cv::Point(point32.x* frame.cols, point32.y* frame.rows), cv::Point(point28.x* frame.cols, point28.y* frame.rows), linecolor, linethickness, lineType);
   // cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    cv::imwrite(writeimageFullPath, frame);
}
int GoogleMediapipeDetect::HolisticTrackingDetect::WriteLog(absl::Status msg){
    ofstream outfile ;
    outfile.open("C:/mediapipeLibs/MediapipeHolisticTrackingDetect.txt",ios::app); //文件的物理地址，文件的打开方式
    int m=0;
    if(outfile.is_open())
    {
        outfile<<msg<<"\n";
        outfile.close();
        return 0;
    }
    else
    {
        return 1;
    }
}

int GoogleMediapipeDetect::HolisticTrackingDetect::DetectImageDirect(int image_width, int image_height, void* image_data, int* detect_result ,Point2D* PoseLandmarks, Point2D* LeftHandLandmarks, Point2D* RightHandLandmarks, bool& isGestureOK)
{
	if (!m_bIsInit)
		return 2;

	absl::Status run_status = Mediapipe_RunMPPGraph_Direct(image_width, image_height, image_data, detect_result,PoseLandmarks,  LeftHandLandmarks,  RightHandLandmarks,  isGestureOK);
	if (!run_status.ok()) {
	    std::cout<<"run_status != ok !!"<<std::endl;
	    std::cout<<run_status<<std::endl;
        WriteLog(run_status);
		return 3;
	}
	return 1;
}

int GoogleMediapipeDetect::HolisticTrackingDetect::DetectCamera(bool show_image)
{
	if (!m_bIsInit)
		return 0;
	absl::Status run_status = Mediapipe_RunMPPGraph_Camera(show_image);
	if (!run_status.ok()) {
		return 0;
	}
	return 1;

}

int GoogleMediapipeDetect::HolisticTrackingDetect::Release()
{
	absl::Status run_status = Mediapipe_ReleaseGraph();
	if (!run_status.ok()) {
		return 0;
	}
	m_bIsRelease = true;
	return 1;
}

absl::Status GoogleMediapipeDetect::HolisticTrackingDetect::Mediapipe_InitGraph(const char* model_path)
{
	std::string calculator_graph_config_contents;
	MP_RETURN_IF_ERROR(mediapipe::file::GetContents(model_path, &calculator_graph_config_contents));
	std::cout << "mediapipe::file::GetContents success" << std::endl;

	mediapipe::CalculatorGraphConfig config =
		mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
			calculator_graph_config_contents);

	MP_RETURN_IF_ERROR(m_Graph.Initialize(config));
	std::cout << "m_Graph.Initialize(config) success" << std::endl;

	// 1 视频输出
	auto videoOutputStream = m_Graph.AddOutputStreamPoller(m_Video_OutputStreamName);
	assert(videoOutputStream.ok());
	m_pVideoPoller = std::make_unique<mediapipe::OutputStreamPoller>(std::move(videoOutputStream.value()));

	// 2 PoseLandmarks输出
	mediapipe::StatusOrPoller poseLandmarks = m_Graph.AddOutputStreamPoller(m_PoseLandmarks_OutputStreamName);
	assert(poseLandmarks.ok());
	m_pPoseLandmarksPoller = std::make_unique<mediapipe::OutputStreamPoller>(std::move(poseLandmarks.value()));

	// 3 LeftHandLandmarks输出
	mediapipe::StatusOrPoller leftHandLandmarks = m_Graph.AddOutputStreamPoller(m_LeftHandLandmarks_OutputStreamName);
	assert(leftHandLandmarks.ok());
	m_pLeftHandLandmarksPoller = std::make_unique<mediapipe::OutputStreamPoller>(std::move(leftHandLandmarks.value()));

	// 4 RightHandLandmarks输出
	mediapipe::StatusOrPoller rightHandLandmarks = m_Graph.AddOutputStreamPoller(m_RightHandLandmarks_OutputStreamName);
	assert(rightHandLandmarks.ok());
	m_pRightHandLandmarksPoller = std::make_unique<mediapipe::OutputStreamPoller>(std::move(rightHandLandmarks.value()));

	// 5 FaceLandmarks输出
	mediapipe::StatusOrPoller faceLandmarks = m_Graph.AddOutputStreamPoller(m_FaceLandmarks_OutputStreamName);
	assert(faceLandmarks.ok());
	m_pFaceLandmarksPoller = std::make_unique<mediapipe::OutputStreamPoller>(std::move(faceLandmarks.value()));

	MP_RETURN_IF_ERROR(m_Graph.StartRun({}));
	std::cout << "----------------Graph StartRun Success---------------------" << std::endl;
	return absl::OkStatus();
}

absl::Status GoogleMediapipeDetect::HolisticTrackingDetect::Mediapipe_RunMPPGraph_Direct(int image_width, int image_height, void* image_data, int* detect_result ,Point2D* PoseLandmarks, Point2D* LeftHandLandmarks, Point2D* RightHandLandmarks, bool& isGestureOK)
{
    m_frameIndex++;
    cv::Mat camera_frame(cv::Size(image_width, image_height), CV_8UC4, (uchar*)image_data);
    
    cv::cvtColor(camera_frame, camera_frame, cv::COLOR_BGRA2BGR);
    cv::rectangle(camera_frame, cv::Point(m_hotBoxLeftTopX, m_hotBoxLeftTopY), cv::Point(m_hotBoxRightBottomX, m_hotBoxRightBottomY), cv::Scalar(0, 0, 255), 3, 8, 0);
    cv::cvtColor(camera_frame, camera_frame, cv::COLOR_BGR2RGB);
	auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
		mediapipe::ImageFormat::SRGB, camera_frame.cols, camera_frame.rows,
		mediapipe::ImageFrame::kDefaultAlignmentBoundary);
	cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
	camera_frame.copyTo(input_frame_mat);
	size_t frame_timestamp_us =
		(double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;

	MP_RETURN_IF_ERROR(m_Graph.AddPacketToInputStream(
		m_Video_InputStreamName, mediapipe::Adopt(input_frame.release())
		.At(mediapipe::Timestamp(frame_timestamp_us))));
	mediapipe::Packet packet;
	if (!m_pVideoPoller->Next(&packet))
	{
		return absl::InvalidArgumentError("no next packet");
	}

    // 3 LeftHandLandmarks
//    mediapipe::Packet leftHandLandmarksPacket;
//    int leftHandDetectResult = Gesture::NoGesture;
//    if (m_pLeftHandLandmarksPoller->QueueSize() > 0)
//    {
//        if (m_pLeftHandLandmarksPoller->Next(&leftHandLandmarksPacket))
//        {
//            auto& output_landmarks = leftHandLandmarksPacket.Get<mediapipe::NormalizedLandmarkList>();
//            //std::cout << "LeftHandLandmarks size:" << output_landmarks.landmark_size() << std::endl;
//
//            std::vector<Point2D> singleGesturePoints;
//            singleGesturePoints.clear();
//
//            for (int i = 0; i < output_landmarks.landmark_size(); ++i)
//            {
//                Point2D tempPoint2D;
//                const mediapipe::NormalizedLandmark landmark = output_landmarks.landmark(i);
//                float x = landmark.x() * camera_frame.cols;
//                float y = landmark.y() * camera_frame.rows;
//                tempPoint2D.x = x;
//                tempPoint2D.y = y;
//                LeftHandLandmarks[i].x = x;
//                LeftHandLandmarks[i].y = y;
//                singleGesturePoints.emplace_back(tempPoint2D);
//            }
//
//            GestureRecognition gestureRecognition;
//            leftHandDetectResult = gestureRecognition.RecognizeProcess(singleGesturePoints);
//            //std::cout << "左手手势识别结果：" << leftHandDetectResult << std::endl;
//        }
//    }
//    detect_result[2] = leftHandDetectResult;

    // 4 RightHandLandmarks
//    mediapipe::Packet rightHandLandmarksPacket;
//    int rightHandDetectResult = Gesture::NoGesture;
//    if (m_pRightHandLandmarksPoller->QueueSize() > 0)
//    {
//        if (m_pRightHandLandmarksPoller->Next(&rightHandLandmarksPacket))
//        {
//            auto& output_landmarks = rightHandLandmarksPacket.Get<mediapipe::NormalizedLandmarkList>();
//            //std::cout << "RightHandLandmarks size:" << output_landmarks.landmark_size() << std::endl;
//
//            std::vector<Point2D> singleGesturePoints;
//            singleGesturePoints.clear();
//
//            for (int i = 0; i < output_landmarks.landmark_size(); ++i)
//            {
//                Point2D tempPoint2D;
//                const mediapipe::NormalizedLandmark landmark = output_landmarks.landmark(i);
//                float x = landmark.x() * camera_frame.cols;
//                float y = landmark.y() * camera_frame.rows;
//                tempPoint2D.x = x;
//                tempPoint2D.y = y;
//                RightHandLandmarks[i].x = x;
//                RightHandLandmarks[i].y = y;
//                singleGesturePoints.emplace_back(tempPoint2D);
//            }
//
//            GestureRecognition gestureRecognition;
//            rightHandDetectResult = gestureRecognition.RecognizeProcess(singleGesturePoints);
//            //std::cout << "右手手势识别结果：" << rightHandDetectResult << std::endl;
//
//        }
//    }
//    detect_result[3] = rightHandDetectResult;

	// 5 PoseLandmarks
	mediapipe::Packet poseeLandmarksPacket;
	if (m_pPoseLandmarksPoller->QueueSize() != 0)
	{
		if (m_pPoseLandmarksPoller->Next(&poseeLandmarksPacket))
		{
			auto& output_landmarks = poseeLandmarksPacket.Get<mediapipe::NormalizedLandmarkList>();
			//std::cout << "PoseLandmarks size:" << output_landmarks.landmark_size() << std::endl;

			std::vector<Point2D> posePoints;
			posePoints.clear();

			for (int i = 0; i < output_landmarks.landmark_size(); ++i)
			{
				Point2D tempPoint2D;
				const mediapipe::NormalizedLandmark landmark = output_landmarks.landmark(i);
//				float x = landmark.x() * camera_frame.cols;
//				float y = landmark.y() * camera_frame.rows;
                float x = landmark.x() ;
                float y = landmark.y() ;
				tempPoint2D.x = x;
				tempPoint2D.y = y;
                PoseLandmarks[i].x = x;
                PoseLandmarks[i].y = y;
				//posePoints.emplace_back(tempPoint2D);

			}
           /* leftHandPoint.x = PoseLandmarks[15].x;
            leftHandPoint.y = PoseLandmarks[15].y;
			rightHandPoint.x = PoseLandmarks[16].x;
            rightHandPoint.y = PoseLandmarks[16].y;

            bool leftGestureOK;
            if(leftHandPoint.x < 0 || leftHandPoint.x > image_width || leftHandPoint.y < 0 || leftHandPoint.y >image_height ||
               rightHandPoint.x < 0 || rightHandPoint.x > image_width || rightHandPoint.y < 0 || rightHandPoint.y > image_height){
                isGestureOK = false;
            }else {
                if( (leftHandPoint.x > m_hotBoxLeftTopX) && (leftHandPoint.x < m_hotBoxRightBottomX) && (leftHandPoint.y > m_hotBoxLeftTopY) && (leftHandPoint.y < m_hotBoxRightBottomY)){
                   
                    if(leftHandDetectResult == Gesture::Five || leftHandDetectResult == Gesture::Four){
                         leftGestureOK =  true;
                     }else{
                         leftGestureOK =  false;
                     }
                }else{
                    leftGestureOK =  false;
                }
                bool rightGestureOK;
                if( (rightHandPoint.x > m_hotBoxLeftTopX) && (rightHandPoint.x < m_hotBoxRightBottomX) && (rightHandPoint.y > m_hotBoxLeftTopY) && (rightHandPoint.y < m_hotBoxRightBottomY)){
                    if(rightHandDetectResult == Gesture::Five || rightHandDetectResult == Gesture::Four){
                         rightGestureOK =  true;
                     }else{
                         rightGestureOK =  false;
                     }
                }else{
                    rightGestureOK =  false;
                }

                isGestureOK = leftGestureOK || rightGestureOK;
            }*/
            
			/*ArmUpAndDownRecognition armUpAndDownRecognition;
			armUpAndDownRecognition.RecognizeProcess(posePoints,
                                                     m_left_arm_current_result,
                                                     m_right_arm_current_result,
                                                     m_left_arm_current_leftRight_result,
                                                     m_right_arm_current_leftRight_result,
                                                     left_arm_up_down_result_DIFF,
                                                     right_arm_up_down_result_DIFF,
                                                     left_arm_left_right_result_DIFF,
                                                     right_arm_left_right_result_DIFF);
			//std::cout << "手臂抬手放手识别结果：" << poseDetectResult << std::endl;
            m_left_arm_result_list[0] = m_left_arm_pre_result;
            m_right_arm_result_list[0] = m_right_arm_pre_result;
            m_left_arm_leftRight_result_list[0] = m_left_arm_pre_leftRight_result;
            m_right_arm_leftRight_result_list[0] = m_right_arm_pre_leftRight_result;

            m_left_arm_result_list[1] = m_left_arm_current_result;
            m_right_arm_result_list[1] = m_right_arm_current_result;
            m_left_arm_leftRight_result_list[1] = m_left_arm_current_leftRight_result;
            m_right_arm_leftRight_result_list[1] = m_right_arm_current_leftRight_result;

            m_left_arm_result_list[2] = left_arm_up_down_result_DIFF;
            m_right_arm_result_list[2] = right_arm_up_down_result_DIFF;
            m_left_arm_leftRight_result_list[2] = left_arm_left_right_result_DIFF;
            m_right_arm_leftRight_result_list[2] = right_arm_left_right_result_DIFF;

            HandsPositionRecognition handsPositionRecognition;
            handsPositionRecognition.PositionRecognizeProcess(posePoints,left_hand_position_result,right_hand_position_result);
            LovePoseRecognition lovePoseRecognition;
            lovePoseRecognition.PoseRecognizeProcess(posePoints,pose_result);*/
		}
	}
/*
	ArmWavedRecognition armWavedRecognition;
	armWavedRecognition.WavedRecognizeProcess(m_left_arm_result_list,m_right_arm_result_list,m_left_arm_leftRight_result_list,m_right_arm_leftRight_result_list,left_arm_waved_updown_result,right_arm_waved_updown_result,left_arm_waved_leftright_result,right_arm_waved_leftright_result);



    //std::cout << "333333333333333333333" << std::endl;
    //左，右臂，放上边，下边结果
	detect_result[0] = m_left_arm_current_result;
	detect_result[1] = m_right_arm_current_result;
//    std::cout << "HolisticTrackingDetect left_arm_result:" <<m_left_arm_current_result<< std::endl;
//    std::cout << "HolisticTrackingDetect right_arm_result:" <<m_right_arm_current_result<< std::endl;

	//左，右臂，放左边，右边结果
    detect_result[4] = m_left_arm_current_leftRight_result;
    detect_result[5] = m_right_arm_current_leftRight_result;

    //左，右臂，上下，左右挥动结果
    detect_result[6] = left_arm_waved_updown_result;
    detect_result[7] = right_arm_waved_updown_result;
    detect_result[8] = left_arm_waved_leftright_result;
    detect_result[9] = right_arm_waved_leftright_result;


    if(leftHandDetectResult != Gesture::NoGesture){
        detect_result[10] = left_hand_position_result;
    }else {
        detect_result[10] = HandPosition::HandPosition_No;
    }

    if(rightHandDetectResult != Gesture::NoGesture){
        detect_result[11] = right_hand_position_result;
    }else {
        detect_result[11] = HandPosition::HandPosition_No;
    }

    detect_result[12] = pose_result;

    m_left_arm_pre_result = m_left_arm_current_result;
    m_right_arm_pre_result = m_right_arm_current_result;
    m_left_arm_pre_leftRight_result = m_left_arm_current_leftRight_result;
    m_right_arm_pre_leftRight_result = m_right_arm_current_leftRight_result;*/

	return absl::OkStatus();
}


absl::Status GoogleMediapipeDetect::HolisticTrackingDetect::Mediapipe_RunMPPGraph_Camera(bool show_image)
{
	std::string cvWindowName = "MediapipeHolistic";

	// 打开OpenCV摄像头
	cv::VideoCapture capture(0);
	if (!capture.isOpened())
	{
		return absl::InvalidArgumentError("cv camera is not open");
	}

	bool grab_frames = true;
	while (grab_frames) {

		// 从摄像头抓取视频帧
		cv::Mat camera_frame_raw;
		capture >> camera_frame_raw;
		if (camera_frame_raw.empty())
			break;

		cv::Mat camera_frame;
		cv::cvtColor(camera_frame_raw, camera_frame, cv::COLOR_BGR2RGB);
		cv::flip(camera_frame, camera_frame, 1);

		// 将OpenCV Mat转换为ImageFrame
		auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
			mediapipe::ImageFormat::SRGB, camera_frame.cols, camera_frame.rows,
			mediapipe::ImageFrame::kDefaultAlignmentBoundary);
		cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
		camera_frame.copyTo(input_frame_mat);

		// 发送图片到图中推理
		size_t frame_timestamp_us =
			(double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;

		MP_RETURN_IF_ERROR(m_Graph.AddPacketToInputStream(
			m_Video_InputStreamName, mediapipe::Adopt(input_frame.release())
			.At(mediapipe::Timestamp(frame_timestamp_us))));

		// 获取推理结果
		mediapipe::Packet packet;
		if (!m_pVideoPoller->Next(&packet)) break;

		if (show_image)
		{
			// 从视频输出获取mediapipe::ImageFrame结果
			auto& output_frame = packet.Get<mediapipe::ImageFrame>();

			// 转换mediapipe::ImageFrame为cv::Mat
			cv::Mat output_frame_mat = mediapipe::formats::MatView(&output_frame);

			// 显示cv::Mat结果
			cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGB2BGR);
			cv::Mat dst;
			cv::resize(output_frame_mat, dst, cv::Size(output_frame_mat.cols / 2, output_frame_mat.rows / 2));
			cv::imshow(cvWindowName, dst);
			cv::waitKey(1);
		}
	}
	if (show_image)
		cv::destroyWindow(cvWindowName);

	return absl::OkStatus();
}

absl::Status GoogleMediapipeDetect::HolisticTrackingDetect::Mediapipe_ReleaseGraph()
{
	MP_RETURN_IF_ERROR(m_Graph.CloseInputStream(m_Video_InputStreamName));
	// MP_RETURN_IF_ERROR(m_Graph.CloseInputStream(m_Video_OutputStreamName));
	// MP_RETURN_IF_ERROR(m_Graph.CloseInputStream(m_PoseLandmarks_OutputStreamName));
	// MP_RETURN_IF_ERROR(m_Graph.CloseInputStream(m_LeftHandLandmarks_OutputStreamName));
	// MP_RETURN_IF_ERROR(m_Graph.CloseInputStream(m_RightHandLandmarks_OutputStreamName));
	// MP_RETURN_IF_ERROR(m_Graph.CloseInputStream(m_FaceLandmarks_OutputStreamName));

	return m_Graph.WaitUntilDone();
}
//*******备份2022年11月25日之前的手势算法，通过判断挥动手臂的手掌关键点和躯干关键点的位置来处理  *****//
//absl::Status GoogleMediapipeDetect::HolisticTrackingDetect::Mediapipe_RunMPPGraph_Direct(int image_width, int image_height, void* image_data, int* detect_result ,Point2D* PoseLandmarks, Point2D* LeftHandLandmarks, Point2D* RightHandLandmarks, bool& isGestureOK)
//{
//    m_frameIndex++;
//    /*----- 1 构造cv::Mat对象 -----*/
//    // cv::Mat camera_frame(cv::Size(image_width, image_height), CV_8UC3, (uchar*)image_data);
//    // cv::cvtColor(camera_frame, camera_frame, cv::COLOR_BGR2RGB);
//    cv::Mat camera_frame(cv::Size(image_width, image_height), CV_8UC4, (uchar*)image_data);
//
//    cv::cvtColor(camera_frame, camera_frame, cv::COLOR_BGRA2BGR);
//    cv::rectangle(camera_frame, cv::Point(m_hotBoxLeftTopX, m_hotBoxLeftTopY), cv::Point(m_hotBoxRightBottomX, m_hotBoxRightBottomY), cv::Scalar(0, 0, 255), 3, 8, 0);
//    // cv::imwrite("C:/mediapipeLibs/MediapipeGestureAction.jpg", camera_frame);
//    cv::cvtColor(camera_frame, camera_frame, cv::COLOR_BGR2RGB);
//    // 水平翻转输入图像
//    //cv::flip(camera_frame, camera_frame, 1);
//    //std::cout << "mediapipe 1111111111111111" << std::endl;
//
//    /*----- 2 将OpenCV Mat转换为ImageFrame -----*/
//    auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
//            mediapipe::ImageFormat::SRGB, camera_frame.cols, camera_frame.rows,
//            mediapipe::ImageFrame::kDefaultAlignmentBoundary);
//    cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
//    camera_frame.copyTo(input_frame_mat);
//    //std::cout << "mediapipe 222222222222222" << std::endl;
//
//    /*----- 3 发送图片到图中推理 -----*/
//    size_t frame_timestamp_us =
//            (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;
//
//    MP_RETURN_IF_ERROR(m_Graph.AddPacketToInputStream(
//            m_Video_InputStreamName, mediapipe::Adopt(input_frame.release())
//                    .At(mediapipe::Timestamp(frame_timestamp_us))));
//    //std::cout << "mediapipe 3333333333333" << std::endl;
//
//    /*----- 4 得到结果 -----*/
//
//    // 1 视频输出结果帧
//    mediapipe::Packet packet;
//    if (!m_pVideoPoller->Next(&packet))
//    {
//        return absl::InvalidArgumentError("no next packet");
//    }
//    // if (show_result_image)
//    // {
//    // 	// 从视频输出获取mediapipe::ImageFrame结果
//    // 	auto& output_frame = packet.Get<mediapipe::ImageFrame>();
//
//    // 	// 转换mediapipe::ImageFrame为cv::Mat
//    // 	cv::Mat output_frame_mat = mediapipe::formats::MatView(&output_frame);
//
//    // 	// 显示cv::Mat结果
//    // 	cv::cvtColor(output_frame_mat, output_frame_mat, cv::COLOR_RGB2BGR);
//    // 	cv::Mat dst;
//    // 	cv::resize(output_frame_mat, dst, cv::Size(output_frame_mat.cols, output_frame_mat.rows));
//    // 	cv::imshow("MediapipeHolistic", dst);
//    // 	cv::waitKey(1);
//    // }
//
//    // 3 LeftHandLandmarks
//    mediapipe::Packet leftHandLandmarksPacket;
//    int leftHandDetectResult = Gesture::NoGesture;
//    if (m_pLeftHandLandmarksPoller->QueueSize() > 0)
//    {
//        if (m_pLeftHandLandmarksPoller->Next(&leftHandLandmarksPacket))
//        {
//            auto& output_landmarks = leftHandLandmarksPacket.Get<mediapipe::NormalizedLandmarkList>();
//            //std::cout << "LeftHandLandmarks size:" << output_landmarks.landmark_size() << std::endl;
//
//            std::vector<Point2D> singleGesturePoints;
//            singleGesturePoints.clear();
//
//            for (int i = 0; i < output_landmarks.landmark_size(); ++i)
//            {
//                Point2D tempPoint2D;
//                const mediapipe::NormalizedLandmark landmark = output_landmarks.landmark(i);
//                float x = landmark.x() * camera_frame.cols;
//                float y = landmark.y() * camera_frame.rows;
//                tempPoint2D.x = x;
//                tempPoint2D.y = y;
//                LeftHandLandmarks[i].x = x;
//                LeftHandLandmarks[i].y = y;
//                singleGesturePoints.emplace_back(tempPoint2D);
//            }
//
//            GestureRecognition gestureRecognition;
//            leftHandDetectResult = gestureRecognition.RecognizeProcess(singleGesturePoints);
//            //std::cout << "左手手势识别结果：" << leftHandDetectResult << std::endl;
//        }
//    }
//    detect_result[2] = leftHandDetectResult;
//
//    // 4 RightHandLandmarks
//    mediapipe::Packet rightHandLandmarksPacket;
//    int rightHandDetectResult = Gesture::NoGesture;
//    if (m_pRightHandLandmarksPoller->QueueSize() > 0)
//    {
//        if (m_pRightHandLandmarksPoller->Next(&rightHandLandmarksPacket))
//        {
//            auto& output_landmarks = rightHandLandmarksPacket.Get<mediapipe::NormalizedLandmarkList>();
//            //std::cout << "RightHandLandmarks size:" << output_landmarks.landmark_size() << std::endl;
//
//            std::vector<Point2D> singleGesturePoints;
//            singleGesturePoints.clear();
//
//            for (int i = 0; i < output_landmarks.landmark_size(); ++i)
//            {
//                Point2D tempPoint2D;
//                const mediapipe::NormalizedLandmark landmark = output_landmarks.landmark(i);
//                float x = landmark.x() * camera_frame.cols;
//                float y = landmark.y() * camera_frame.rows;
//                tempPoint2D.x = x;
//                tempPoint2D.y = y;
//                RightHandLandmarks[i].x = x;
//                RightHandLandmarks[i].y = y;
//                singleGesturePoints.emplace_back(tempPoint2D);
//            }
//
//            GestureRecognition gestureRecognition;
//            rightHandDetectResult = gestureRecognition.RecognizeProcess(singleGesturePoints);
//            //std::cout << "右手手势识别结果：" << rightHandDetectResult << std::endl;
//
//        }
//    }
//    detect_result[3] = rightHandDetectResult;
//
//    // 2 PoseLandmarks
//    mediapipe::Packet poseeLandmarksPacket;
////	int left_arm_result = (int)ArmUpDown::NoResult;
////	int right_arm_result = (int)ArmUpDown::NoResult;
////	int left_arm_leftRight_result = (int)ArmLeftRight::NoLeftRightResult;
////	int right_arm_leftRight_result = (int)ArmLeftRight::NoLeftRightResult;
//
//    int left_arm_waved_updown_result = (int)ArmWaved::NoArmWavedResult;
//    int right_arm_waved_updown_result = (int)ArmWaved::NoArmWavedResult;
//    int left_arm_waved_leftright_result = (int)ArmWaved::NoArmWavedResult;
//    int right_arm_waved_leftright_result = (int)ArmWaved::NoArmWavedResult;
//
//    int left_arm_up_down_result_DIFF;
//    int right_arm_up_down_result_DIFF;
//
//    int left_arm_left_right_result_DIFF;
//    int right_arm_left_right_result_DIFF;
//
//    Point2D leftHandPoint;
//    Point2D rightHandPoint;
//    if (m_pPoseLandmarksPoller->QueueSize() != 0)
//    {
//        if (m_pPoseLandmarksPoller->Next(&poseeLandmarksPacket))
//        {
//            auto& output_landmarks = poseeLandmarksPacket.Get<mediapipe::NormalizedLandmarkList>();
//            //std::cout << "PoseLandmarks size:" << output_landmarks.landmark_size() << std::endl;
//
//            std::vector<Point2D> posePoints;
//            posePoints.clear();
//
//            for (int i = 0; i < output_landmarks.landmark_size(); ++i)
//            {
//                Point2D tempPoint2D;
//                const mediapipe::NormalizedLandmark landmark = output_landmarks.landmark(i);
//                float x = landmark.x() * camera_frame.cols;
//                float y = landmark.y() * camera_frame.rows;
//                tempPoint2D.x = x;
//                tempPoint2D.y = y;
//                PoseLandmarks[i].x = x;
//                PoseLandmarks[i].y = y;
//                posePoints.emplace_back(tempPoint2D);
//
//            }
//            leftHandPoint.x = PoseLandmarks[15].x;
//            leftHandPoint.y = PoseLandmarks[15].y;
//            rightHandPoint.x = PoseLandmarks[16].x;
//            rightHandPoint.y = PoseLandmarks[16].y;
//
//            bool leftGestureOK;
//            if(leftHandPoint.x < 0 || leftHandPoint.x > image_width || leftHandPoint.y < 0 || leftHandPoint.y >image_height ||
//               rightHandPoint.x < 0 || rightHandPoint.x > image_width || rightHandPoint.y < 0 || rightHandPoint.y > image_height){
//                isGestureOK = false;
//            }else {
//                if( (leftHandPoint.x > m_hotBoxLeftTopX) && (leftHandPoint.x < m_hotBoxRightBottomX) && (leftHandPoint.y > m_hotBoxLeftTopY) && (leftHandPoint.y < m_hotBoxRightBottomY)){
//
//                    if(leftHandDetectResult == Gesture::Five || leftHandDetectResult == Gesture::Four){
//                        leftGestureOK =  true;
//                    }else{
//                        leftGestureOK =  false;
//                    }
//                }else{
//                    leftGestureOK =  false;
//                }
//                bool rightGestureOK;
//                if( (rightHandPoint.x > m_hotBoxLeftTopX) && (rightHandPoint.x < m_hotBoxRightBottomX) && (rightHandPoint.y > m_hotBoxLeftTopY) && (rightHandPoint.y < m_hotBoxRightBottomY)){
//                    if(rightHandDetectResult == Gesture::Five || rightHandDetectResult == Gesture::Four){
//                        rightGestureOK =  true;
//                    }else{
//                        rightGestureOK =  false;
//                    }
//                }else{
//                    rightGestureOK =  false;
//                }
//
//                isGestureOK = leftGestureOK || rightGestureOK;
//            }
//
//            ArmUpAndDownRecognition armUpAndDownRecognition;
//            armUpAndDownRecognition.RecognizeProcess(posePoints,
//                                                     m_left_arm_current_result,
//                                                     m_right_arm_current_result,
//                                                     m_left_arm_current_leftRight_result,
//                                                     m_right_arm_current_leftRight_result,
//                                                     left_arm_up_down_result_DIFF,
//                                                     right_arm_up_down_result_DIFF,
//                                                     left_arm_left_right_result_DIFF,
//                                                     right_arm_left_right_result_DIFF);
//            //std::cout << "手臂抬手放手识别结果：" << poseDetectResult << std::endl;
//            m_left_arm_result_list[0] = m_left_arm_pre_result;
//            m_right_arm_result_list[0] = m_right_arm_pre_result;
//            m_left_arm_leftRight_result_list[0] = m_left_arm_pre_leftRight_result;
//            m_right_arm_leftRight_result_list[0] = m_right_arm_pre_leftRight_result;
//
//            m_left_arm_result_list[1] = m_left_arm_current_result;
//            m_right_arm_result_list[1] = m_right_arm_current_result;
//            m_left_arm_leftRight_result_list[1] = m_left_arm_current_leftRight_result;
//            m_right_arm_leftRight_result_list[1] = m_right_arm_current_leftRight_result;
//
//            m_left_arm_result_list[2] = left_arm_up_down_result_DIFF;
//            m_right_arm_result_list[2] = right_arm_up_down_result_DIFF;
//            m_left_arm_leftRight_result_list[2] = left_arm_left_right_result_DIFF;
//            m_right_arm_leftRight_result_list[2] = right_arm_left_right_result_DIFF;
//        }
//    }
//
//    ArmWavedRecognition armWavedRecognition;
//    armWavedRecognition.WavedRecognizeProcess(m_left_arm_result_list,m_right_arm_result_list,m_left_arm_leftRight_result_list,m_right_arm_leftRight_result_list,left_arm_waved_updown_result,right_arm_waved_updown_result,left_arm_waved_leftright_result,right_arm_waved_leftright_result);
//
//    //std::cout << "333333333333333333333" << std::endl;
//    //左，右臂，放上边，下边结果
//    detect_result[0] = m_left_arm_current_result;
//    detect_result[1] = m_right_arm_current_result;
////    std::cout << "HolisticTrackingDetect left_arm_result:" <<m_left_arm_current_result<< std::endl;
////    std::cout << "HolisticTrackingDetect right_arm_result:" <<m_right_arm_current_result<< std::endl;
//
//    //左，右臂，放左边，右边结果
//    detect_result[4] = m_left_arm_current_leftRight_result;
//    detect_result[5] = m_right_arm_current_leftRight_result;
//
//    //左，右臂，上下，左右挥动结果
//    detect_result[6] = left_arm_waved_updown_result;
//    detect_result[7] = right_arm_waved_updown_result;
//    detect_result[8] = left_arm_waved_leftright_result;
//    detect_result[9] = right_arm_waved_leftright_result;
//
//    m_left_arm_pre_result = m_left_arm_current_result;
//    m_right_arm_pre_result = m_right_arm_current_result;
//    m_left_arm_pre_leftRight_result = m_left_arm_current_leftRight_result;
//    m_right_arm_pre_leftRight_result = m_right_arm_current_leftRight_result;
//
//    //std::cout << "444444444444444" << std::endl;
//    // 4 FaceLandmarks
//    //mediapipe::Packet faceLandmarksPacket;
//    //if (m_pFaceLandmarksPoller->QueueSize() > 0)
//    //{
//    //	if (m_pFaceLandmarksPoller->Next(&faceLandmarksPacket))
//    //	{
//    //		auto& output_landmarks = faceLandmarksPacket.Get<mediapipe::NormalizedLandmarkList>();
//    //		std::cout << "FaceLandmarks size:" << output_landmarks.landmark_size() << std::endl;
//
//    //		for (int i = 0; i < output_landmarks.landmark_size(); ++i)
//    //		{
//    //			const mediapipe::NormalizedLandmark landmark = output_landmarks.landmark(i);
//    //			float x = landmark.x() * camera_frame.cols;
//    //			float y = landmark.y() * camera_frame.rows;
//    //		}
//    //	}
//    //}
//
//    return absl::OkStatus();
//}
