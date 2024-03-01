//
// Created by GMY on 2022/4/1.
//
#include <vector>

#include "PoseTrackingDetect.h"
#include "TrackingDataStructure.h"
#include "ArmUpAndDownRecognition.h"
#include "ArmWavedRecognition.h"

GoogleMediapipeDetect::PoseTrackingDetect::PoseTrackingDetect()
{
    m_bIsInit = false;
    m_bIsRelease = false;

    m_Video_InputStreamName = "input_video";

    m_Video_OutputStreamName = "output_video";
    m_PoseLandmarks_OutputStreamName = "pose_landmarks";
    
    m_pVideoPoller = nullptr;
    m_pPoseLandmarksPoller = nullptr;
}

GoogleMediapipeDetect::PoseTrackingDetect::~PoseTrackingDetect()
{
    if (m_bIsInit && !m_bIsRelease)
    {
        Release();
    }
}

int GoogleMediapipeDetect::PoseTrackingDetect::InitModel(const char* model_path)
{
    absl::Status run_status = Mediapipe_InitGraph(model_path);
    if (!run_status.ok())
        return 0;
    m_bIsInit = true;
    return  1;
}

int GoogleMediapipeDetect::PoseTrackingDetect::BoxRegister(int hotBoxLeftTopX,int hotBoxLeftTopY,int hotBoxRightBottomX,int hotBoxRightBottomY ,int okGester){
    m_OKGesture = okGester;
    m_hotBoxLeftTopX = hotBoxLeftTopX;
    m_hotBoxLeftTopY = hotBoxLeftTopY;
    m_hotBoxRightBottomX = hotBoxRightBottomX;
    m_hotBoxRightBottomY = hotBoxRightBottomY;

    return 1;
}

int GoogleMediapipeDetect::PoseTrackingDetect::DetectImageDirect(int image_width, int image_height, void* image_data, int* detect_result ,Point2D* PoseLandmarks, Point2D* LeftHandLandmarks, Point2D* RightHandLandmarks, bool& isGestureOK)
{
    if (!m_bIsInit)
        return 2;

    absl::Status run_status = Mediapipe_RunMPPGraph_Direct(image_width, image_height, image_data, detect_result,PoseLandmarks,  LeftHandLandmarks,  RightHandLandmarks,  isGestureOK);
    if (!run_status.ok()) {
        return 3;
    }
    return 1;
}

int GoogleMediapipeDetect::PoseTrackingDetect::Release()
{
    absl::Status run_status = Mediapipe_ReleaseGraph();
    if (!run_status.ok()) {
        return 0;
    }
    m_bIsRelease = true;
    return 1;
}

absl::Status GoogleMediapipeDetect::PoseTrackingDetect::Mediapipe_InitGraph(const char* model_path)
{
    std::string calculator_graph_config_contents;
    MP_RETURN_IF_ERROR(mediapipe::file::GetContents(model_path, &calculator_graph_config_contents));
    std::cout << "mediapipe::file::GetContents success" << std::endl;

    mediapipe::CalculatorGraphConfig config =
            mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
                    calculator_graph_config_contents);

    MP_RETURN_IF_ERROR(m_Graph.Initialize(config));
    std::cout << "m_Graph.Initialize(config) success" << std::endl;

    // 1 ÊÓÆµÊä³ö
    auto videoOutputStream = m_Graph.AddOutputStreamPoller(m_Video_OutputStreamName);
    assert(videoOutputStream.ok());
    m_pVideoPoller = std::make_unique<mediapipe::OutputStreamPoller>(std::move(videoOutputStream.value()));

    // 2 PoseLandmarksÊä³ö
    mediapipe::StatusOrPoller poseLandmarks = m_Graph.AddOutputStreamPoller(m_PoseLandmarks_OutputStreamName);
    assert(poseLandmarks.ok());
    m_pPoseLandmarksPoller = std::make_unique<mediapipe::OutputStreamPoller>(std::move(poseLandmarks.value()));

    MP_RETURN_IF_ERROR(m_Graph.StartRun({}));
    std::cout << "----------------Graph StartRun Success---------------------" << std::endl;
    return absl::OkStatus();
}

void rotate_arbitrarily_angle(cv::Mat& src, cv::Mat& dst, float angle)
{
    float radian = (float)(angle / 180.0 * CV_PI);

    //填充图像
    int maxBorder = (int)(std::max(src.cols, src.rows) * 1.414); //即为sqrt(2)*max
    int dx = (maxBorder - src.cols) / 2;
    int dy = (maxBorder - src.rows) / 2;
    copyMakeBorder(src, dst, dy, dy, dx, dx, cv::BORDER_CONSTANT);

    //旋转
    cv::Point2f center((float)(dst.cols / 2), (float)(dst.rows / 2));
    cv::Mat affine_matrix = getRotationMatrix2D(center, angle, 1.0);//求得旋转矩阵
    warpAffine(dst, dst, affine_matrix, dst.size());

    //计算图像旋转之后包含图像的最大的矩形
    float sinVal = abs(sin(radian));
    float cosVal = abs(cos(radian));
    cv::Size targetSize((int)(src.cols * cosVal + src.rows * sinVal),
        (int)(src.cols * sinVal + src.rows * cosVal));

    //剪掉多余边框
    int x = (dst.cols - targetSize.width) / 2;
    int y = (dst.rows - targetSize.height) / 2;
    cv::Rect rect(x, y, targetSize.width, targetSize.height);
    dst = cv::Mat(dst, rect);
}

void DrawPoseLandmarks(cv::Mat frame, Point2D* PoseLandmarks) {
    for (int i = 0; i < 33; i++) {
        Point2D info = PoseLandmarks[i];
        if (info.x <= 0 || info.y <= 0) {
            return;
        }
        cv::Point point = cv::Point(info.x, info.y);
        cv::circle(frame, point, 6, cv::Scalar(31, 188, 210), -1);
    }

    cv::Scalar color = cv::Scalar(0, 255, 0);
    cv::line(frame, cv::Point(PoseLandmarks[0].x, PoseLandmarks[0].y), cv::Point(PoseLandmarks[1].x, PoseLandmarks[1].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[1].x, PoseLandmarks[1].y), cv::Point(PoseLandmarks[2].x, PoseLandmarks[2].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[2].x, PoseLandmarks[2].y), cv::Point(PoseLandmarks[3].x, PoseLandmarks[3].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[3].x, PoseLandmarks[3].y), cv::Point(PoseLandmarks[7].x, PoseLandmarks[7].y), color);

    cv::line(frame, cv::Point(PoseLandmarks[0].x, PoseLandmarks[0].y), cv::Point(PoseLandmarks[4].x, PoseLandmarks[4].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[4].x, PoseLandmarks[4].y), cv::Point(PoseLandmarks[5].x, PoseLandmarks[5].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[5].x, PoseLandmarks[5].y), cv::Point(PoseLandmarks[6].x, PoseLandmarks[6].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[6].x, PoseLandmarks[6].y), cv::Point(PoseLandmarks[8].x, PoseLandmarks[8].y), color);

    cv::line(frame, cv::Point(PoseLandmarks[9].x, PoseLandmarks[9].y), cv::Point(PoseLandmarks[10].x, PoseLandmarks[10].y), color);

    cv::line(frame, cv::Point(PoseLandmarks[11].x, PoseLandmarks[11].y), cv::Point(PoseLandmarks[12].x, PoseLandmarks[12].y), color);

    cv::line(frame, cv::Point(PoseLandmarks[11].x, PoseLandmarks[11].y), cv::Point(PoseLandmarks[13].x, PoseLandmarks[13].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[13].x, PoseLandmarks[13].y), cv::Point(PoseLandmarks[15].x, PoseLandmarks[15].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[15].x, PoseLandmarks[15].y), cv::Point(PoseLandmarks[21].x, PoseLandmarks[21].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[15].x, PoseLandmarks[15].y), cv::Point(PoseLandmarks[17].x, PoseLandmarks[17].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[17].x, PoseLandmarks[17].y), cv::Point(PoseLandmarks[19].x, PoseLandmarks[19].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[19].x, PoseLandmarks[19].y), cv::Point(PoseLandmarks[15].x, PoseLandmarks[15].y), color);

    cv::line(frame, cv::Point(PoseLandmarks[12].x, PoseLandmarks[12].y), cv::Point(PoseLandmarks[14].x, PoseLandmarks[14].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[14].x, PoseLandmarks[14].y), cv::Point(PoseLandmarks[16].x, PoseLandmarks[16].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[16].x, PoseLandmarks[16].y), cv::Point(PoseLandmarks[22].x, PoseLandmarks[22].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[16].x, PoseLandmarks[16].y), cv::Point(PoseLandmarks[18].x, PoseLandmarks[18].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[18].x, PoseLandmarks[18].y), cv::Point(PoseLandmarks[20].x, PoseLandmarks[20].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[20].x, PoseLandmarks[20].y), cv::Point(PoseLandmarks[16].x, PoseLandmarks[16].y), color);

    cv::line(frame, cv::Point(PoseLandmarks[12].x, PoseLandmarks[12].y), cv::Point(PoseLandmarks[24].x, PoseLandmarks[24].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[11].x, PoseLandmarks[11].y), cv::Point(PoseLandmarks[23].x, PoseLandmarks[23].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[23].x, PoseLandmarks[23].y), cv::Point(PoseLandmarks[24].x, PoseLandmarks[24].y), color);

    cv::line(frame, cv::Point(PoseLandmarks[24].x, PoseLandmarks[24].y), cv::Point(PoseLandmarks[26].x, PoseLandmarks[26].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[23].x, PoseLandmarks[23].y), cv::Point(PoseLandmarks[25].x, PoseLandmarks[25].y), color);

    cv::line(frame, cv::Point(PoseLandmarks[26].x, PoseLandmarks[26].y), cv::Point(PoseLandmarks[28].x, PoseLandmarks[28].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[25].x, PoseLandmarks[25].y), cv::Point(PoseLandmarks[27].x, PoseLandmarks[27].y), color);

    cv::line(frame, cv::Point(PoseLandmarks[28].x, PoseLandmarks[28].y), cv::Point(PoseLandmarks[30].x, PoseLandmarks[30].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[27].x, PoseLandmarks[27].y), cv::Point(PoseLandmarks[29].x, PoseLandmarks[29].y), color);

    cv::line(frame, cv::Point(PoseLandmarks[30].x, PoseLandmarks[30].y), cv::Point(PoseLandmarks[32].x, PoseLandmarks[32].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[29].x, PoseLandmarks[29].y), cv::Point(PoseLandmarks[31].x, PoseLandmarks[31].y), color);

    cv::line(frame, cv::Point(PoseLandmarks[32].x, PoseLandmarks[32].y), cv::Point(PoseLandmarks[28].x, PoseLandmarks[28].y), color);
    cv::line(frame, cv::Point(PoseLandmarks[31].x, PoseLandmarks[31].y), cv::Point(PoseLandmarks[27].x, PoseLandmarks[27].y), color);


}

absl::Status GoogleMediapipeDetect::PoseTrackingDetect::Mediapipe_RunMPPGraph_Direct(int image_width, int image_height, void* image_data, int* detect_result ,Point2D* PoseLandmarks, Point2D* LeftHandLandmarks, Point2D* RightHandLandmarks, bool& isGestureOK)
{
    m_frameIndex++;
    /*----- 1 ¹¹Ôìcv::Mat¶ÔÏó -----*/
    //cv::Mat camera_frame(cv::Size(image_width, image_height), CV_8UC3, (uchar*)image_data);
    //cv::cvtColor(camera_frame, camera_frame, cv::COLOR_BGR2RGB);
    cv::Mat camera_frame(cv::Size(image_width, image_height), CV_8UC4, (uchar*)image_data);

    //cv::imwrite("C:/mediapipeLibs/Mediapipe_camera_frame_Image.jpg", camera_frame);
    cv::cvtColor(camera_frame, camera_frame, cv::COLOR_RGBA2RGB);

    // cv::Mat bgr_frame;
    // cv::cvtColor(camera_frame, bgr_frame, cv::COLOR_RGB2BGR);
   
    // cv::imwrite("C:/mediapipeLibs/Mediapipe_Color_Image.jpg", bgr_frame);

    // Ë®Æ½·­×ªÊäÈëÍ¼Ïñ
    //cv::flip(camera_frame, camera_frame, 1);
    //std::cout << "cv::Mat¶ÔÏó¹¹½¨Íê³É" << std::endl;

    /*----- 2 ½«OpenCV Mat×ª»»ÎªImageFrame -----*/
    auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
            mediapipe::ImageFormat::SRGB, camera_frame.cols, camera_frame.rows,
            mediapipe::ImageFrame::kDefaultAlignmentBoundary);
    cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
    camera_frame.copyTo(input_frame_mat);
    //std::cout << "½«OpenCV Mat×ª»»ÎªImageFrameÍê³É" << std::endl;

    /*----- 3 ·¢ËÍÍ¼Æ¬µ½Í¼ÖÐÍÆÀí -----*/
    size_t frame_timestamp_us =
            (double)cv::getTickCount() / (double)cv::getTickFrequency() * 1e6;

    MP_RETURN_IF_ERROR(m_Graph.AddPacketToInputStream(
            m_Video_InputStreamName, mediapipe::Adopt(input_frame.release())
                    .At(mediapipe::Timestamp(frame_timestamp_us))));
   // std::cout << "·¢ËÍÍ¼Æ¬µ½Í¼ÖÐÍÆÀíÍê³É" << std::endl;

    /*----- 4 µÃµ½½á¹û -----*/

    // 1 ÊÓÆµÊä³ö½á¹ûÖ¡
    mediapipe::Packet packet;
    if (!m_pVideoPoller->Next(&packet))
    {
        return absl::InvalidArgumentError("no next packet");
    }

    // 2 PoseLandmarks
    mediapipe::Packet poseeLandmarksPacket;
//    int left_arm_result = (int)ArmUpDown::NoResult;
//    int right_arm_result = (int)ArmUpDown::NoResult;
//    int left_arm_leftRight_result = (int)ArmLeftRight::NoLeftRightResult;
//    int right_arm_leftRight_result = (int)ArmLeftRight::NoLeftRightResult;

    int left_arm_waved_updown_result = (int)ArmWaved::NoArmWavedResult;
    int right_arm_waved_updown_result = (int)ArmWaved::NoArmWavedResult;
    int left_arm_waved_leftright_result = (int)ArmWaved::NoArmWavedResult;
    int right_arm_waved_leftright_result = (int)ArmWaved::NoArmWavedResult;

    int left_arm_up_down_result_DIFF;
    int right_arm_up_down_result_DIFF;

    int left_arm_left_right_result_DIFF;
    int right_arm_left_right_result_DIFF;

    Point2D leftHandPoint;
    Point2D rightHandPoint;
   // std::cout << "11111111111111" << std::endl;
    if (m_pPoseLandmarksPoller->QueueSize() != 0)
    {
        //std::cout << "22222222222" << std::endl;
        if (m_pPoseLandmarksPoller->Next(&poseeLandmarksPacket))
        {
            //std::cout << "33333333333333333" << std::endl;
            auto& output_landmarks = poseeLandmarksPacket.Get<mediapipe::NormalizedLandmarkList>();
            //std::cout << "PoseLandmarks size:" << output_landmarks.landmark_size() << std::endl;

            std::vector<Point2D> posePoints;
            posePoints.clear();

            for (int i = 0; i < output_landmarks.landmark_size(); ++i)
            {
                Point2D tempPoint2D;
                const mediapipe::NormalizedLandmark landmark = output_landmarks.landmark(i);
                float x = landmark.x() * camera_frame.cols;
                float y = landmark.y() * camera_frame.rows;
                tempPoint2D.x = x;
                tempPoint2D.y = y;
                PoseLandmarks[i].x = x;
                PoseLandmarks[i].y = y;
                posePoints.emplace_back(tempPoint2D);

            }
            leftHandPoint.x = PoseLandmarks[15].x;
            leftHandPoint.y = PoseLandmarks[15].y;
            rightHandPoint.x = PoseLandmarks[16].x;
            rightHandPoint.y = PoseLandmarks[16].y;

            bool leftGestureOK;
            if(leftHandPoint.x < 0 || leftHandPoint.x > image_width || leftHandPoint.y < 0 || leftHandPoint.y >image_height ||
                    rightHandPoint.x < 0 || rightHandPoint.x > image_width || rightHandPoint.y < 0 || rightHandPoint.y > image_height){
                isGestureOK = false;
            }else {
                if( (leftHandPoint.x > m_hotBoxLeftTopX) && (leftHandPoint.x < m_hotBoxRightBottomY) && (leftHandPoint.y > m_hotBoxLeftTopY) && (leftHandPoint.y < m_hotBoxRightBottomY)){
                    leftGestureOK =  true;
                }else{
                    leftGestureOK =  false;
                }
                bool rightGestureOK;
                if( (rightHandPoint.x > m_hotBoxLeftTopX) && (rightHandPoint.x < m_hotBoxRightBottomY) && (rightHandPoint.y > m_hotBoxLeftTopY) && (rightHandPoint.y < m_hotBoxRightBottomY)){
                    rightGestureOK =  true;
                }else{
                    rightGestureOK =  false;
                }

                isGestureOK = leftGestureOK || rightGestureOK;
            }

            ArmUpAndDownRecognition armUpAndDownRecognition;
            armUpAndDownRecognition.RecognizeProcess(posePoints,
                                                     m_left_arm_current_result,
                                                     m_right_arm_current_result,
                                                     m_left_arm_current_leftRight_result,
                                                     m_right_arm_current_leftRight_result,
                                                     left_arm_up_down_result_DIFF,
                                                     right_arm_up_down_result_DIFF,
                                                     left_arm_left_right_result_DIFF,
                                                     right_arm_left_right_result_DIFF);
            //std::cout << "ÊÖ±ÛÌ§ÊÖ·ÅÊÖÊ¶±ð½á¹û£º" << poseDetectResult << std::endl;
//            m_left_arm_result_list[m_frameIndex%2] = left_arm_result;
//            m_right_arm_result_list[m_frameIndex%2] = right_arm_result;
//            m_left_arm_leftRight_result_list[m_frameIndex%2] = left_arm_leftRight_result;
//            m_right_arm_leftRight_result_list[m_frameIndex%2] = right_arm_leftRight_result;

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

            //std::cout << "44444444444" << std::endl;
        }
    }

    ArmWavedRecognition armWavedRecognition;
    armWavedRecognition.WavedRecognizeProcess(m_left_arm_result_list,m_right_arm_result_list,m_left_arm_leftRight_result_list,m_right_arm_leftRight_result_list,left_arm_waved_updown_result,right_arm_waved_updown_result,left_arm_waved_leftright_result,right_arm_waved_leftright_result);


    detect_result[0] = m_left_arm_current_result;
    detect_result[1] = m_right_arm_current_result;


    //×ó£¬ÓÒ±Û£¬·Å×ó±ß£¬ÓÒ±ß½á¹û
    detect_result[4] = m_left_arm_current_leftRight_result;
    detect_result[5] = m_right_arm_current_leftRight_result;

    //×ó£¬ÓÒ±Û£¬ÉÏÏÂ£¬×óÓÒ»Ó¶¯½á¹û
    detect_result[6] = left_arm_waved_updown_result;
    detect_result[7] = right_arm_waved_updown_result;
    detect_result[8] = left_arm_waved_leftright_result;
    detect_result[9] = right_arm_waved_leftright_result;

     m_left_arm_pre_result = m_left_arm_current_result;
     m_right_arm_pre_result = m_right_arm_current_result;
     m_left_arm_pre_leftRight_result = m_left_arm_current_leftRight_result;
     m_right_arm_pre_leftRight_result = m_right_arm_current_leftRight_result;

//    DrawPoseLandmarks(camera_frame, PoseLandmarks);
//    cv::rectangle(camera_frame, cv::Point(m_hotBoxLeftTopX, m_hotBoxLeftTopY), cv::Point(m_hotBoxRightBottomX, m_hotBoxRightBottomY), cv::Scalar(255, 0, 0), 3, 8, 0);
    return absl::OkStatus();
}

absl::Status GoogleMediapipeDetect::PoseTrackingDetect::Mediapipe_ReleaseGraph()
{
    MP_RETURN_IF_ERROR(m_Graph.CloseInputStream(m_Video_InputStreamName));
    //MP_RETURN_IF_ERROR(m_Graph.CloseInputStream(m_Video_OutputStreamName));
    //MP_RETURN_IF_ERROR(m_Graph.CloseInputStream(m_PoseLandmarks_OutputStreamName));
    //MP_RETURN_IF_ERROR(m_Graph.CloseInputStream(m_LeftHandLandmarks_OutputStreamName));
    //MP_RETURN_IF_ERROR(m_Graph.CloseInputStream(m_RightHandLandmarks_OutputStreamName));
    //MP_RETURN_IF_ERROR(m_Graph.CloseInputStream(m_FaceLandmarks_OutputStreamName));

    return m_Graph.WaitUntilDone();
}