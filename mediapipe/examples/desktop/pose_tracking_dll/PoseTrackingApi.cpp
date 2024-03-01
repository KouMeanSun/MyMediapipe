//
// Created by GMY on 2022/4/1.
//

#include "PoseTrackingApi.h"
#include "PoseTrackingDetect.h"
#include "TrackingDataStructure.h"

using namespace GoogleMediapipeDetect;

PoseTrackingDetect m_PoseTrackingDetect;

EXPORT_API int MediapipePoseTrackingInit(const char* model_path)
{
    return m_PoseTrackingDetect.InitModel(model_path);
}

EXPORT_API int MediapipePoseTrackingDetectFrameDirect(int image_width, int image_height, void* image_data, int* detect_result,Point2D* PoseLandmarks, Point2D* LeftHandLandmarks, Point2D* RightHandLandmarks, bool& isGestureOK)
{
    return m_PoseTrackingDetect.DetectImageDirect(image_width, image_height, image_data, detect_result, PoseLandmarks, LeftHandLandmarks,  RightHandLandmarks, isGestureOK);
}

EXPORT_API int MediapipePoseTrackingRelease()
{
    return m_PoseTrackingDetect.Release();
}

EXPORT_API int Mediapipe_Hand_Tracking_Box_Register(int hotBoxLeftTopX,int hotBoxLeftTopY,int hotBoxRightBottomX,int hotBoxRightBottomY ,int okGester){
    return m_PoseTrackingDetect.BoxRegister(hotBoxLeftTopX,hotBoxLeftTopY,hotBoxRightBottomX,hotBoxRightBottomY,okGester);
}