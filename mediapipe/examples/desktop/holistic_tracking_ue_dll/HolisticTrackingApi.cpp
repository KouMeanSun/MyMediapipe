#include "HolisticTrackingApi.h"
#include "HolisticTrackingDetect.h"
#include "TrackingDataStructure.h"

using namespace GoogleMediapipeDetect;

HolisticTrackingDetect m_HolisticTrackingDetect;

EXPORT_API int MediapipeHolisticTrackingInit(const char* model_path)
{
	return m_HolisticTrackingDetect.InitModel(model_path);
}

EXPORT_API int MediapipeHolisticTrackingDetectFrameDirect(int image_width, int image_height, void* image_data, int* detect_result,Point2D* PoseLandmarks, Point2D* LeftHandLandmarks, Point2D* RightHandLandmarks, bool& isGestureOK)
{
	return m_HolisticTrackingDetect.DetectImageDirect(image_width, image_height, image_data, detect_result, PoseLandmarks, LeftHandLandmarks,  RightHandLandmarks, isGestureOK);
}

EXPORT_API int MediapipeHolisticTrackingDetectCamera(bool show_image)
{
	return m_HolisticTrackingDetect.DetectCamera(show_image);
}

EXPORT_API int MediapipeHolisticTrackingRelease()
{
	return m_HolisticTrackingDetect.Release();
}

EXPORT_API int Mediapipe_Hand_Tracking_Box_Register(int hotBoxLeftTopX,int hotBoxLeftTopY,int hotBoxRightBottomX,int hotBoxRightBottomY ,int okGester){
    return m_HolisticTrackingDetect.BoxRegister(hotBoxLeftTopX,hotBoxLeftTopY,hotBoxRightBottomX,hotBoxRightBottomY,okGester);
}
EXPORT_API void MediapipeHolisticTrackingDrawPoseLandmarksTest(int image_width, int image_height, void* image_data,Point2D* PoseLandmarks,const char* writeimageFullPath){
    return m_HolisticTrackingDetect.DrawPoseLandmarksTest(image_width,image_height,image_data,PoseLandmarks,writeimageFullPath);
}