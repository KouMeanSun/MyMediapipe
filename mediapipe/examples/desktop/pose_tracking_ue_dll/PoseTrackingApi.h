//
// Created by GMY on 2022/4/1.
//

#ifndef MEDIAPIPE_MASTER_POSETRACKINGAPI_H
#define MEDIAPIPE_MASTER_POSETRACKINGAPI_H



#define EXPORT

/* ���嶯̬���ӿ�dll�ĵ��� */
#include <malloc.h>
#ifdef _WIN32
#ifdef EXPORT
#define EXPORT_API __declspec(dllexport)
#else
#define EXPORT_API __declspec(dllimport)
#endif
#else
#include <stdlib.h>

#ifdef EXPORT
#define EXPORT_API __attribute__((visibility ("default")))
#else
#endif

#endif


#ifdef __cplusplus
extern "C" {
#endif

#ifndef EXPORT_API
#define EXPORT_API
#endif


struct Point2D;

/*
@brief ��ʼ��Google Mediapipe
@param[in] model_path ��Ҫ���ص�ģ��·��
@return ���ز����ɹ�����ʧ��
    0 ʧ��
    1 �ɹ�
*/
EXPORT_API int MediapipePoseTrackingInit(const char* model_path);

/*
@brief �����Ƶ֡
@param[in] image_width ��Ƶ֡���
@param[in] image_height ��Ƶ֡�߶�
@param[in] image_data ��Ƶ֡����
@param[in] show_result_image �Ƿ���ʾ���ͼƬ
@param[out] gesture_result - ����ʶ����
@return ���ز����ɹ�����ʧ��
    0 ʧ��
    1 �ɹ�
*/
EXPORT_API int MediapipePoseTrackingDetectFrameDirect(int image_width, int image_height, void* image_data, int* detect_result, Point2D* PoseLandmarks, Point2D* LeftHandLandmarks, Point2D* RightHandLandmarks, bool& isGestureOK);

/*
@brief Google Mediapipe�ͷ�
@return ���ز����ɹ�����ʧ��
    0 ʧ��
    1 �ɹ�
*/
EXPORT_API int MediapipePoseTrackingRelease();

EXPORT_API int Mediapipe_Hand_Tracking_Box_Register(int hotBoxLeftTopX,int hotBoxLeftTopY,int hotBoxRightBottomX,int hotBoxRightBottomY ,int okGester);


#ifdef __cplusplus
}
#endif

#endif //MEDIAPIPE_MASTER_POSETRACKINGAPI_H
