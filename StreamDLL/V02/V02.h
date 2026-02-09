#pragma once

#ifndef _V02_H_
#define _V02_H_

#ifdef _WIN32
  #ifdef VARJOSTREAM_EXPORTS
    #define EXPORT __declspec(dllexport)
  #else
    #define EXPORT __declspec(dllimport)
  #endif
#else
  #define EXPORT
#endif

extern "C" EXPORT void StartStreaming();

extern "C" EXPORT void StopStreaming();

extern "C" EXPORT void LockImages();

extern "C" EXPORT void UnlockImages();

extern "C" EXPORT void LockXtrinsics();

extern "C" EXPORT void UnlockXtrinsics();

//extern "C" EXPORT uint8_t* GetLeftImg();

//extern "C" EXPORT uint8_t* GetRightImg();

extern "C" EXPORT float* GetLeftImg();

extern "C" EXPORT float* GetRightImg();

extern "C" EXPORT double* GetLeftExtrinsics();

extern "C" EXPORT double* GetRightExtrinsics();

extern "C" EXPORT double* GetLeftIntrinsics();

extern "C" EXPORT double* GetRightIntrinsics();

extern "C" EXPORT double* GetIntrinsicsUndistorted();

extern "C" EXPORT void SetUndistort(bool activate);

extern "C" EXPORT void SetThresholding(bool activate, float H_max, float H_min, float S_max, float S_min, float V_max, float V_min);

extern "C" EXPORT void SetFindRope(bool activate, float H_max0, float H_min0, float S_max0, float S_min0, float V_max0, float V_min0, float H_max1, float H_min1, float S_max1, float S_min1, float V_max1, float V_min1);

extern "C" EXPORT void LockRope();

extern "C" EXPORT void UnlockRope();

extern "C" EXPORT float* GetRope();

extern "C" EXPORT float* GetPointcloudVertices();

extern "C" EXPORT int* GetPointcloudColors();

extern "C" EXPORT float* GetPointcloudNormals();

extern "C" EXPORT int* GetPointcloudIndices();

#endif
