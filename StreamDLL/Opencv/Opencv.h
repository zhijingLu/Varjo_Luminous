#pragma once

#ifndef _OPENCV_H_
#define _OPENCV_H_

#if defined(_WIN32) //&& defined(MYLIB_DLL)
    #ifdef DLL_EXPORT
    #   define EXPORT __declspec(dllexport)
    #else
    #   define EXPORT __declspec(dllimport)
    #endif
#else
    #define EXPORT
#endif

extern "C" EXPORT float* DetectArucoMarkers(unsigned char* img, int width, int height, int numChannels, int bitsPerChannel, int dictionary);

extern "C" EXPORT float* Triangulate2DPoints(float* coordLeft, float* coordRight, unsigned int size, float* projMatrixLeft, float* projMatrixRight);

extern "C" EXPORT float* Triangulate2DPoints0(float* coordLeft, float* coordRight, unsigned int size);

extern "C" EXPORT float* GetRegions(unsigned char* img, int width, int height, int numChannels, int bitsPerChannel, float H_max, float H_min, float S_max, float S_min, float V_max, float V_min, float min_area, int max_regions, const char* windowName);

extern "C" EXPORT float* GetPixels(unsigned char* img, int width, int height, int numChannels, int bitsPerChannel, float H_max, float H_min, float S_max, float S_min, float V_max, float V_min, float reduction);

extern "C" EXPORT float* DetectFeatures(unsigned char* imgLeft, unsigned char* imgRight, int width, int height, int numChannels, int bitsPerChannel, float H_max, float H_min, float S_max, float S_min, float V_max, float V_min);

extern "C" EXPORT float* FindRope(float* img_left, float* img_right, int width, int height, int numChannels, int bitsPerChannel, 
    float H0_max, float H0_min, float S0_max, float S0_min, float V0_max, float V0_min, 
    float H1_max, float H1_min, float S1_max, float S1_min, float V1_max, float V1_min);

#endif
