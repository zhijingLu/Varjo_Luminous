#pragma once

#include "Varjo.h"
#include "Varjo_datastream.h"
#include <thread>

struct UserData {
    const char* text = "My Data!";
    bool stop = false;
    bool undistort = false; // whether to undistort the images
    bool threshold = false; // whether to threshold the images
    bool find_rope = false; // whether to locate the rope (using the thresholding parameters)

    //uint8_t* imgLeft[2];
    //uint8_t* imgRight[2];
    float* imgLeft[2];
    float* imgRight[2];
    int frontBufferIdx;
    varjo_Session* m_session;
    varjo_StreamConfig conf;
    double* extrinsics_l;
    double* intrinsics_l;
    double* extrinsics_r;
    double* intrinsics_r;

    // image copies for the rope finder
    float* left;
    float* right;
    float* RopeData;
    std::thread RopeWorker;

    // thresholding parameters
    float H_max0 = 180, H_min0 = 0, V_max0 = 255, V_min0 = 0, S_max0 = 255, S_min0 = 0;
    float H_max1 = 180, H_min1 = 0, V_max1 = 255, V_min1 = 0, S_max1 = 255, S_min1 = 0;

    // undistortion parameters
    double* intrinsics_undistorted;
    int outputSizeWidth = 1152, outputSizeHeight = 1152;
    float knew[3][3] =
    {
        {outputSizeWidth / 1.6f, 0, outputSizeWidth / 2.0f},
        {0, outputSizeHeight / 1.6f, outputSizeHeight / 2.0f},
        {0, 0, 1}
    };

    UserData() {
        m_session = nullptr;
        frontBufferIdx = 0;

        imgLeft[0] = new float[1152 * 1152 * 3];
        imgLeft[1] = new float[1152 * 1152 * 3];
        imgRight[0] = new float[1152 * 1152 * 3];
        imgRight[1] = new float[1152 * 1152 * 3];

        left = new float[1152 * 1152 * 3];
        right = new float[1152 * 1152 * 3];
        extrinsics_l = new double[16];
        intrinsics_l = new double[10];
        extrinsics_r = new double[16];
        intrinsics_r = new double[10];
        intrinsics_undistorted = new double[10];
        
        RopeData = new float[100];
        RopeData[0] = RopeData[1] = 0;

        // update intrinsics undistorted (they are the same for the left and right view!)
        intrinsics_undistorted[0] = knew[0][2] / outputSizeWidth;
        intrinsics_undistorted[1] = knew[1][2] / outputSizeHeight;
        intrinsics_undistorted[2] = knew[0][0] / outputSizeWidth;
        intrinsics_undistorted[3] = knew[1][1] / outputSizeHeight;
        intrinsics_undistorted[4] = intrinsics_undistorted[5] = intrinsics_undistorted[6] = intrinsics_undistorted[7] = intrinsics_undistorted[8] = intrinsics_undistorted[9] = 0;
    }

    ~UserData() {
        delete[] imgLeft[0];
        delete[] imgLeft[1];
        delete[] imgRight[0];
        delete[] imgRight[1];
        delete[] extrinsics_l;
        delete[] intrinsics_l;
        delete[] extrinsics_r;
        delete[] intrinsics_r;
        delete[] intrinsics_undistorted;
    }
};

extern UserData myData;