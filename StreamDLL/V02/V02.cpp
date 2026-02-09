// V02.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <iostream>
#include <vector>
#include "Varjo.h"
#include "Varjo_datastream.h"
#include "UserData.h"
#include "V02.h"
#include <mutex>

#define DEBUGOUT false
//#define CAPTURE_MESH        // whether to activate mesh capture
#define CAPTURE_POINTCLOUD// whether to activate pointcloud capture
#define UNDISTORT             // whether to undistort the original raw images

#if defined(UNDISTORT)
#include <opencv2/opencv.hpp>
#include <opencv2/ccalib/omnidir.hpp>
#include "../Opencv/Opencv.h"
#endif

// needed for thresholding
#include "opencv2/imgproc.hpp"

#if defined(CAPTURE_MESH) || defined(CAPTURE_POINTCLOUD)
#include "Pointcloud.h"
#endif

#include <iostream>
#include <fstream>

UserData myData;
std::mutex mutexImages;
std::mutex mutexXtrinsics;

int clip(int n, int lower, int upper)
{
    return std::max(lower, std::min(n, upper));
}

inline void convertYUVtoRGB(uint8_t Y, uint8_t U, uint8_t V, uint8_t& R, uint8_t& G, uint8_t& B)
{
    const int C = static_cast<int>(Y) - 16;
    const int D = static_cast<int>(U) - 128;
    const int E = static_cast<int>(V) - 128;
    R = static_cast<uint8_t>(clip((298 * C + 409 * E + 128) >> 8, 0, 255));
    G = static_cast<uint8_t>(clip((298 * C - 100 * D - 208 * E + 128) >> 8, 0, 255));
    B = static_cast<uint8_t>(clip((298 * C + 516 * D + 128) >> 8, 0, 255));
}

inline bool convertToR8G8B8A(const varjo_BufferMetadata& buffer, const void* input, void* output, size_t outputRowStride)
{
    constexpr int32_t components = 4;
    if (outputRowStride == 0) {
        outputRowStride = buffer.width * components;
    }

    if (buffer.format != varjo_TextureFormat_NV12) {
        std::cout << "Invalid buffer format!" << std::endl;
    }

    // Convert YUV420 NV12 to RGBA8
    const uint8_t* bY = reinterpret_cast<const uint8_t*>(input);
    const uint8_t* bUV = bY + buffer.rowStride * buffer.height;

    for (int32_t y = 0; y < buffer.height; y++) {
        uint8_t* line = static_cast<uint8_t*>(output) + buffer.width * outputRowStride * y;// * (buffer.height - 1 - y); // flipping image!
        size_t lineOffs = 0;
        for (int32_t x = 0; x < buffer.width; x++) {
            const uint8_t Y = bY[x];

            const auto uvX = x - (x & 1);
            const uint8_t U = bUV[uvX + 0];
            const uint8_t V = bUV[uvX + 1];

            uint8_t R, G, B;
            convertYUVtoRGB(Y, U, V, R, G, B);

            // Write RGBA
            line[lineOffs + 0] = R;
            line[lineOffs + 1] = G;
            line[lineOffs + 2] = B;
            line[lineOffs + 3] = 255;
            lineOffs += components;
        }
        bY += buffer.rowStride;
        if ((y & 1) == 1) {
            bUV += buffer.rowStride;
        }
    }
    return true;
}

inline bool convertToR8G8B8(const varjo_BufferMetadata& buffer, const void* input, void* output, size_t outputRowStride)
{
    constexpr int32_t components = 3;
    if (outputRowStride == 0) {
        outputRowStride = buffer.width * components;
    }

    if (buffer.format != varjo_TextureFormat_NV12) {
        std::cout << "Invalid buffer format!" << std::endl;
    }

    // Convert YUV420 NV12 to RGB8
    const uint8_t* bY = reinterpret_cast<const uint8_t*>(input);
    const uint8_t* bUV = bY + buffer.rowStride * buffer.height;

    for (int32_t y = 0; y < buffer.height; y++) {
        float* line = static_cast<float*>(output) + buffer.width * outputRowStride * y; // * (buffer.height - 1 - y); // flipping image!
        size_t lineOffs = 0;
        for (int32_t x = 0; x < buffer.width; x++) {
            const uint8_t Y = bY[x];

            const auto uvX = x - (x & 1);
            const uint8_t U = bUV[uvX + 0];
            const uint8_t V = bUV[uvX + 1];

            uint8_t R, G, B;
            convertYUVtoRGB(Y, U, V, R, G, B);

            // Write RGB in [0,1]
            line[lineOffs + 0] = R / 255.0f;
            line[lineOffs + 1] = G / 255.0f;
            line[lineOffs + 2] = B / 255.0f;
            lineOffs += components;
        }
        bY += buffer.rowStride;
        if ((y & 1) == 1) {
            bUV += buffer.rowStride;
        }
    }
    return true;
}


#if defined(UNDISTORT)
inline void undistort(uint8_t* img, double* extr, double* intr)
{
    cv::Size inputSize(1152, 1152);
    double principalPointX = intr[0] * inputSize.width;
	double principalPointY = intr[1] * inputSize.height;
	double focalLengthX = intr[2] * inputSize.width;
	double focalLengthY = intr[3] * inputSize.height;
	double d0 = intr[4];
	double d1 = intr[5];
	double d2 = intr[6];
	double d3 = intr[7];
	double d4 = intr[8];
    double d5 = intr[9];

	float k[3][3] = {
        {focalLengthX, focalLengthX * d2, principalPointX},
        {0, focalLengthY, principalPointY},
        {0, 0, 1}
	};

	float d[4] = { d0, d1, d4, d5 };

	double rotationMatrix[3][3] =
	{
		{extr[0], extr[4], extr[8]},
		{extr[1], extr[5], extr[9]},
		{extr[2], extr[6], extr[10]}
	};

	cv::Mat K = cv::Mat(3, 3, CV_32F, k);
	cv::Mat D = cv::Mat(4, 1, CV_32F, d);
	cv::Mat R = cv::Mat(3, 3, CV_64F, rotationMatrix);
    cv::Mat Knew = cv::Mat(3, 3, CV_32F, myData.knew);

    cv::Mat distortedImage(1152, 1152, CV_8UC4, img);
    cv::Mat undistortedImage = cv::Mat();

    // undistort!
    cv::omnidir::undistortImage(distortedImage, undistortedImage, K, D, d3, cv::omnidir::RECTIFY_PERSPECTIVE, Knew, cv::Size(myData.outputSizeWidth, myData.outputSizeHeight), R);
    
    // update image data (Notice: over-writing!)
    undistortedImage.copyTo(distortedImage);
}
inline void undistort(float* img, double* extr, double* intr)
{
    cv::Size inputSize(1152, 1152);
    double principalPointX = intr[0] * inputSize.width;
    double principalPointY = intr[1] * inputSize.height;
    double focalLengthX = intr[2] * inputSize.width;
    double focalLengthY = intr[3] * inputSize.height;
    double d0 = intr[4];
    double d1 = intr[5];
    double d2 = intr[6];
    double d3 = intr[7];
    double d4 = intr[8];
    double d5 = intr[9];

    float k[3][3] = {
        {focalLengthX, focalLengthX * d2, principalPointX},
        {0, focalLengthY, principalPointY},
        {0, 0, 1}
    };

    float d[4] = { d0, d1, d4, d5 };

    double rotationMatrix[3][3] =
    {
        {extr[0], extr[4], extr[8]},
        {extr[1], extr[5], extr[9]},
        {extr[2], extr[6], extr[10]}
    };

    cv::Mat K = cv::Mat(3, 3, CV_32F, k);
    cv::Mat D = cv::Mat(4, 1, CV_32F, d);
    cv::Mat R = cv::Mat(3, 3, CV_64F, rotationMatrix);
    cv::Mat Knew = cv::Mat(3, 3, CV_32F, myData.knew);

    cv::Mat distortedImage(1152, 1152, CV_32FC3, img); // CV_8UC4
    cv::Mat undistortedImage = cv::Mat();

    // undistort!
    cv::omnidir::undistortImage(distortedImage, undistortedImage, K, D, d3, cv::omnidir::RECTIFY_PERSPECTIVE, Knew, cv::Size(myData.outputSizeWidth, myData.outputSizeHeight), R);

    // update image data (Notice: over-writing!)
    undistortedImage.copyTo(distortedImage);
}
#endif

void writeToFile(std::vector<cv::Rect> rects, const char* filename)
{
    std::ofstream fout(filename, std::ios_base::app);

    if (!fout)
    {
        std::cout << "File Not Opened" << std::endl;
        return;
    }

    for (int i = 0; i < rects.size(); i++)
    {
        fout << i << ") " << rects[i].x << " " << rects[i].y << " " << rects[i].width << " " << rects[i].height << " " << rects[i].area() << std::endl;
    }

    fout.close();
}

void threshold(uint8_t* img)
{
    cv::Mat image(1152, 1152, CV_8UC4, img);
    cv::Mat imageHSV = cv::Mat();
    cv::Mat mask = cv::Mat(), maskInv = cv::Mat(), maskC4 = cv::Mat();

    // Convert from BGR to HSV colorspace
    cv::cvtColor(image, imageHSV, cv::COLOR_RGB2HSV); //cv::COLOR_BGR2HSV);

    // Apply thresholding
    cv::inRange(imageHSV, cv::Scalar(myData.H_min0, myData.S_min0, myData.V_min0), cv::Scalar(myData.H_max0, myData.S_max0, myData.V_max0), mask);

    // Fill-in the colors
    cv::bitwise_not(mask, maskInv);
    image.setTo(cv::Scalar(0, 0, 0), maskInv);
}

void onDataStreamFrame(const varjo_StreamFrame* frame, varjo_Session* session, void* userData)
{
    UserData* uData = (UserData*)userData;

    // Get frame world pose
    const varjo_Matrix& pose = frame->hmdPose;

    if (frame->type != varjo_StreamType_DistortedColor) {
        std::cerr << "Invalid frame type!" << std::endl;
        return;
    }

    // Get the metadata specific for DistortedColor stream type
    const varjo_DistortedColorFrameMetadata& frameMetadata = frame->metadata.distortedColor;
    // std::cout << "DCFM timestamp = " << frameMetadata.timestamp << std::endl;

    int eye = 0;
    for (varjo_ChannelIndex channelIndex : {varjo_ChannelIndex_Left, varjo_ChannelIndex_Right}) {
#if(DEBUGOUT)
        std::cout << "Frame channels = " << frame->channels << " --- Expected [3]\n";
#endif
        if (!(frame->channels & (1ull << channelIndex))) {
            continue;
        }

        // Get camera extrinsics if available
        if (frame->dataFlags & varjo_DataFlag_Extrinsics) {
            varjo_Matrix extrinsics = varjo_GetCameraExtrinsics(session, frame->id, frame->frameNumber, channelIndex);

            mutexXtrinsics.lock();
            if (eye % 2) {
                for (int i = 0; i < 16; i++)
                    myData.extrinsics_r[i] = extrinsics.value[i];
            }
            else {
                for (int i = 0; i < 16; i++)
                    myData.extrinsics_l[i] = extrinsics.value[i];
            }
            mutexXtrinsics.unlock();

        }

        // Get camera intrinsics if available
        if (frame->dataFlags & varjo_DataFlag_Intrinsics) {
            varjo_CameraIntrinsics intrinsics = varjo_GetCameraIntrinsics(session, frame->id, frame->frameNumber, channelIndex);

            mutexXtrinsics.lock();
            if (eye % 2) {
                myData.intrinsics_r[0] = intrinsics.principalPointX;
                myData.intrinsics_r[1] = intrinsics.principalPointY;
                myData.intrinsics_r[2] = intrinsics.focalLengthX;
                myData.intrinsics_r[3] = intrinsics.focalLengthY;
                for (int i = 0; i <= 5; i++)
                    myData.intrinsics_r[i + 4] = intrinsics.distortionCoefficients[i];
            }
            else {
                myData.intrinsics_l[0] = intrinsics.principalPointX;
                myData.intrinsics_l[1] = intrinsics.principalPointY;
                myData.intrinsics_l[2] = intrinsics.focalLengthX;
                myData.intrinsics_l[3] = intrinsics.focalLengthY;
                for (int i = 0; i <= 5; i++)
                    myData.intrinsics_l[i + 4] = intrinsics.distortionCoefficients[i];
            }
            mutexXtrinsics.unlock();
        }

        // Get buffer ID for the channel, if available
        if (frame->dataFlags & varjo_DataFlag_Buffer) {
            varjo_BufferId bufferId = varjo_GetBufferId(session, frame->id, frame->frameNumber, channelIndex);
            // Lock buffer
            varjo_LockDataStreamBuffer(session, bufferId);

            // Get buffer metadata
            varjo_BufferMetadata meta = varjo_GetBufferMetadata(session, bufferId);

            // Get buffer contents
            if (meta.type == varjo_BufferType_CPU) {
                void* cpuData = varjo_GetBufferCPUData(session, bufferId);

                mutexXtrinsics.lock();
                {
                    int backBufferIdx = 1 - myData.frontBufferIdx;

                    if (eye % 2) {
                        //convertToR8G8B8A(meta, cpuData, myData.imgRight[backBufferIdx], 4);
                        convertToR8G8B8(meta, cpuData, myData.imgRight[backBufferIdx], 3);

#if defined(UNDISTORT)
                        if (myData.undistort)
                        {
                            // Undistort right image
                            undistort(myData.imgRight[backBufferIdx], myData.extrinsics_r, myData.intrinsics_r);
                        }

                        /*if (myData.threshold)
                        {
                            cv::Mat imageo(1152, 1152, CV_8UC4, myData.imgRight[backBufferIdx]);
                            cv::imwrite("C:\\Users\\robertini\\Documents\\coding\\GreifbAR_Unity_DFKI\\Output\\right.png", imageo);

                            threshold(myData.imgRight[backBufferIdx]);

                            cv::Mat image(1152, 1152, CV_8UC4, myData.imgRight[backBufferIdx]);
                            cv::imwrite("C:\\Users\\robertini\\Documents\\coding\\GreifbAR_Unity_DFKI\\Output\\right_t.png", image);
                        }*/
#endif
                    }
                    else
                    {
                        //convertToR8G8B8A(meta, cpuData, myData.imgLeft[backBufferIdx], 4);
                        convertToR8G8B8(meta, cpuData, myData.imgLeft[backBufferIdx], 3);

#if defined(UNDISTORT)
                        if (myData.undistort)
                        {
                            // Undistort left image
                            undistort(myData.imgLeft[backBufferIdx], myData.extrinsics_l, myData.intrinsics_l);
                        }

                        /*if (myData.threshold)
                        {
                            cv::Mat imageo(1152, 1152, CV_8UC4, myData.imgLeft[backBufferIdx]);
                            cv::imwrite("C:\\Users\\robertini\\Documents\\coding\\GreifbAR_Unity_DFKI\\Output\\left.png", imageo);

                            threshold(myData.imgLeft[backBufferIdx]);

                            cv::Mat image(1152, 1152, CV_8UC4, myData.imgLeft[backBufferIdx]);
                            cv::imwrite("C:\\Users\\robertini\\Documents\\coding\\GreifbAR_Unity_DFKI\\Output\\left_t.png", image);
                        }*/
#endif
                    }
                }
                mutexXtrinsics.unlock();
                eye++;
            }

            // Unlock buffer
            varjo_UnlockDataStreamBuffer(session, bufferId);
        }
    
    }

    {
        mutexImages.lock();

        // flip buffer in secure mode
        myData.frontBufferIdx = 1 - myData.frontBufferIdx;

        mutexImages.unlock();
    }
}

void dataStreamFrameCallback(const varjo_StreamFrame* frame, varjo_Session* session, void* userData)
{
    // "C" trampoline callback function to get back to "C++"

    UserData* uData = (UserData*)userData;
    if (uData->stop)
        return;

    // checking callback and availability of userData
    onDataStreamFrame(frame, session, uData);

    //uData->stop = true;

#if defined(CAPTURE_MESH)
    onMeshStreamFrame(session, uData);
#elif defined(CAPTURE_POINTCLOUD)
    onPointcloudStreamFrame(session, uData);
#endif
}

extern "C" EXPORT void StartStreaming()
{
    varjo_Session* m_session = varjo_SessionInit();
    std::vector<varjo_StreamConfig> configs;

    configs.resize(varjo_GetDataStreamConfigCount(m_session));
    varjo_Error err = varjo_GetError(m_session);
#if(DEBUGOUT)
    std::cout << "Error =" << varjo_GetErrorDesc(err) << "\n";
#endif
    varjo_GetDataStreamConfigs(m_session, configs.data(), static_cast<int32_t>(configs.size()));
    err = varjo_GetError(m_session);
#if(DEBUGOUT)
    std::cout << "Error =" << varjo_GetErrorDesc(err) << "\n";
#endif

    varjo_StreamConfig conf = configs[varjo_StreamType_DistortedColor];
#if(DEBUGOUT)
    std::cout << "streamID =" << conf.streamId << "\n";
#endif

    myData.text = "Hurray!";
    myData.stop = false;
    myData.m_session = m_session;
    myData.conf = conf;

    varjo_StartDataStream(m_session, conf.streamId, varjo_ChannelFlag_Left | varjo_ChannelFlag_Right, dataStreamFrameCallback, (void*)(&myData));
    err = varjo_GetError(m_session);

    std::cout << "Error =" << varjo_GetErrorDesc(err) << "\n";

#if defined(CAPTURE_MESH)
	// Additionally retrieve the mesh data:
	varjo_MRSetMeshReconstruction(m_session, true); // enable mesh reconstruction
	checkVarjoError(m_session);

	StartStreamingMesh(m_session);
#else
	//varjo_MRSetMeshReconstruction(m_session, false); // disable mesh reconstruction
	//checkVarjoError(m_session);
#endif


#if defined(CAPTURE_POINTCLOUD)
	varjo_MRSetReconstruction(m_session, true); // enable pointcloud
	checkVarjoError(m_session);

	StartStreamingPointcloud(m_session);
#else
	//varjo_MRSetReconstruction(m_session, false); // disable pointcloud
	//checkVarjoError(m_session);
#endif

}

extern "C" EXPORT void StopStreaming() {
    varjo_StopDataStream(myData.m_session, myData.conf.streamId);

    if (myData.find_rope)
    {
        myData.find_rope = false;
        myData.RopeWorker.join();
    }
}

extern "C" EXPORT void LockImages() {
    mutexImages.lock();
}

extern "C" EXPORT void UnlockImages() {
    mutexImages.unlock();
}

extern "C" EXPORT void LockXtrinsics() {
    mutexXtrinsics.lock();
}

extern "C" EXPORT void UnlockXtrinsics() {
    mutexXtrinsics.unlock();
}

extern "C" EXPORT float* GetLeftImg() {
    // Make sure to call this function between LockImages() and UnlockImages()
    return myData.imgLeft[myData.frontBufferIdx];
}

extern "C" EXPORT float* GetRightImg() {
    // Make sure to call this function between LockImages() and UnlockImages()
    return myData.imgRight[myData.frontBufferIdx];
}

extern "C" EXPORT double* GetLeftExtrinsics() {
    // Make sure to call this function between LockXtrinsics() and UnlockXtrinsics()
    return myData.extrinsics_l;
}

std::mutex mutexRope;

void RopeTrackerWorker()
{
    while (myData.find_rope)
    {
        LockImages();
        std::memcpy(myData.left, myData.imgLeft[myData.frontBufferIdx], 1152 * 1152 * 3 * sizeof(float));
        std::memcpy(myData.right, myData.imgRight[myData.frontBufferIdx], 1152 * 1152 * 3 * sizeof(float));
        UnlockImages();

        // process the rope!
        float* result = FindRope(myData.left, myData.right, 1152, 1152, 3, 8, myData.H_max0, myData.H_min0, myData.S_max0, myData.S_min0, myData.V_max0, myData.V_min0,
            myData.H_max1, myData.H_min1, myData.S_max1, myData.S_min1, myData.V_max1, myData.V_min1);

        const int size = result[0] + result[1] + 1;
        const int minsize = std::min(100, size) * sizeof(float);

        LockRope();
        std::memcpy(myData.RopeData, result, minsize);
        UnlockRope();
    }
}

extern "C" EXPORT void LockRope() {
    mutexRope.lock();
}

extern "C" EXPORT void UnlockRope() {
    mutexRope.unlock();
}

extern "C" EXPORT float* GetRope()
{
    // Make sure to call this function between LockRope() and UnlockRope()
    return myData.RopeData;
}

extern "C" EXPORT void SetFindRope(bool activate, float H_max0, float H_min0, float S_max0, float S_min0, float V_max0, float V_min0, float H_max1, float H_min1, float S_max1, float S_min1, float V_max1, float V_min1)
{
    myData.find_rope = activate;
    myData.H_max0 = H_max0;
    myData.H_min0 = H_min0;
    myData.S_max0 = S_max0;
    myData.S_min0 = S_min0;
    myData.V_max0 = V_max0;
    myData.V_min0 = V_min0;

    myData.H_max1 = H_max1;
    myData.H_min1 = H_min1;
    myData.S_max1 = S_max1;
    myData.S_min1 = S_min1;
    myData.V_max1 = V_max1;
    myData.V_min1 = V_min1;

    if (myData.find_rope)
    {
        // start the thread
        myData.RopeWorker = std::thread(RopeTrackerWorker);
    }
    else
    {
        // stop the thread
        myData.RopeWorker.join(); // it will exit automatically once it checks the myData.find_rope value
    }
}

extern "C" EXPORT double* GetRightExtrinsics() {
    return myData.extrinsics_r;
}

extern "C" EXPORT double* GetLeftIntrinsics() {
    return myData.intrinsics_l;
}

extern "C" EXPORT double* GetRightIntrinsics() {
    return myData.intrinsics_r;
}

extern "C" EXPORT double* GetIntrinsicsUndistorted() {
    return myData.intrinsics_undistorted;
}

extern "C" EXPORT void SetUndistort(bool activate)
{
    myData.undistort = activate;
}

