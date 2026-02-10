
#ifndef NOMINMAX
#define NOMINMAX
#endif

#define OPENCV_CONTRIB

#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <iterator>
#include <opencv2/opencv.hpp>
#ifdef OPENCV_CONTRIB
    #include <opencv2/ccalib/omnidir.hpp>
    #include <opencv2/aruco.hpp>
#endif
#include <opencv2/flann.hpp>
#include "Opencv.h"

#define DEBUGOUT true

cv::Mat createMat(unsigned char* img, int width, int height, int numChannels, int bitsPerChannel, bool convertToGrayscale = false)
{  
    cv::Mat image;
    cv::Mat gray;

    switch (numChannels) {
    case 1: // Grayscale
        switch (bitsPerChannel) {
        case 8:
            image = cv::Mat(height, width, CV_8UC1, img);
            gray = image;
            break;
        case 16:
            image = cv::Mat(height, width, CV_16UC1, img);
            gray = image;
            break;
        default:
            // Handle unsupported bits per channel
            break;
        }
        break;

    case 3: // RGB color
        switch (bitsPerChannel) {
        case 8:
            image = cv::Mat(height, width, CV_8UC3, img);
            if (convertToGrayscale)
                cv::cvtColor(image, gray, cv::ColorConversionCodes::COLOR_RGB2GRAY);
            break;
        case 16:
            image = cv::Mat(height, width, CV_16UC3, img);
            if (convertToGrayscale)
                cv::cvtColor(image, gray, cv::ColorConversionCodes::COLOR_RGB2GRAY);
            break;
        default:
            // Handle unsupported bits per channel
            break;
        }
        break;

    case 4: // RGBA color
        switch (bitsPerChannel) {
        case 8:
            image = cv::Mat(height, width, CV_8UC4, img);
            if (convertToGrayscale)
                cv::cvtColor(image, gray, cv::ColorConversionCodes::COLOR_RGBA2GRAY);
            break;
        case 16:
            image = cv::Mat(height, width, CV_16UC4, img);
            if (convertToGrayscale)
                cv::cvtColor(image, gray, cv::ColorConversionCodes::COLOR_RGBA2GRAY);
            break;
        default:
            // Handle unsupported bits per channel
            break;
        }
        break;

    default:
        // Handle unsupported number of channels
        break;
    }

    if (convertToGrayscale)
        return gray;
    return image;
}

cv::Mat createMat(float* img, int width, int height, int numChannels, bool convertToGrayscale = false)
{
    cv::Mat image;
    cv::Mat gray;

    switch (numChannels) {
    case 1: // Grayscale
        image = cv::Mat(height, width, CV_32FC1, img);
        gray = image;
        break;

    case 3: // RGB color
        image = cv::Mat(height, width, CV_32FC3, img);
        if (convertToGrayscale)
            cv::cvtColor(image, gray, cv::ColorConversionCodes::COLOR_RGB2GRAY);
        break;

    case 4: // RGBA color
        image = cv::Mat(height, width, CV_32FC4, img);
        if (convertToGrayscale)
            cv::cvtColor(image, gray, cv::ColorConversionCodes::COLOR_RGB2GRAY);
        break;

    default:
        // Handle unsupported number of channels
        break;
    }

    if (convertToGrayscale)
        return gray;
    return image;
}


void writeToFile(cv::Mat& m, const char* filename)
{
    std::ofstream fout(filename, std::ios_base::app);

    if (!fout)
    {
        std::cout << "File Not Opened" << std::endl;
        return;
    }

    for (int i = 0; i < m.rows; i++)
    {
        for (int j = 0; j < m.cols; j++)
        {
            fout << m.at<float>(i, j) << "\t";
        }
        fout << std::endl;
    }

    fout.close();
}

void writeToFile(std::vector<cv::Vec2d> points, const char* filename)
{
    std::ofstream fout(filename, std::ios_base::app);

    if (!fout)
    {
        std::cout << "File Not Opened" << std::endl;
        return;
    }

    for (int i = 0; i < points.size(); i++)
    {
        fout << points[i][0] << " " << points[i][1] << std::endl;
    }

    fout.close();
}

void writeToFile(int value, const char* filename)
{
    std::ofstream fout(filename, std::ios_base::app);

    if (!fout)
    {
        std::cout << "File Not Opened" << std::endl;
        return;
    }

    fout << "value is " << value << std::endl;

    fout.close();
}

void writeToFile(std::vector<cv::Point2i> points, const char* filename)
{
    std::ofstream fout(filename, std::ios_base::app);

    if (!fout)
    {
        std::cout << "File Not Opened" << std::endl;
        return;
    }

    for (int i = 0; i < points.size(); i++)
    {
        fout << points[i].x << " " << points[i].y << std::endl;
    }

    fout << "----------" << std::endl;

    fout.close();
}

void writeToFile(std::vector<cv::Vec3d> points, const char* filename)
{
    std::ofstream fout(filename, std::ios_base::app);

    if (!fout)
    {
        std::cout << "File Not Opened" << std::endl;
        return;
    }

    for (int i = 0; i < points.size(); i++)
    {
        fout << points[i][0] << " " << points[i][1] << " " << points[i][2] << std::endl;
    }

    fout.close();
}

void writeToFile(std::vector<cv::Vec4d> points, const char* filename)
{
    std::ofstream fout(filename, std::ios_base::app);

    if (!fout)
    {
        std::cout << "File Not Opened" << std::endl;
        return;
    }

    for (int i = 0; i < points.size(); i++)
    {
        fout << points[i][0] << " " << points[i][1] << " " << points[i][2] << " " << points[i][3] << std::endl;
    }

    fout.close();
}

void writeToFile(const char* message, const char* filename)
{
    std::ofstream fout(filename, std::ios_base::app);

    if (!fout)
    {
        std::cout << "File Not Opened" << std::endl;
        return;
    }

    fout << message << std::endl;

    fout.close();
}

float* convertToPointer(std::vector<cv::Vec3d> points)
{
    const int size = points.size();
    float* result = new float[size * 3];

    for (int i = 0; i < size; ++i)
    {
        result[i * 3 + 0] = points[i][0];
        result[i * 3 + 1] = points[i][1];
        result[i * 3 + 2] = points[i][2];
    }

    return result;
}

float* convertToPointer(std::vector<cv::Point3f> points)
{
    const int size = points.size();
    float* result = new float[size * 3];

    for (int i = 0; i < size; ++i)
    {
        result[i * 3 + 0] = points[i].x;
        result[i * 3 + 1] = points[i].y;
        result[i * 3 + 2] = points[i].z;
    }

    return result;
}

float* convertToPointer(cv::Mat& mat)
{
    float* result = new float[mat.rows * mat.cols];

    for (int i = 0; i < mat.cols; ++i)
    {
        for (int j = 0; j < mat.rows; ++j)
        {
            result[i * mat.rows + j] = mat.at<float>(j, i);
        }
    }

    return result;
}

std::vector<cv::Vec2d> convertToVector(float* points, int size)
{
    std::vector<cv::Vec2d> result(size);

    for (int i = 0; i < size; ++i)
    {
        result[i][0] = points[i * 2 + 0];
        result[i][1] = points[i * 2 + 1];
    }

    return result;
}

#ifdef OPENCV_CONTRIB
/// <summary>
/// Detect Aruco Markers,
/// </summary>
/// <param name="img">Input image data pointer</param>
/// <param name="width">Image width</param>
/// <param name="height">Image height</param>
/// <param name="numChannels">Number of channels, e.g. 3 for RGB, 4 for RGBA</param>
/// <param name="bitsPerChannel">Bits per channel, e.g. for RGB typical is 8 bits (in [0,255])</param>
extern "C" EXPORT float* DetectArucoMarkers(unsigned char* img, int width, int height, int numChannels, int bitsPerChannel, int dictionary)
{
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    try
    {
        // Transform input to OpenCV readable data
        cv::Mat inputImage = createMat(img, width, height, numChannels, bitsPerChannel, true);

        // Choose Aruco dictionary
        int dict = cv::aruco::DICT_5X5_250; // default dictionary
        if (0 <= dictionary && dictionary < 21) // 21 is the current number of available Aruco Dictionaries
            dict = dictionary;
        cv::Ptr<cv::aruco::Dictionary> arucoDictionary = cv::Ptr<cv::aruco::Dictionary>(new cv::aruco::Dictionary(cv::aruco::getPredefinedDictionary(dict))); // DICT_5X5_1000, DICT_5X5_250

        // Detect Aruco markers
        cv::aruco::detectMarkers(inputImage, arucoDictionary, markerCorners, markerIds);
    }
    catch (cv::Exception& e)
    {
#if(DEBUGOUT)
        std::cout << "Error while detecting Aruco Markers: " << e.what() << "\n";
#endif

        return nullptr;
    }

    // Create output result in the fashion: total marker detected; id, corner A, corner B, corner C, corner D; id, ...
    float* result = new float[1 + markerIds.size() * 9];
    result[0] = markerIds.size();

    for (int i = 0; i < markerIds.size(); ++i)
    {
        result[1 + i * 9] = markerIds[i];

        result[1 + i * 9 + 1] = markerCorners[i][0].x;
        result[1 + i * 9 + 2] = markerCorners[i][0].y;

        result[1 + i * 9 + 3] = markerCorners[i][1].x;
        result[1 + i * 9 + 4] = markerCorners[i][1].y;

        result[1 + i * 9 + 5] = markerCorners[i][2].x;
        result[1 + i * 9 + 6] = markerCorners[i][2].y;

        result[1 + i * 9 + 7] = markerCorners[i][3].x;
        result[1 + i * 9 + 8] = markerCorners[i][3].y;
    }

    return result;
}
#endif

float* GetPixels(unsigned char* img, int width, int height, int numChannels, int bitsPerChannel, float H_max, float H_min, float S_max, float S_min, float V_max, float V_min, float reduction)
{
    // Transform input to OpenCV readable data
    cv::Mat image = createMat(img, width, height, numChannels, bitsPerChannel);

    // Convert from RGB to HSV colorspace
    cv::Mat imageHSV = cv::Mat();
    cv::cvtColor(image, imageHSV, cv::COLOR_RGB2HSV); //cv::COLOR_BGR2HSV);

    // Reduce image size
    cv::Mat imageHSVSmall = cv::Mat();
    cv::resize(imageHSV, imageHSVSmall, cv::Size(), reduction, reduction);

    // Apply thresholding and find non-zero locations
    cv::Mat mask = cv::Mat();
    cv::inRange(imageHSVSmall, cv::Scalar(H_min, S_min, V_min), cv::Scalar(H_max, S_max, V_max), mask);
    std::vector<cv::Point2i> locations;
    cv::findNonZero(mask, locations);

    // Debug
    //writeToFile(locations, "C:\\Users\\robertini\\Documents\\coding\\GreifbAR_Unity_DFKI\\Output\\locations.txt");
    //writeToFile(locations.size(), "C:\\Users\\robertini\\Documents\\coding\\GreifbAR_Unity_DFKI\\Output\\locations.txt");
    //cv::imwrite("C:\\Users\\robertini\\Documents\\coding\\GreifbAR_Unity_DFKI\\Output\\mask.png", mask);
    //

    // Convert to floating points output
    float* result = new float[1 + locations.size() * 6];
    result[0] = locations.size();
    float invReduction = 1.0f / reduction;

    for (int i = 0; i < locations.size(); ++i)
    {
        result[1 + i * 6 + 0] = locations[i].x * invReduction;
        result[1 + i * 6 + 1] = locations[i].y * invReduction;
        result[1 + i * 6 + 2] = invReduction;
        cv::Vec3b color = imageHSVSmall.at<cv::Vec3b>(locations[i]);
        result[1 + i * 6 + 3] = color[0];
        result[1 + i * 6 + 4] = color[1];
        result[1 + i * 6 + 5] = color[2];
    }

    return result;
}

inline cv::Point2f NextPoint(cv::RotatedRect rect, cv::Point2f prevPoint, bool opposite = false, bool checkDirection = false)
{
    cv::Point2f points[4];
    rect.points(points);

    // get the longer side
    const float dist01 = cv::norm(points[1] - points[0]);
    const float dist12 = cv::norm(points[2] - points[1]);

    cv::Point2f p0, p1;
    if (dist01 > dist12)
    {
        p0 = points[1] + (points[2] - points[1]) / 2;
        p1 = points[0] + (points[3] - points[0]) / 2;
    }
    else
    {
        p0 = points[0] + (points[1] - points[0]) / 2;
        p1 = points[2] + (points[3] - points[2]) / 2;
    }

    if (prevPoint.x < 0 || prevPoint.y < 0)
        if (!opposite)
            return p0; // default
        else
            return p1;

    const float dist0 = cv::norm(prevPoint - p0);
    const float dist1 = cv::norm(prevPoint - p1);

    if (checkDirection)
    {
        cv::Point2f p0i = points[0] + (points[1] - points[0]) / 2;
        cv::Point2f p1i = points[1] + (points[2] - points[1]) / 2;
        cv::Point2f p2i = points[2] + (points[3] - points[2]) / 2;
        cv::Point2f p3i = points[0] + (points[3] - points[0]) / 2;

        const cv::Point2f prevCenter = rect.center - prevPoint;
        const float prevCenterNorm = cv::norm(prevCenter);
        const float cosDir0 = prevCenter.dot(p0i - prevPoint) / (prevCenterNorm * cv::norm(p0i - prevPoint));
        const float cosDir1 = prevCenter.dot(p1i - prevPoint) / (prevCenterNorm * cv::norm(p1i - prevPoint));
        const float cosDir2 = prevCenter.dot(p2i - prevPoint) / (prevCenterNorm * cv::norm(p2i - prevPoint));
        const float cosDir3 = prevCenter.dot(p3i - prevPoint) / (prevCenterNorm * cv::norm(p3i - prevPoint));

        if (cosDir0 >= cosDir1 && cosDir0 >= cosDir2 && cosDir0 >= cosDir3)
            return p0i;
        else if (cosDir1 >= cosDir0 && cosDir1 >= cosDir2 && cosDir1 >= cosDir3)
            return p1i;
        else if (cosDir2 >= cosDir0 && cosDir2 >= cosDir1 && cosDir2 >= cosDir3)
            return p2i;
        else
            return p3i;
    }

    if (dist0 > dist1)
        return p0;
    return p1;

    // obtain the direction considering the rectangle angle
    //return rect.center;
    /*
    float angle; // angle along longer side
    if(rect.size.width < rect.size.height)
        angle = rect.angle + 180;
    else
        angle = rect.angle + 90;
    
    const float angleRadiance = angle * M_PI / 180.0f;
    float cosAlpha = std::cos(angleRadiance);
    float sinAlpha = std::sin(angleRadiance);
    
    cv::Point2f direction(1, 0);
    direction.x = direction.x * cosAlpha + direction.y * sinAlpha;
    direction.y = - direction.x * sinAlpha + direction.y * cosAlpha;
    cv::Point2f direction90(direction.y, -direction.x);

    if (prevPoint.x >= 0 && prevPoint.y >= 0)
    {
        // get the best direction based on the angle and the previous rectangle
        cv::Point2f conn = rect.center - prevPoint;
        conn = conn / cv::norm(conn);
        const float forward = conn.dot(direction);
        const float backward = conn.dot(-direction);

        if (forward < backward)
            return rect.center - direction * rect.size.width;
    }

    return rect.center + direction * rect.size.width;*/
}

inline std::vector<cv::Point2f> FindPeaks(const std::vector<std::list<cv::Point2f>>& segments, const cv::Mat& image, cv::Mat imageDrawOn, float thresholdSaturation = 200, float thresholdPixelCount = 2, 
    float thresholdSkipBoundary = 10, bool visualize = false, std::string windowname = "Peaks")
{
    // Find the features on the rope
    // TODO: improve!

    std::vector<cv::Point2f> peaks;
    std::vector<float> values;
    float lowestSaturation = 10000;

    for (int ll = 0; ll < segments.size(); ++ll)
    {
        cv::Point2f start = cv::Point2f(-1,-1);
        int count = 0;

        std::vector<cv::Point2f> seg{ std::make_move_iterator(std::begin(segments[ll])), 
                  std::make_move_iterator(std::end(segments[ll])) };

        for (int l = 0; l < seg.size() - 1; ++l)
        {
            cv::Point2f p0 = seg[l + 0];
            cv::Point2f p1 = seg[l + 1];
            float distance = cv::norm(p1 - p0);
            cv::Point2f dir = (p1 - p0) / distance; // normalized direction

            cv::Point2f p = p0;

            // skip initial and final peaks - too close to the line boundaries
            int i = 0;
            if (l == 0)
                i = thresholdSkipBoundary;
            else if (l == seg.size() - 2)
                distance -= thresholdSkipBoundary;

            for (; i < (int)distance; ++i)
            {
                cv::Vec3b value = image.at<cv::Vec3b>(int(p.y), int(p.x));
                values.push_back(value[1]);

                //float dist = std::abs(prev[1] - value[1]);
                if (i > 0 && value[1] < thresholdSaturation)//dist >= threshold)
                {
                    if (start.x < 0)
                    {
                        start = p;
                        count = 1;
                    }
                    else
                        count++;

#if(DEBUGOUT)
                    if (visualize)
                    {
                        imageDrawOn.at<cv::Vec3b>(int(p.y), int(p.x)) = cv::Vec3f(255, 255, 255);
                    }
#endif

                    if (lowestSaturation < value[1])
                        lowestSaturation = value[1];
                }
                else if (i > 0 && value[1] >= thresholdSaturation)//dist < threshold)
                {
                    if (start.x >= 0)
                    {
                        if (count >= thresholdPixelCount) // at least a length of X pixels
                        {
                            cv::Point2f center = start + (p - start) / 2.0f;
                            peaks.push_back(center); // std::pair<cv::Point2f, float>(center, value[1]));

#if(DEBUGOUT)
                            if (visualize)
                            {
                                cv::drawMarker(imageDrawOn, cv::Point(int(center.x), int(center.y)), cv::Scalar(0, 255, 255), 0);
                            }
#endif
                        }

                        start.x = start.y = -1; // reset!
                        count = 0;
                    }

#if(DEBUGOUT)
                    if (visualize)
                    {
                        imageDrawOn.at<cv::Vec3b>(int(p.y), int(p.x)) = cv::Vec3f(255, 0, 255);
                    }
#endif
                }

                // for the next iteration
                p += dir;
                //prev = value;
            }

            std::cout << std::endl;
        }
    }

    //for (int i = 0; i < peaks.size(); ++i)
    //    peaks[i].second = 1.0f - (peaks[i].second - lowestSaturation) / (thresholdSaturation - lowestSaturation);

#if(DEBUGOUT)
    if (visualize)
    {
        cv::imshow(windowname, imageDrawOn);
        cv::waitKey(0);
    }
#endif

    return peaks;
}

inline std::vector<cv::Point2f> FindPeaksStandalone(const std::vector<std::list<cv::Point2f>>& segments, const cv::Mat& image, cv::Mat imageDrawOn,
    float thresholdPixelCount = 2, bool visualize = false, std::string windowname = "PeaksStandalone")
{
    // Find the features on the rope
    std::vector<float> values;
    std::vector<cv::Point2f> points;
    float max = 0, min = 255, avg = 0;

    for (int ll = 0; ll < segments.size(); ++ll)
    {
        std::vector<cv::Point2f> seg{ std::make_move_iterator(std::begin(segments[ll])),
                  std::make_move_iterator(std::end(segments[ll])) };

        for (int l = 0; l < seg.size() - 1; ++l)
        {
            cv::Point2f p0 = seg[l + 0];
            cv::Point2f p1 = seg[l + 1];
            float distance = cv::norm(p1 - p0);
            cv::Point2f dir = (p1 - p0) / distance; // normalized direction
            cv::Point2f p = p0;

            for (int i = 0; i < (int)distance; ++i)
            {
                if (!(p.x >= 0 && p.y >= 0 && p.x < image.cols && p.y < image.rows))
                    break; 
                cv::Vec3b value = image.at<cv::Vec3b>(int(p.y), int(p.x));
                values.push_back(value[1]);
                points.push_back(p);
                p += dir;

                if (value[1] < min)
                    min = value[1];
                if (value[1] > max)
                    max = value[1];
                avg += value[1];
            }
        }
    }
    avg /= values.size();

    // Analyze signal to find the most prominent peaks
    std::vector<cv::Point2f> peaks;
    int start = -1, count = 0;

    for (int i = 0; i < values.size(); ++i)
    {
        //std::cout << i << ") ";
        if (values[i] < avg)
        {
            //std::cout << "values[i]("<< values[i] <<") < avg(" << avg << ") ";
            if (start < 0)
            {
                start = i;
                count = 1;

                //std::cout << "starting with start=" << i << ", count=" << count;
            }
            else
            {
                count++;
                //std::cout << "counting with start=" << i << ", count=" << count;
            }
        }
        else
        {
            //std::cout << "values[i](" << values[i] << ") >= avg(" << avg << ") ";
            if (start >= 0)
            {
                if (count >= thresholdPixelCount) // at least a length of X pixels
                {
                    int id = start + (i - start) / 2;
                    peaks.push_back(points[id]);
                    //std::cout << "adding peak at id=" << id << " RESET";


#if(DEBUGOUT)
                    if (visualize)
                    {
                        cv::drawMarker(imageDrawOn, cv::Point(int(points[id].x), int(points[id].y)), cv::Scalar(0, 255, 255), 0);
                    }
#endif
                }
                else
                {
                    //std::cout << "NO peak, since count(" << count << ") < thresholdPixelCount (" << thresholdPixelCount << ") RESET";
                }


                start = -1; // reset!
                count = 0;
            }
        }

        std::cout << std::endl;
    }


#if(DEBUGOUT)
    if (visualize)
    {
        cv::imshow(windowname, imageDrawOn);
        cv::waitKey(0);
    }
#endif

    return peaks;
}

inline void GetDetails(const std::list<cv::Point2f>& s, cv::Point2f& end0, cv::Point2f& end1, cv::Point2f& diff0, cv::Point2f& diff1)
{
    end0 = s.front();
    end1 = s.back();

    if (s.size() > 1)
    {
        const cv::Point2f prev0 = *std::prev(std::prev(s.end()));
        const cv::Point2f prev1 = *std::next(s.begin());
        
        diff0 = end0 - prev0;
        const float diff0norm = cv::norm(diff0);
        diff0 /= diff0norm;
        diff1 = end1 - prev1;
        const float diff1norm = cv::norm(diff1);
        diff1 /= diff1norm;
    }
    else
    {
        diff0 = cv::Point2f(0,0);
        diff1 = cv::Point2f(0,0);
    }
}

inline std::vector<std::list<cv::Point2f>> ConnectedLines(const std::vector<std::list<cv::Point2f>> segments, cv::Mat imageDrawOn, bool visualize = false)
{
    std::vector<std::list<cv::Point2f>> segments_new;

    if (segments.size() > 6)
    {
        // first fuse together close segments
        std::vector<cv::Point2f> points;
        std::vector<int> segmentId;

        for (int i = 0; i < segments.size(); ++i)
        {
            if (segments[i].size() > 1)
            {
                points.push_back(segments[i].back());
                segmentId.push_back(i);
            }

            points.push_back(segments[i].front());
            segmentId.push_back(i);
        }

        cv::flann::KDTreeIndexParams indexParams;//(2);
        cv::flann::Index kdtree(cv::Mat(points).reshape(1), indexParams);
        std::vector<float> query(2);
        unsigned int max_neighbours = 5;
        float factor = 300;
        float radius = 4.0f * factor * factor;

        std::vector<std::pair<int,int>> to_fuse;
        std::vector<bool> assigned(segments.size());

        for (int i = 0; i < segments.size(); ++i)
        {
            if (assigned[i])
                continue;

            std::vector<cv::Point2f> endpoints;
            if (segments[i].size() > 1)
                endpoints.push_back(segments[i].back());
            endpoints.push_back(segments[i].front());

            bool fuse = false;
            for (int p = 0; p < endpoints.size() && !fuse; ++p) // do for both end points until at least one is connected
            {
                cv::Point2f pt = endpoints[p];

                std::vector<int> indices;
                std::vector<float> distances;
            
                query[0] = pt.x;
                query[1] = pt.y;

                int found = kdtree.radiusSearch(query, indices, distances, radius, max_neighbours);
                // Note: indices[0] is the point itself

                std::vector<std::pair<int, float>> pts;
                for (int f = 1; f < found && f < max_neighbours; ++f)
                {
                    if (segmentId[indices[f]] != i && !assigned[segmentId[indices[f]]])
                        pts.push_back(std::pair<int, float>(segmentId[indices[f]], distances[f]));
                }

                if (pts.size() > 1)
                {
                    float dist_first = pts[0].second;
                    float dist_second = pts[1].second;

                    if (dist_second > 3.0f * dist_first)
                    {
                        // fuse segments together
                        to_fuse.push_back(std::pair<int,int>(i, pts[0].first));
                        assigned[i] = true;
                        assigned[pts[0].first] = true;
                        fuse = true;
                    }
                }
            }
        }

        // fuse segments together
        std::vector<int> still_to_add(segments.size());
        for (int i = 0; i < segments.size(); ++i)
            still_to_add[i] = i;

        for (int i = 0; i < to_fuse.size(); ++i)
        {
            std::list<cv::Point2f> s1 = segments[to_fuse[i].first];
            std::list<cv::Point2f> s2 = segments[to_fuse[i].second];
            std::list<cv::Point2f> fused;

            float d0 = cv::norm(s1.front() - s2.front());
            float d1 = cv::norm(s1.front() - s2.back());
            float d2 = cv::norm(s1.back() - s2.front());
            float d3 = cv::norm(s1.back() - s2.back());

            if (d0 <= d1 && d0 <= d2 && d0 <= d3)
            {
                std::reverse(s1.begin(), s1.end()); // reverse s1 first
                fused.insert( fused.end(), s1.begin(), s1.end() ); // first s1, then s2
                fused.insert( fused.end(), s2.begin(), s2.end() );
            }
            else if (d1 <= d0 && d1 <= d2 && d1 <= d3)
            {
                fused.insert( fused.end(), s2.begin(), s2.end() ); // first s2, then s1
                fused.insert( fused.end(), s1.begin(), s1.end() );
            }
            else if (d2 <= d0 && d2 <= d1 && d2 <= d3)
            {
                fused.insert( fused.end(), s1.begin(), s1.end() ); // first s1, then s2
                fused.insert( fused.end(), s2.begin(), s2.end() );
            }
            else
            {
                std::reverse(s2.begin(), s2.end()); // reverse s2 first
                fused.insert( fused.end(), s1.begin(), s1.end() ); // first s1, then s2
                fused.insert( fused.end(), s2.begin(), s2.end() );
            }

            segments_new.push_back(fused);

            // remove from the segments to add later
            auto to_remove = std::find(still_to_add.begin(), still_to_add.end(), to_fuse[i].first);
            if (to_remove != still_to_add.end())
                still_to_add.erase(to_remove);
        
            to_remove = std::find(still_to_add.begin(), still_to_add.end(), to_fuse[i].second);
            if (to_remove != still_to_add.end())
                still_to_add.erase(to_remove);
        }

        // add the missing segments
        for (int i = 0; i < still_to_add.size(); ++i)
            segments_new.push_back(segments[still_to_add[i]]);

#if(DEBUGOUT)
        if (visualize)
        {
            // Draw new segments
            for (int l = 0; l < segments_new.size(); ++l)
            {
                cv::Scalar color(rand() * 255, rand() * 255, rand() * 255);
                std::cout << "Drawing new line " << l << " with size " << segments_new[l].size() << std::endl;

                cv::Point2f prevPoint(-1, -1);
                for (cv::Point2f point : segments_new[l])
                {
                    if (prevPoint.x > 0)
                        cv::line(imageDrawOn, point, prevPoint, color, 2);
                    prevPoint = point;
                }
                if (segments_new[l].size() == 1)
                    cv::drawMarker(imageDrawOn, segments_new[l].front(), color, 4, 10);
            }

            cv::imshow("New Lines", imageDrawOn);
            cv::waitKey(0);
        }
#endif 

    }
    else
    {
        segments_new = segments;
    }

    if (segments_new.size() > 6 || segments_new.size() <= 1)
        return segments_new; // do not connect lines in these cases!

    // find best permutation sequence!
    std::vector<int> sequence(segments_new.size());
    for (int i = 0; i < segments_new.size(); ++i)
        sequence[i] = i;

    // TODO: pick first 5 best combinations and compute direction accuracy
    //std::vector<std::pair<std::vector<int>, float>> scores;
    std::vector<bool> conn(segments.size() * 2);
    const float max_float = std::numeric_limits<float>::max();
    
    float best_score = max_float;
    std::vector<int> chosen(sequence.size());

    do
    {
        // compute minimum distances sum

        float sum = 0.0f;
        for (int i = 0; i < sequence.size() - 1; ++i)
        {
            const int id0 = sequence[i];
            const int id1 = sequence[i+1];
            std::list<cv::Point2f> s1 = segments_new[id0];
            std::list<cv::Point2f> s2 = segments_new[id1];

            float d0 = (!conn[id0 * 2] && !conn[id1 * 2]) ? cv::norm(s1.front() - s2.front()) : max_float;
            float d1 = (!conn[id0 * 2] && !conn[id1 * 2 + 1]) ? cv::norm(s1.front() - s2.back()) : max_float;
            float d2 = (!conn[id0 * 2 + 1] && !conn[id1 * 2]) ? cv::norm(s1.back() - s2.front()) : max_float;
            float d3 = (!conn[id0 * 2 + 1] && !conn[id1 * 2 + 1]) ?cv::norm(s1.back() - s2.back()) : max_float;

            sum += std::min(std::min(std::min(d0, d1), d2), d3);
        }

        //scores.push_back(std::pair<std::vector<int>,float>(sequence, sum));
        if (sum < best_score)
        {
            best_score = sum;
            chosen = sequence; // TODO: Check that it has a copy!
        }

    } while (std::next_permutation(sequence.begin(), sequence.end()));

    //std::sort(scores.begin(), scores.end(), [](auto &left, auto &right) {
    //    return left.second < right.second;
    //});
    //std::vector<int> chosen = scores[0].first;
    
    // generate continuous line
    // reverse segments where needed

    for (int i = 0; i < segments_new.size() - 1; ++i)
    {
        const int id0 = chosen[i];
        const int id1 = chosen[i + 1];
        std::list<cv::Point2f> s1 = segments_new[id0];
        std::list<cv::Point2f> s2 = segments_new[id1];

        float d0 = cv::norm(s1.front() - s2.front());
        float d1 = cv::norm(s1.front() - s2.back());
        float d2 = cv::norm(s1.back() - s2.front());
        float d3 = cv::norm(s1.back() - s2.back());

        if (d0 <= d1 && d0 <= d2 && d0 <= d3)
        {
            std::reverse(segments_new[id0].begin(), segments_new[id0].end()); // reverse s1
        }
        else if (d1 <= d0 && d1 <= d2 && d1 <= d3)
        {
            std::reverse(segments_new[id0].begin(), segments_new[id0].end()); // reverse s1
            std::reverse(segments_new[id1].begin(), segments_new[id1].end()); // reverse s2
        }
        else if (d2 <= d0 && d2 <= d1 && d2 <= d3)
        {
        }
        else
        {
            std::reverse(segments_new[id1].begin(), segments_new[id1].end()); // reverse s2
        }
    }

    // swap lines and create final line
    std::vector<std::list<cv::Point2f>> segments_final(segments_new.size());
    for (int i = 0; i < chosen.size(); ++i)
        segments_final[i] = segments_new[chosen[i]];

#if(DEBUGOUT)
    if (visualize)
    {
        // draw connected lines!
        cv::Point2f prev(0, 0);
        for (int i = 0; i < segments_final.size(); ++i)
        {
            for (auto it = segments_final[i].begin(); it != segments_final[i].end(); ++it)
            {
                cv::line(imageDrawOn, prev, *it, cv::Scalar(50, 30, 100), 1);
                prev = *it;
            }
        }

        cv::imshow("Final line", imageDrawOn);
        cv::waitKey(0);
    }
#endif

    return segments_final;
}

inline double ContainsPoint(cv::RotatedRect rectangle, cv::Point2f point) {

    //Get the corner points
    cv::Point2f corners[4];
    rectangle.points(corners);

    //Convert the point array to a vector.
    cv::Point2f* lastItemPointer = (corners + sizeof corners / sizeof corners[0]);
    std::vector<cv::Point2f> contour(corners, lastItemPointer);

    //Check if the point is within the rectangle.
    double indicator = pointPolygonTest(contour, point, true);

    return indicator;
}

inline std::vector<std::list<cv::Point2f>> FindSegments(cv::Mat imageMask, float factor, cv::Mat imageDrawOn, int minRectSize = 3, bool visualize = false)
{
    std::vector<std::list<cv::Point2f>> lines;
    std::list<cv::Point2f> lines_to_process;

    //cv::imwrite("C:\\Users\\robertini\\Documents\\coding\\GreifbAR_Unity_DFKI\\Output\\test.png", imageMask);

    // find points where possible lines are starting
    for (int i = 100; i < imageMask.rows - 100; i+=factor)
    {
        for(int j = 100; j < imageMask.cols - 100; j+=factor)
        {
            if (imageMask.at<uchar>(cv::Point(j, i)) > 0)
            {
                lines_to_process.push_back(cv::Point(j, i));
            }
        }
    }

    //for (int i = 0; i < imageMask.cols; i+=factor)
    //{
    //    for(int j = 0; j < imageMask.rows; j++)
    //    {
    //        if (imageMask.at<uchar>(cv::Point(i, j)) > 0)
    //        {
    //            lines_to_process.push_back(cv::Point(i, j));
    //            break;
    //        }
    //    }
    //}

    // Detect continuous lines
    const int boxSize = 2 * factor;
    const int halfBoxSize = factor;
    cv::Mat boxMask = cv::Mat::zeros(imageMask.rows, imageMask.cols, CV_8U); // all 0
    cv::Mat prevBoxMask;
    cv::Mat imageMaskReducing; imageMask.copyTo(imageMaskReducing);

#if(DEBUGOUT)
    std::vector<cv::Scalar> colors(300);
    for (int i = 0; i < colors.size(); ++i)
        colors[i] = cv::Scalar(rand()*255, rand()*255, rand()*255);
#endif

    while (!lines_to_process.empty())
    {
        // pop a line to process
        cv::Point2f point = lines_to_process.back(), prevPoint(-1,-1);
        lines_to_process.pop_back();

        if (imageMaskReducing.at<uchar>(point.y, point.x) <= 0)
            continue; // this point has been considered already!

#if(DEBUGOUT)
        int colorId = lines_to_process.size() - 1;
#endif

        std::list<cv::Point2f> line;
        std::list<cv::RotatedRect> rectangles;
        cv::Mat mask;
        bool twoSidesChecked = false;

        do {

            // current box
            const int x = std::min(std::max(0.0f, point.x - halfBoxSize), (float)imageMask.cols - 1);
            const int y = std::min(std::max(0.0f, point.y - halfBoxSize), (float)imageMask.rows - 1);
            const int w = (x + boxSize < imageMask.cols)? boxSize: imageMask.cols - x - 1;
            const int h = (y + boxSize < imageMask.rows)? boxSize: imageMask.rows - y - 1;
            cv::Rect box(x, y, w, h);

            boxMask(box) = 255;

            // masked pixels inside the box
            cv::bitwise_and(imageMaskReducing, boxMask, mask);
            boxMask(box) = 0; // immediately remove the box mask: not needed after this step!

#if(DEBUGOUT)
            if (visualize)
            {
                cv::imshow("boxMask", boxMask);
                cv::imshow("Reducing", imageMaskReducing);
                cv::imshow("mask", mask);
            }
#endif

            // find contours
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // for each contour fit a rect
            std::vector<cv::RotatedRect> rects;
            for (int i = 0; i < contours.size(); ++i)
            {
                if (!contours[i].empty())
                {
                    rects.push_back(cv::minAreaRect(contours[i]));
                    
#if(DEBUGOUT)
                    /*cv::Point2f vertices2f[4];
                    rects[i].points(vertices2f);

                    cv::Point vertices[4];    
                    for(int i = 0; i < 4; ++i){
                        vertices[i] = vertices2f[i];
                    }

                    cv::fillConvexPoly(imageDrawOn, vertices, 4, colors[colorId]);
                    cv::drawMarker(imageDrawOn, point, cv::Scalar(0,255,255)); // yellow
                    cv::drawMarker(imageDrawOn, rects[i].center, colors[colorId]);*/
#endif
                }
            }

#if(DEBUGOUT)
            //cv::imshow("rects", imageDrawOn);
            //cv::waitKey(0);
#endif

            // continue current line and add any additional detached lines to the list to be processed later
            int id = -1;
            for (int i = 0; i < rects.size(); ++i)
            {
                if (rects[i].size.width < minRectSize || rects[i].size.height < minRectSize)
                    continue; // skip invalid rectangles

                if (ContainsPoint(rects[i], point) >= -3) // check whether its distance to the current line is smaller enough
                {
                    //std::cout << "Found connection\n";
                    if (!twoSidesChecked)
                    {
                        line.push_back(rects[i].center);
                        rectangles.push_back(rects[i]);
                    }
                    else
                    {
                        line.push_front(rects[i].center);
                        rectangles.push_front(rects[i]);
                    }

                    id = i;

                    // point to check at the next iteration
                    point = NextPoint(rects[i], prevPoint);
                    prevPoint = rects[i].center;
                    
                    // remove the processed pixels
                    cv::Point2f vertices2f[4];
                    rects[i].points(vertices2f);

                    cv::Point vertices[4];    
                    for(int i = 0; i < 4; ++i){
                        vertices[i] = vertices2f[i];
                    }

                    cv::fillConvexPoly(imageMaskReducing, vertices, 4, cv::Scalar(0,0,0));
                }
                else
                {
                    //std::cout << "Found detached line\n";
                    lines_to_process.push_back(rects[i].center);
                }
            }
    
            // check whether it was possible to continue the line, or no rectangles have been found
            if (id < 0)// || (rects.size() == 1 && rects[0].size.width < minRectSize || rects[0].size.height < minRectSize))
            {
                //std::cout << "No connection\n";
                // check if we still need to go the opposite direction
                if (!twoSidesChecked && !rectangles.empty())
                {
                    point = NextPoint(rectangles.front(), cv::Point2f(-1,-1), true);
                    prevPoint = rectangles.front().center;

                    twoSidesChecked = true;
                }
                else
                {
                    if (!line.empty())
                    {
                        // add both sides till line end
                        cv::Point2f end0, end1;
                        if (line.size() >= 2)
                        {
                            end0 = NextPoint(rectangles.back(), *std::prev(std::prev(line.end())), false, true);
                            end1 = NextPoint(rectangles.front(), *std::next(line.begin()), false, true);
                            line.push_back(end0);
                            line.push_front(end1);
                        }
                        else
                        {
                            //end0 = NextPoint(rectangles.back(), cv::Point2f(-1,-1), false);
                            //end1 = NextPoint(rectangles.back(), cv::Point2f(-1,-1), true);
                            // just add the single point, since the sides might be wrong!
                        }

                        lines.push_back(line); // store the current line
                    }
                    
                    break; // exit the loop!
                }

                //if (!line.empty())
                //    lines.push_back(line); // store the current line
                //break;
            }

        } while(true);
    }

    // final pass to include all remaining blobs as point segments
    std::vector<std::vector<cv::Point>> contoursFinal;
    cv::findContours(imageMaskReducing, contoursFinal, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // for each contour fit a rect
    for (int i = 0; i < contoursFinal.size(); ++i)
    {
        if (!contoursFinal[i].empty())
        {
            cv::RotatedRect rect = cv::minAreaRect(contoursFinal[i]);

            if (rect.size.width < minRectSize || rect.size.height < minRectSize)
                continue; // skip invalid rectangles
            
            if (rect.center.x <= 100 || rect.center.y <= 100 || rect.center.x > imageMask.cols - 100 || rect.center.y > imageMask.rows - 100)
                continue; // skip rectangle too close to the image boundaries!

            std::list<cv::Point2f> line;
            line.push_back(rect.center);
            lines.push_back(line);
        }
    }

#if(DEBUGOUT)
    if (visualize)
    {
        for (int l = 0; l < lines.size(); ++l)
        {
            cv::Scalar color(rand() * 255, rand() * 255, rand() * 255);
            //std::cout << "Drawing line " << l << " with size " << lines[l].size() << std::endl;

            cv::Point2f prevPoint(-1, -1);
            for (cv::Point2f point : lines[l])
            {
                if (prevPoint.x > 0)
                    cv::line(imageDrawOn, point, prevPoint, color, 2);
                prevPoint = point;
            }
            if (lines[l].size() == 1)
                cv::drawMarker(imageDrawOn, lines[l].front(), color, 4, 10);
        }

        cv::imshow("Lines", imageDrawOn);
        cv::waitKey(0);
    }
#endif

    return lines;
}

inline std::vector<float> ComputeExpectedPeakDistances(const std::vector<std::list<cv::Point2f>>& line, const cv::Mat& mask)
{
    // Compute expected peaks distance
    std::vector<float> expectedPeaksDistances;
    expectedPeaksDistances.reserve(line.size() * 20);

    cv::Point2f prev(-1, -1);
    float expectedPeaksDistance = 0;

    for (int i = 0; i < line.size(); ++i)
    {
        for (auto it = line[i].begin(); it != line[i].end(); ++it)
        {
            if (prev.x >= 0 && prev.y >= 0)
            {
                cv::Point2f curr = *it;
                float distance = cv::norm(curr - prev);
                cv::Point2f dir = (curr - prev) / distance; // normalized direction

                // computed expected peak distance at this location
                cv::Point2f dir90(dir.y, -dir.x);
                cv::Point2f p = prev;
                int length = 0;
                while (int(p.x) >= 0 && int(p.y) >= 0 && int(p.x) < mask.cols && int(p.y) < mask.rows && mask.at<uchar>(int(p.y), int(p.x)) > 0)
                {
                    length++;
                    p += dir90;
                }
                p = prev - dir90;
                while (int(p.x) >= 0 && int(p.y) >= 0 && int(p.x) < mask.cols && int(p.y) < mask.rows && mask.at<uchar>(int(p.y), int(p.x)) > 0)
                {
                    length++;
                    p -= dir90;
                }

                if (length > 0) // use previous value, if this one is zero
                    expectedPeaksDistance = 15.0f * length / 4.0f;

                expectedPeaksDistances.push_back(expectedPeaksDistance);
            }

            prev = *it;
        }
    }

    // Apply some smoothing
    if (expectedPeaksDistances.size() > 2)
    {
        for (int i = 1; i < expectedPeaksDistances.size() - 1; ++i)
            expectedPeaksDistances[i] = (expectedPeaksDistances[i - 1] + expectedPeaksDistances[i] + expectedPeaksDistances[i + 1]) / 3.0f;
    }

    return expectedPeaksDistances;
}

inline std::vector<cv::Point2f> RetrievePeaks(const std::vector<std::list<cv::Point2f>>& line, const std::vector<float>& expectedPeaksDistances, const cv::Mat& mask, cv::Mat& imageDrawOn, bool visualize = false)
{
    // Identify approximate peaks
    int pathLength = 0;
    cv::Point2f prev(-1, -1);
    int id = 0;
    std::vector<cv::Point2f> peaks_final;

    for (int i = 0; i < line.size(); ++i)
    {
        for (auto it = line[i].begin(); it != line[i].end(); ++it)
        {
            if (prev.x >= 0 && prev.y >= 0)
            {
                cv::Point2f curr = *it;
                float distance = cv::norm(curr - prev);
                cv::Point2f dir = (curr - prev) / distance; // normalized direction

                // how many peaks to add in this segment:
                const int n = std::floor((pathLength + distance) / expectedPeaksDistances[id]);

                for (int j = 1; j <= n; ++j)
                {
                    cv::Point2f peak = curr + (j * expectedPeaksDistances[id] - pathLength) * dir;
                    if (int(peak.x) >= 0 && int(peak.y) >=0 && int(peak.x) < mask.cols && int(peak.y) <= mask.rows)
                        if (mask.at<uchar>(int(peak.y), int(peak.x)) > 0)
                            peaks_final.push_back(peak);
                }

                pathLength = pathLength + distance - n * expectedPeaksDistances[id];
                id++;
            }

            prev = *it;
        }
    }

#if(DEBUGOUT)
    if (visualize)
    {
        // Show final peaks

        for (int i = 0; i < peaks_final.size(); ++i)
            cv::drawMarker(imageDrawOn, peaks_final[i], cv::Scalar(0, 0, 0), 0, 20, 2);

        cv::imshow("Final Peaks", imageDrawOn);
        cv::waitKey(0);
    }
#endif

    return peaks_final;
}

inline cv::Point2f FindPivot(std::vector<std::list<cv::Point2f>>& line0, std::vector<std::list<cv::Point2f>>& line1, cv::Mat imageDrawOn, bool visualize = false)
{
    if (line0.empty() || line1.empty())
        return cv::Point2f(0, 0); // invalid pivot

    // find pivot and reverse lines such that their start is at the pivot
    cv::Point2f line0End, line0Start, line1End, line1Start;
    line0End = line0.back().back();
    line0Start = line0.front().front();
    line1End = line1.back().back();
    line1Start = line1.front().front();

    const float d00 = cv::norm(line0Start - line1Start);
    const float d01 = cv::norm(line0Start - line1End);
    const float d10 = cv::norm(line0End - line1Start);
    const float d11 = cv::norm(line0End - line1End);

    cv::Point2f pivot;
    if (d00 <= d01 && d00 <= d10 && d00 <= d11)
    {
        pivot = line1Start + (line0Start - line1Start) / 2;
    }
    else if (d01 <= d00 && d01 <= d10 && d01 <= d11)
    {
        pivot = line1End + (line0Start - line1End) / 2;
        std::reverse(line1.begin(), line1.end());
        for (int i = 0; i < line1.size(); ++i)
            std::reverse(line1[i].begin(), line1[i].end());
    }
    else if (d10 <= d00 && d10 <= d01 && d10 <= d11)
    {
        pivot = line1Start + (line0End - line1Start) / 2;
        std::reverse(line0.begin(), line0.end());
        for (int i = 0; i < line0.size(); ++i)
            std::reverse(line0[i].begin(), line0[i].end());
    }
    else
    {
        pivot = line1End + (line0End - line1End) / 2;
        std::reverse(line1.begin(), line1.end());
        std::reverse(line0.begin(), line0.end());
        for (int i = 0; i < line0.size(); ++i)
            std::reverse(line0[i].begin(), line0[i].end());
        for (int i = 0; i < line1.size(); ++i)
            std::reverse(line1[i].begin(), line1[i].end());
    }

    // TODO: Validate the pivot (sometimes it is really not found)

#if(DEBUGOUT)
    if (visualize)
    {
        // draw connected lines!
        cv::Point2f prev(0, 0);
        for (int i = 0; i < line0.size(); ++i)
        {
            for (auto it = line0[i].begin(); it != line0[i].end(); ++it)
            {
                cv::line(imageDrawOn, prev, *it, cv::Scalar(50, 100, 150), 1);
                prev = *it;
            }
        }

        cv::drawMarker(imageDrawOn, pivot, cv::Scalar(0, 0, 0));
        cv::imshow("Line with Pivot", imageDrawOn);
        cv::waitKey(0);
    }
#endif

    return pivot;
    
    // Identify peaks
    //std::vector<std::pair<cv::Point2f, float>> peaks0 = FindPeaks(line0, image, imageDrawOn, 200, 1, 10, true);
    //std::vector<std::pair<cv::Point2f, float>> peaks1 = FindPeaks(line1, image, imageDrawOn, 180, 1, true);

    // Navigate through the line starting at the pivot
    // by a distance specified in the expectedPeaksDistances

    // the first peak is at a distance within [0, expectedPeaksDistances + 5]

    /*
    int id = 0;
    int thresholdSaturation = 200;
    int thresholdPixelCount = 1;
    cv::Point2f firstPeak(-1, -1);
    int pathLength = 0;
    for (int l = 0; l < line0.size(); ++l)
    {
        if (line0[l].size() < 2)
        {
            id++;
            continue; // at least two points are needed
        }

        cv::Point2f p0 = *(line0[l].begin());
        cv::Point2f start = cv::Point2f(-1, -1);
        int count = 0;

        if (l > 0)
            pathLength += cv::norm(line0[l].front() - line0[l - 1].back()); // Add missing pathLength

        for (auto it = std::next(line0[l].begin()); it != line0[l].end(); ++it, ++id)
        {
            cv::Point2f p1 = *it;
            float distance = cv::norm(p1 - p0);
            cv::Point2f dir = (p1 - p0) / distance; // normalized direction

            //cv::Point2f expected_point = p0 + dir * expectedPeaksDistances[id];

            if (peaks_final.empty())
            {
                // for the first point navigate from the first found peak
                // until a distance of expectedPeaksDistances[id] is reached

                cv::Point2f p = p0;

                // skip initial and final peaks - too close to the line boundaries
                int i = 0;
                if (i == 0)
                    i = thresholdSkipBoundary;
                else if (l == line0.size() - 2)
                    distance -= thresholdSkipBoundary;
                int lowestSaturation = 10000;

                for (; i < (int)distance; ++i, ++pathLength)
                {
                    cv::Vec3b value = image.at<cv::Vec3b>(int(p.y), int(p.x));

                    //float dist = std::abs(prev[1] - value[1]);
                    if (i > 0 && value[1] < thresholdSaturation)//dist >= threshold)
                    {
                        if (start.x < 0)
                        {
                            start = p;
                            count = 1;
                        }
                        else
                            count++;

#if(DEBUGOUT)
                        if (visualize)
                        {
                            imageDrawOn.at<cv::Vec3b>(int(p.y), int(p.x)) = cv::Vec3f(255, 255, 255);
                        }
#endif
                    }
                    else if (i > 0 && value[1] >= thresholdSaturation)//dist < threshold)
                    {
                        if (start.x >= 0)
                        {
                            if (count >= thresholdPixelCount) // at least a length of X pixels
                            {
                                cv::Point2f center = start + (p - start) / 2.0f;

                                if (firstPeak.x < 0 || firstPeak.y < 0 || lowestSaturation > value[1])
                                {
                                    lowestSaturation = value[1];
                                    firstPeak = center;
                                    pathLength = 0;
                                }
#if(DEBUGOUT)
                                if (visualize)
                                {
                                    cv::drawMarker(imageDrawOn, cv::Point(int(center.x), int(center.y)), cv::Scalar(0, 255, 255), 0);
                                }
#endif
                            }

                            start.x = start.y = -1; // reset!
                            count = 0;
                        }

#if(DEBUGOUT)
                        if (visualize)
                        {
                            imageDrawOn.at<cv::Vec3b>(int(p.y), int(p.x)) = cv::Vec3f(255, 0, 255);
                        }
#endif
                    }

                    // for the next iteration
                    p += dir;
                    //prev = value;
                }

                if (distance >= expectedPeaksDistances[id] + 10)
                {
                    if (firstPeak.x < 0 || firstPeak.y < 0)
                        std::cout << "ERROR: No first peak found\n";

                    peaks_final.push_back(firstPeak);
                }
            }
            else
            {
                // for the next points check peaks at a distance [expectedPeaksDistances[id]-10, expectedPeaksDistances[id]+10]
                cv::Point2f p = p0;

                // location of next peak around
                int expected = expectedPeaksDistances[id] - pathLength;

                for (int i = expected - 10; i < expected + 10; ++i, ++pathLength)
                {
                    cv::Vec3b value = image.at<cv::Vec3b>(int(p.y), int(p.x));

                    //float dist = std::abs(prev[1] - value[1]);
                    if (i > 0 && value[1] < thresholdSaturation)//dist >= threshold)
                    {
                        if (start.x < 0)
                        {
                            start = p;
                            count = 1;
                        }
                        else
                            count++;

#if(DEBUGOUT)
                        if (visualize)
                        {
                            imageDrawOn.at<cv::Vec3b>(int(p.y), int(p.x)) = cv::Vec3f(255, 255, 255);
                        }
#endif
                    }
                    else if (i > 0 && value[1] >= thresholdSaturation)//dist < threshold)
                    {
                        if (start.x >= 0)
                        {
                            if (count >= thresholdPixelCount) // at least a length of X pixels
                            {
                                cv::Point2f center = start + (p - start) / 2.0f;

                                if (firstPeak.x < 0 || firstPeak.y < 0 || lowestSaturation > value[1])
                                {
                                    lowestSaturation = value[1];
                                    firstPeak = center;
                                    pathLength = 0;
                                }
#if(DEBUGOUT)
                                if (visualize)
                                {
                                    cv::drawMarker(imageDrawOn, cv::Point(int(center.x), int(center.y)), cv::Scalar(0, 255, 255), 0);
                                }
#endif
                            }

                            start.x = start.y = -1; // reset!
                            count = 0;
                        }

#if(DEBUGOUT)
                        if (visualize)
                        {
                            imageDrawOn.at<cv::Vec3b>(int(p.y), int(p.x)) = cv::Vec3f(255, 0, 255);
                        }
#endif
                    }

                    // for the next iteration
                    p += dir;
                    //prev = value;
                }

            }

            p0 = p1; // for the next iteration!
        }
    }
    */

    /*
    // compute mean
    float mean_dist = 0;
    std::vector<float> dists(peaks0.size() - 1);
    for (int i = 0; i < peaks0.size() - 1; ++i)
    {
        dists[i] = cv::norm(peaks0[i].first - peaks0[i + 1].first);
        mean_dist += dists[i];
        std::cout << dists[i] << "\n";
    }

    std::cout << "Mean: " << (mean_dist / peaks0.size()) << "\n";
    std::cout << "\n";

    // Remove peaks until they are exactly 24 and are at a mean distance
    while (peaks0.size() > 24)
    {
        auto it = std::min_element(std::begin(dists), std::end(dists));
        int id = std::distance(std::begin(dists), it);

        dists.erase(it);
        peaks0.erase(peaks0.begin() + id);
    }
    */
}

inline std::vector<std::pair<int, int>> ConnectStereoPoints(const std::vector<cv::Point2f>& left, const std::vector<cv::Point2f>& right, const int epsilon = 3)
{
    int idL = 0, idR = 0;
    std::vector<std::pair<int, int>> corresponding;

    if (left.size() == 1)
    {
        int LY = left[0].y;
        float min = 1152;
        int idR = -1;

        for (int i = 0; i < right.size(); ++i)
        {
            int RY = right[i].y;
            const int distPx = std::abs(LY - RY);
            if (distPx < min)
            {
                min = distPx;
                idR = i;
            }
        }

        if (idR >= 0 && min < epsilon)
        {
            corresponding.push_back(std::pair<int, int>(0, idR));
        }

        return corresponding;
    }

    if (right.size() == 1)
    {
        int RY = right[0].y;
        float min = 1152;
        int idL = -1;

        for (int i = 0; i < left.size(); ++i)
        {
            int LY = left[i].y;
            const int distPx = std::abs(LY - RY);
            if (distPx < min)
            {
                min = distPx;
                idL = i;
            }
        }

        if (idL >= 0 && min < epsilon)
        {
            corresponding.push_back(std::pair<int, int>(idL, 0));
        }

        return corresponding;
    }

    while (idL < left.size() && idR < right.size())
    {
        // update current Y values for left and right
        int LY = left[idL].y, RY = right[idR].y;
        const int distPx = std::abs(LY - RY);
        int prevLY, prevRY;

        // update previous Y values for left and right
        if (idL <= 0)
        {
            if (left[1].y > left[0].y)
                prevLY = left[0].y - epsilon - 1; // fictional previous!
            else
                prevLY = left[0].y + epsilon + 1; // fictional previous!
        }
        else
            prevLY = left[idL - 1].y;

        if (idR <= 0)
        {
            if (right[1].y > right[0].y)
                prevRY = right[0].y - epsilon - 1; // fictional previous!
            else
                prevRY = right[0].y + epsilon + 1; // fictional previous!
        }
        else
            prevRY = right[idR - 1].y;

        if (distPx <= epsilon)
        {
            std::cout << "distPx(" << distPx << ") <= epsilon(" << epsilon << ") -->";
            std::cout << "ADD correspondence with idL(" << idL << "), idR(" << idR << ")";

            corresponding.push_back(std::pair<int, int>(idL, idR));
            idL++;
            idR++;
        }
        else
        {
            std::cout << "distPx(" << distPx << ") > epsilon(" << epsilon << ") -->";
            std::cout << "idL(" << idL << "), idR(" << idR << ")";

            // going down or up or constant
            if (LY >= prevLY) // going up/constant
			{
				if (LY < RY)
				{
                    std::cout << "LY >= prevLY(" << prevLY << ") && LY(" << LY << ") < RY(" << RY << ") --> idL++";
					idL++;
				}
                else
                {
                    std::cout << "LY >= prevLY(" << prevLY << ") && LY(" << LY << ") >= RY(" << RY << ") --> idR++";
                    idR++;
                }
            }
            else // going down
            {
                if (LY < RY)
                {
                    std::cout << "LY < prevLY(" << prevLY << ") && LY(" << LY << ") < RY(" << RY << ") --> idR++";
                    idR++;
                }
                else
                {
                    std::cout << "LY < prevLY(" << prevLY << ") && LY(" << LY << ") >= RY(" << RY << ") --> idL++";
                    idL++;
                }
            }
        }
        std::cout << std::endl;
    }

    return corresponding;
}

inline std::vector<cv::Point3f> Triangulate(const cv::Point2f& pivotL, const cv::Point2f& pivotR, const std::vector<cv::Point2f>& peaksL0, const std::vector<cv::Point2f>& peaksL1,
    const std::vector<cv::Point2f>& peaksR0, const std::vector<cv::Point2f>& peaksR1, int& npoints0, int& npoints1, cv::Mat& imageDrawOnLeft, cv::Mat& imageDrawOnRight, bool visualize = false)
{
    // Triangulate corresponding points

#if(DEBUGOUT)
    std::vector<cv::Point2f> pointsL, pointsR;
#endif

    npoints0 = std::min(peaksL0.size(), peaksR0.size());
    npoints1 = std::min(peaksL1.size(), peaksR1.size());
    const int npoints = npoints0 + npoints1 + 1; // including pivot!

    cv::Mat coordL = cv::Mat(2, npoints, CV_32F);
    cv::Mat coordR = cv::Mat(2, npoints, CV_32F);

    for (int i = 0; i < npoints0; ++i)
    {
        coordL.at<float>(0, i) = peaksL0[i].x;
        coordL.at<float>(1, i) = peaksL0[i].y;
        coordR.at<float>(0, i) = peaksR0[i].x;
        coordR.at<float>(1, i) = peaksR0[i].y;

#if(DEBUGOUT)
        pointsL.push_back(peaksL0[i]);
        pointsR.push_back(peaksR0[i]);
#endif
        
    }

    for (int i = npoints0; i < npoints0 + npoints1; ++i)
    {
        coordL.at<float>(0, i) = peaksL1[i - npoints0].x;
        coordL.at<float>(1, i) = peaksL1[i - npoints0].y;
        coordR.at<float>(0, i) = peaksR1[i - npoints0].x;
        coordR.at<float>(1, i) = peaksR1[i - npoints0].y;

#if(DEBUGOUT)
        pointsL.push_back(peaksL1[i - npoints0]);
        pointsR.push_back(peaksR1[i - npoints0]);
#endif
    }

    coordL.at<float>(0, npoints0 + npoints1) = pivotL.x;
    coordL.at<float>(1, npoints0 + npoints1) = pivotL.y;
    coordR.at<float>(0, npoints0 + npoints1) = pivotR.x;
    coordR.at<float>(1, npoints0 + npoints1) = pivotR.y;

#if(DEBUGOUT)
    pointsL.push_back(pivotL);
    pointsR.push_back(pivotR);
#endif    

    // Initialize projection matrices and intrinsics
    int outputSizeWidth = 1152, outputSizeHeight = 1152;
    float knew[3][3] =
    {
        {outputSizeWidth / 1.6f, 0, outputSizeWidth / 2.0f},
        {0, outputSizeHeight / 1.6f, outputSizeHeight / 2.0f},
        {0, 0, 1}
    };
    cv::Mat Knew = cv::Mat(3, 3, CV_32F, knew);

    float RTLd[3][4] =
    {
        {0.999988496878533,  -0.00408887531640368, -0.00250743079489038, 0},
        {0.00409079255504173,0.999991343892442,    0.000759971320900271, 0},
        {0.00250430166231527,-0.000770219958116164,0.999996567611299,    0}
    };

    float RTRd[3][4] =
    {
        {0.999940248657206,    -8.30767569805218E-06,-0.0109315619852459,  -0.0640309229493141  },
        {-2.71041674686418E-14,0.999999711221771,    -0.000759971302640667,0   },
        {0.0109315651420439,   0.00075992589333556,  0.999939959896231,    -0.000699999975040555  }
    };

    cv::Mat RtL = cv::Mat(3, 4, CV_32F, RTLd);
    cv::Mat RtR = cv::Mat(3, 4, CV_32F, RTRd);
    cv::Mat projL = Knew * RtL;
    cv::Mat projR = Knew * RtR;

    // Triangulate all points
    cv::Mat points4D;
    cv::triangulatePoints(projL, projR, coordL, coordR, points4D);

    // Trasform to Euclidean coordinates
    std::vector<cv::Point3f> points3D(npoints);
    for (int i = 0; i < npoints; i++)
    {
        float z = points4D.at<float>(3, i);
        if (std::abs(z) < 1e-7)
            z = 1.0f;

        points3D[i].x = points4D.at<float>(0, i) / z;
        points3D[i].y = points4D.at<float>(1, i) / z;
        points3D[i].z = points4D.at<float>(2, i) / z;
    }
    //cv::convertPointsFromHomogeneous(points4D.reshape(4, size), points3D);

#if(DEBUGOUT)
    if (visualize)
    {
        // Test
        for (int i = 0; i < points3D.size(); ++i)
        {
            cv::Mat testPoint = cv::Mat(cv::Vec4f(points3D[i].x, points3D[i].y, points3D[i].z, 1.0f));
            cv::Mat val = projL * testPoint;
            val.at<float>(0, 0) /= val.at<float>(2, 0);
            val.at<float>(1, 0) /= val.at<float>(2, 0);

            cv::drawMarker(imageDrawOnLeft, cv::Point(val.at<float>(0, 0), val.at<float>(1, 0)), cv::Scalar(0,255,0),0,(i < points3D.size() - 1)? 20: 50);
            cv::drawMarker(imageDrawOnLeft, pointsL[i], cv::Scalar(0,0,255),0,(i < points3D.size() - 1)? 20: 50);

            val = projR * testPoint;
            val.at<float>(0, 0) /= val.at<float>(2, 0);
            val.at<float>(1, 0) /= val.at<float>(2, 0);

            cv::drawMarker(imageDrawOnRight, cv::Point(val.at<float>(0, 0), val.at<float>(1, 0)), cv::Scalar(0,255,0),0,(i < points3D.size() - 1)? 20: 50);
            cv::drawMarker(imageDrawOnRight, pointsR[i], cv::Scalar(0,0,255),0,(i < points3D.size() - 1)? 20: 50);
        }

        cv::imshow("Triangulation Left", imageDrawOnLeft);
        cv::imshow("Triangulation Right", imageDrawOnRight);
        cv::waitKey(0);
        cv::imwrite("C:\\Users\\robertini\\Documents\\coding\\GreifbAR_Unity_DFKI\\Output\\left.png", imageDrawOnLeft);
        cv::imwrite("C:\\Users\\robertini\\Documents\\coding\\GreifbAR_Unity_DFKI\\Output\\right.png", imageDrawOnRight);
    }
#endif

    return points3D;
}

inline std::vector<cv::Point3f> Triangulate(const std::vector<cv::Point2f>& peaksL, const std::vector<cv::Point2f>& peaksR,
    const std::vector<std::pair<int,int>>& corr, cv::Mat& imageDrawOnLeft, cv::Mat& imageDrawOnRight, bool visualize = false)
{
    // Triangulate corresponding points

#if(DEBUGOUT)
    std::vector<cv::Point2f> pointsL, pointsR;
#endif

    int npoints = corr.size();
    cv::Mat coordL = cv::Mat(2, npoints, CV_32F);
    cv::Mat coordR = cv::Mat(2, npoints, CV_32F);

    for (int i = 0; i < corr.size(); ++i)
    {
        coordL.at<float>(0, i) = peaksL[corr[i].first].x;
        coordL.at<float>(1, i) = peaksL[corr[i].first].y;
        coordR.at<float>(0, i) = peaksR[corr[i].second].x;
        coordR.at<float>(1, i) = peaksR[corr[i].second].y;

#if(DEBUGOUT)
        pointsL.push_back(peaksL[corr[i].first]);
        pointsR.push_back(peaksR[corr[i].second]);
#endif

    }

    // Initialize projection matrices and intrinsics
    int outputSizeWidth = 1152, outputSizeHeight = 1152;
    float knew[3][3] =
    {
        {outputSizeWidth / 1.6f, 0, outputSizeWidth / 2.0f},
        {0, outputSizeHeight / 1.6f, outputSizeHeight / 2.0f},
        {0, 0, 1}
    };
    cv::Mat Knew = cv::Mat(3, 3, CV_32F, knew);

    float RTLd[3][4] =
    {
        {0.999988496878533,  -0.00408887531640368, -0.00250743079489038, 0},
        {0.00409079255504173,0.999991343892442,    0.000759971320900271, 0},
        {0.00250430166231527,-0.000770219958116164,0.999996567611299,    0}
    };

    float RTRd[3][4] =
    {
        {0.999940248657206,    -8.30767569805218E-06,-0.0109315619852459,  -0.0640309229493141  },
        {-2.71041674686418E-14,0.999999711221771,    -0.000759971302640667,0   },
        {0.0109315651420439,   0.00075992589333556,  0.999939959896231,    -0.000699999975040555  }
    };

    cv::Mat RtL = cv::Mat(3, 4, CV_32F, RTLd);
    cv::Mat RtR = cv::Mat(3, 4, CV_32F, RTRd);
    cv::Mat projL = Knew * RtL;
    cv::Mat projR = Knew * RtR;

    // Triangulate all points
    cv::Mat points4D;
    cv::triangulatePoints(projL, projR, coordL, coordR, points4D);

    // Trasform to Euclidean coordinates
    std::vector<cv::Point3f> points3D(npoints);
    for (int i = 0; i < npoints; i++)
    {
        float z = points4D.at<float>(3, i);
        if (std::abs(z) < 1e-7)
            z = 1.0f;

        points3D[i].x = points4D.at<float>(0, i) / z;
        points3D[i].y = points4D.at<float>(1, i) / z;
        points3D[i].z = points4D.at<float>(2, i) / z;
    }
    //cv::convertPointsFromHomogeneous(points4D.reshape(4, size), points3D);

#if(DEBUGOUT)
    if (visualize)
    {
        // Test
        for (int i = 0; i < points3D.size(); ++i)
        {
            cv::Mat testPoint = cv::Mat(cv::Vec4f(points3D[i].x, points3D[i].y, points3D[i].z, 1.0f));
            cv::Mat val = projL * testPoint;
            val.at<float>(0, 0) /= val.at<float>(2, 0);
            val.at<float>(1, 0) /= val.at<float>(2, 0);

            cv::drawMarker(imageDrawOnLeft, cv::Point(val.at<float>(0, 0), val.at<float>(1, 0)), cv::Scalar(0, 255, 0), 0, (i < points3D.size() - 1) ? 20 : 50);
            cv::drawMarker(imageDrawOnLeft, pointsL[i], cv::Scalar(0, 0, 255), 0, (i < points3D.size() - 1) ? 20 : 50);

            val = projR * testPoint;
            val.at<float>(0, 0) /= val.at<float>(2, 0);
            val.at<float>(1, 0) /= val.at<float>(2, 0);

            cv::drawMarker(imageDrawOnRight, cv::Point(val.at<float>(0, 0), val.at<float>(1, 0)), cv::Scalar(0, 255, 0), 0, (i < points3D.size() - 1) ? 20 : 50);
            cv::drawMarker(imageDrawOnRight, pointsR[i], cv::Scalar(0, 0, 255), 0, (i < points3D.size() - 1) ? 20 : 50);
        }

        cv::imshow("Triangulation Left", imageDrawOnLeft);
        cv::imshow("Triangulation Right", imageDrawOnRight);
        cv::waitKey(0);
        cv::imwrite("C:\\Users\\robertini\\Documents\\coding\\GreifbAR_Unity_DFKI\\Output\\left.png", imageDrawOnLeft);
        cv::imwrite("C:\\Users\\robertini\\Documents\\coding\\GreifbAR_Unity_DFKI\\Output\\right.png", imageDrawOnRight);
    }
#endif

    return points3D;
}

float* FindRope(float* img_left, float* img_right, int width, int height, int numChannels, int bitsPerChannel,
    float H0_max, float H0_min, float S0_max, float S0_min, float V0_max, float V0_min, 
    float H1_max, float H1_min, float S1_max, float S1_min, float V1_max, float V1_min)
{
    // Transform input to OpenCV readable data
    cv::Mat imageLeft = createMat(img_left, width, height, numChannels);
    cv::Mat imageRight = createMat(img_right, width, height, numChannels);

    // Convert to 8UC3
    cv::multiply(imageLeft, cv::Scalar(255, 255, 255), imageLeft);
    cv::multiply(imageRight, cv::Scalar(255, 255, 255), imageRight);
    imageLeft.convertTo(imageLeft, CV_8UC3);
    imageLeft.convertTo(imageLeft, CV_8UC3);
    imageRight.convertTo(imageRight, CV_8UC3);

    // Apply flipping along the Y-axis
    cv::flip(imageLeft, imageLeft, 0);
    cv::flip(imageRight, imageRight, 0);

    cv::imshow("ImageLeft", imageLeft);
    cv::imshow("ImageRight", imageRight);
    cv::waitKey(10);

    // Convert from RGB to HSV colorspace
    cv::Mat imageHSVLeft, imageHSVRight;
    cv::cvtColor(imageLeft, imageHSVLeft, cv::COLOR_RGB2HSV);
    cv::cvtColor(imageRight, imageHSVRight, cv::COLOR_RGB2HSV);

    // Apply thresholding
    cv::Mat maskLeft0, maskLeft1, maskRight0, maskRight1;
    cv::inRange(imageHSVLeft, cv::Scalar(H0_min, S0_min, V0_min), cv::Scalar(H0_max, S0_max, V0_max), maskLeft0);
    cv::inRange(imageHSVLeft, cv::Scalar(H1_min, S1_min, V1_min), cv::Scalar(H1_max, S1_max, V1_max), maskLeft1);
    cv::inRange(imageHSVRight, cv::Scalar(H0_min, S0_min, V0_min), cv::Scalar(H0_max, S0_max, V0_max), maskRight0);
    cv::inRange(imageHSVRight, cv::Scalar(H1_min, S1_min, V1_min), cv::Scalar(H1_max, S1_max, V1_max), maskRight1);

    // Filter mask
    int morph_size = 1;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1), cv::Point(morph_size, morph_size)); 

    // Opening - Closing
    morphologyEx(maskLeft0, maskLeft0, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 2); 
    morphologyEx(maskLeft1, maskLeft1, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 2); 
    morphologyEx(maskRight0, maskRight0, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 2); 
    morphologyEx(maskRight1, maskRight1, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 2);

    cv::Mat imageLeft00, imageLeft01, imageLeft02, imageLeft03, imageRight00, imageRight01, imageRight02, imageRight03;
    bool visualize = false;
#if(DEBUGOUT)
    imageLeft.copyTo(imageLeft00);
    imageLeft.copyTo(imageLeft01);
    imageLeft.copyTo(imageLeft02);
    imageLeft.copyTo(imageLeft03);
    imageRight.copyTo(imageRight00);
    imageRight.copyTo(imageRight01);
    imageRight.copyTo(imageRight02);
    imageRight.copyTo(imageRight03);
    visualize = false;
#endif

    // Find segments
    //std::vector<std::list<cv::Point2f>> segmentsL0, segmentsL1, segmentsR0, segmentsR1;
    std::vector<std::list<cv::Point2f>> segmentsL0 = FindSegments(maskLeft0, std::min(height, width) / 70.0f, imageLeft00, 3, visualize);
    std::vector<std::list<cv::Point2f>> segmentsR0 = FindSegments(maskRight0, std::min(height, width) / 70.0f, imageRight00, 3, visualize);
    std::vector<std::list<cv::Point2f>> segmentsL1 = FindSegments(maskLeft1, std::min(height, width) / 70.0f, imageLeft00, 3, visualize);
    std::vector<std::list<cv::Point2f>> segmentsR1 = FindSegments(maskRight1, std::min(height, width) / 70.0f, imageRight00, 3, visualize);
    
    // Connect segments
    //std::vector<std::list<cv::Point2f>> segments_connectedL0, segments_connectedL1, segments_connectedR0, segments_connectedR1;
    std::vector<std::list<cv::Point2f>> segments_connectedL0 = ConnectedLines(segmentsL0, imageLeft01, visualize);
    std::vector<std::list<cv::Point2f>> segments_connectedR0 = ConnectedLines(segmentsR0, imageRight01, visualize);
    std::vector<std::list<cv::Point2f>> segments_connectedL1 = ConnectedLines(segmentsL1, imageLeft01, visualize);
    std::vector<std::list<cv::Point2f>> segments_connectedR1 = ConnectedLines(segmentsR1, imageRight01, visualize);
    
    // Find pivot
    cv::Point2f pivotL = FindPivot(segments_connectedL0, segments_connectedL1, imageLeft02, visualize);
    cv::Point2f pivotR = FindPivot(segments_connectedR0, segments_connectedR1, imageRight02, visualize);

    // Compute expected peaks distance
    //std::vector<float> distL0 = ComputeExpectedPeakDistances(segments_connectedL0, maskLeft0);
    //std::vector<float> distL1 = ComputeExpectedPeakDistances(segments_connectedL1, maskLeft1);
    //std::vector<float> distR0 = ComputeExpectedPeakDistances(segments_connectedR0, maskRight0);
    //std::vector<float> distR1 = ComputeExpectedPeakDistances(segments_connectedR1, maskRight1);

    // Identify peaks
    //std::vector<cv::Point2f> peaksL0 = RetrievePeaks(segments_connectedL0, distL0, maskLeft0, imageLeft03, visualize);
    //std::vector<cv::Point2f> peaksL1 = RetrievePeaks(segments_connectedL1, distL1, maskLeft1, imageLeft03, visualize);
    //std::vector<cv::Point2f> peaksR0 = RetrievePeaks(segments_connectedR0, distR0, maskRight0, imageRight03, visualize);
    //std::vector<cv::Point2f> peaksR1 = RetrievePeaks(segments_connectedR1, distR1, maskRight1, imageRight03, visualize);

#if(DEBUGOUT)
    visualize = true;
#endif
    
    // Find peaks
    //std::vector<cv::Point2f> peaksL0 = FindPeaks(segments_connectedL0, imageLeft, imageLeft00, 170, 1, 10, visualize, "PeaksL0");
    //std::vector<cv::Point2f> peaksL1 = FindPeaks(segments_connectedL1, imageLeft, imageLeft01, 40, 1, 10, visualize, "PeaksL1");
    //std::vector<cv::Point2f> peaksR0 = FindPeaks(segments_connectedR0, imageRight, imageRight00, 170, 1, 10, visualize, "PeaksR0");
    //std::vector<cv::Point2f> peaksR1 = FindPeaks(segments_connectedR1, imageRight, imageRight01, 40, 1, 10, visualize, "PeaksR1");
    std::vector<cv::Point2f> peaksL0 = FindPeaksStandalone(segments_connectedL0, imageLeft, imageLeft00, 2, visualize, "PeaksL0");
    std::vector<cv::Point2f> peaksL1 = FindPeaksStandalone(segments_connectedL1, imageLeft, imageLeft01, 2, visualize, "PeaksL1");
    std::vector<cv::Point2f> peaksR0 = FindPeaksStandalone(segments_connectedR0, imageRight, imageRight00, 2, visualize, "PeaksR0");
    std::vector<cv::Point2f> peaksR1 = FindPeaksStandalone(segments_connectedR1, imageRight, imageRight01, 2, visualize, "PeaksR1");

    // Connect stereo peaks
    std::vector<std::pair<int, int>> corr0 = ConnectStereoPoints(peaksL0, peaksR0, 2);
    std::vector<std::pair<int, int>> corr1 = ConnectStereoPoints(peaksL1, peaksR1, 2);
    int npoints0 = corr0.size();
    int npoints1 = corr1.size();
    std::vector<cv::Point3f> points3D0 = Triangulate(peaksL0, peaksR0, corr0, imageLeft02, imageRight02, visualize);
    std::vector<cv::Point3f> points3D1 = Triangulate(peaksL1, peaksR1, corr1, imageLeft03, imageRight03, visualize);

    std::vector<cv::Point2f> pivotLl, pivotRl;
    pivotLl.push_back(pivotL);
    pivotRl.push_back(pivotR);
    std::vector<std::pair<int, int>> corr;
    corr.push_back(std::pair<int, int>(0, 0));
    std::vector<cv::Point3f> pivot = Triangulate(pivotLl, pivotRl, corr, imageLeft03, imageRight03, visualize);

    std::vector<cv::Point2f> corrL0, corrL1, corrR0, corrR1;
    //for (int i = 0; i < corr0.size(); ++i)
    //{
    //    corrL0.push_back(peaksL0[corr0[i].first]);
    //    corrL1.push_back(peaksR0[corr0[i].second]);
    //}
    //for (int i = 0; i < corr1.size(); ++i)
    //{
    //    corrR0.push_back(peaksL1[corr1[i].first]);
    //    corrR1.push_back(peaksR1[corr1[i].second]);
    //}

    // Triangulate
    //int npoints0 = 0; int npoints1 = 0;
    //std::vector<cv::Point3f> points3D = Triangulate(pivotL, pivotR, peaksL0, peaksL1, peaksR0, peaksR1, npoints0, npoints1, imageLeft03, imageRight03, visualize);
    //std::vector<cv::Point3f> points3D = Triangulate(pivotL, pivotR, corrL0, corrL1, corrR0, corrR1, npoints0, npoints1, imageLeft03, imageRight03, visualize);

    // -------------------------------------------------------------------------------------------
    // Generate resulting output
    float* result = new float[2 + (npoints0 + npoints1 + 1) * 3]; // plus two dimensions
    result[0] = npoints0;
    result[1] = npoints1;
    for (int i = 0; i < npoints0; ++i)
    {
        result[2 + 3 * i + 0] = points3D0[i].x;
        result[2 + 3 * i + 1] = points3D0[i].y;
        result[2 + 3 * i + 2] = points3D0[i].z;
    }
    for (int i = 0; i < npoints1; ++i)
    {
        result[2 + 3 * npoints0 + 3 * i + 0] = points3D1[i].x;
        result[2 + 3 * npoints0 + 3 * i + 1] = points3D1[i].y;
        result[2 + 3 * npoints0 + 3 * i + 2] = points3D1[i].z;
    }

    result[(npoints0 + npoints1 + 1) * 3 - 1] = pivot[0].x;
    result[(npoints0 + npoints1 + 1) * 3 + 0] = pivot[0].y;
    result[(npoints0 + npoints1 + 1) * 3 + 1] = pivot[0].z;

    return result;
    
    //float* result = new float[2 + (0 + 0 + 1) * 3]; // plus two dimensions
    //return nullptr;
}

/// <summary>
/// Triangulate 2D pixel coordinates given stereo projection matrices
/// </summary>
/// <param name="coordLeft">list of pixel coordinates on the left view</param>
/// <param name="coordRight">list of pixel coordinates on the right view</param>
/// <param name="size">Number of coordinates for left and right. Notice that they should match.</param>
/// <param name="projMatrixLeft">Projection matrix for the left camera assumed to be of size 3x4</param>
/// <param name="projMatrixRight">Projection matrix for the right camera assumed to be of size 3x4</param>
/// <returns></returns>
extern "C" EXPORT float* Triangulate2DPoints(float* coordLeft, float* coordRight, unsigned int size, float* projMatrixLeft, float* projMatrixRight)
{
    cv::Mat projL, projR;
    cv::Mat coordL, coordR;
    cv::Mat points4D, points3D;

    try
    {
        // Transform input to OpenCV readable data
        projL = cv::Mat(3, 4, CV_32F, projMatrixLeft);
        projR = cv::Mat(3, 4, CV_32F, projMatrixRight);

        coordL = cv::Mat(2, size, CV_32F, coordLeft);
        coordR = cv::Mat(2, size, CV_32F, coordRight);

        // Triangulate
        cv::triangulatePoints(projL, projR, coordL, coordR, points4D);

        // Trasform to Euclidean coordinates
        points3D = cv::Mat(3, size, CV_32F);
        for (int i = 0; i < size; i++)
        {
            float z = points4D.at<float>(3, i);
            if (std::abs(z) < 1e-7)
                z = 1.0f;

            points3D.at<float>(0, i) = points4D.at<float>(0, i) / z;
            points3D.at<float>(1, i) = points4D.at<float>(1, i) / z;
            points3D.at<float>(2, i) = points4D.at<float>(2, i) / z;
        }
        //cv::convertPointsFromHomogeneous(points4D.reshape(4, size), points3D);
    }
    catch (cv::Exception& e)
    {
#if(DEBUGOUT)
        std::cout << "Error while triangulating " << e.what() << "\n";
#endif

        return nullptr;
    }

    return convertToPointer(points3D);
}

extern "C" EXPORT float* Triangulate2DPoints0(float* coordLeft, float* coordRight, unsigned int size)
{
    cv::Mat coordL = cv::Mat(2, size, CV_32F);
    cv::Mat coordR = cv::Mat(2, size, CV_32F);

    for (int i = 0; i < size; ++i)
    {
        coordL.at<float>(0, i) = coordLeft[i * 2 + 0];
        coordL.at<float>(1, i) = coordLeft[i * 2 + 1];
        coordR.at<float>(0, i) = coordRight[i * 2 + 0];
        coordR.at<float>(1, i) = coordRight[i * 2 + 1];
    }

    // Initialize projection matrices and intrinsics
    int outputSizeWidth = 1152, outputSizeHeight = 1152;
    float knew[3][3] =
    {
        {outputSizeWidth / 1.6f, 0, outputSizeWidth / 2.0f},
        {0, outputSizeHeight / 1.6f, outputSizeHeight / 2.0f},
        {0, 0, 1}
    };
    cv::Mat Knew = cv::Mat(3, 3, CV_32F, knew);

    float RTLd[3][4] =
    {
        {0.999988496878533,  -0.00408887531640368, -0.00250743079489038, 0},
        {0.00409079255504173,0.999991343892442,    0.000759971320900271, 0},
        {0.00250430166231527,-0.000770219958116164,0.999996567611299,    0}
    };

    float RTRd[3][4] =
    {
        {0.999940248657206,    -8.30767569805218E-06,-0.0109315619852459,  -0.0640309229493141  },
        {-2.71041674686418E-14,0.999999711221771,    -0.000759971302640667,0   },
        {0.0109315651420439,   0.00075992589333556,  0.999939959896231,    -0.000699999975040555  }
    };

    cv::Mat RtL = cv::Mat(3, 4, CV_32F, RTLd);
    cv::Mat RtR = cv::Mat(3, 4, CV_32F, RTRd);
    cv::Mat projL = Knew * RtL;
    cv::Mat projR = Knew * RtR;

    // Triangulate all points
    cv::Mat points4D;
    cv::triangulatePoints(projL, projR, coordL, coordR, points4D);

    // Trasform to Euclidean coordinates
    std::vector<cv::Point3f> points3D(size);
    for (int i = 0; i < size; i++)
    {
        float z = points4D.at<float>(3, i);
        if (std::abs(z) < 1e-7)
            z = 1.0f;

        points3D[i].x = points4D.at<float>(0, i) / z;
        points3D[i].y = points4D.at<float>(1, i) / z;
        points3D[i].z = points4D.at<float>(2, i) / z;
    }

    return convertToPointer(points3D);
}

float* GetRegions(unsigned char* img, int width, int height, int numChannels, int bitsPerChannel, float H_max, float H_min, float S_max, float S_min, float V_max, float V_min, float min_area, int max_regions, const char* windowName = "Display")
{
    // Transform input to OpenCV readable data
    cv::Mat image = createMat(img, width, height, numChannels, bitsPerChannel);
    cv::Mat imageHSV = cv::Mat();
    cv::Mat mask = cv::Mat();//, maskInv = cv::Mat(), maskC4 = cv::Mat();

    // Convert from BGR to HSV colorspace
    cv::cvtColor(image, imageHSV, cv::COLOR_RGB2HSV);

    // Apply thresholding
    cv::inRange(imageHSV, cv::Scalar(H_min, S_min, V_min), cv::Scalar(H_max, S_max, V_max), mask);

    // Filter mask
    int morph_size = 1;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * morph_size + 1, 2 * morph_size + 1), cv::Point(morph_size, morph_size));

    // Opening - Closing
    morphologyEx(mask, mask, cv::MORPH_CLOSE, element, cv::Point(-1, -1), 2);

    // Detect regions of minimal size
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //std::vector<cv::Rect> found;
    std::vector<cv::RotatedRect> found;
    //std::vector<cv::Scalar> colors;
    for (int i = 0; i < contours.size(); ++i)
    {
        //cv::Rect rect = cv::boundingRect(contours[i]);
        cv::RotatedRect rect = cv::minAreaRect(contours[i]);

        if (found.size() >= max_regions)
            continue;

        //if (rect.area() < min_area)
        if (rect.size.width * rect.size.height < min_area)
            continue;

        // ToDo: Remove if its center is outside the masked image

        // add found rectangle to the list
        found.push_back(rect);

        // add color of the rectangle
        //cv::Mat maskRect = cv::Mat::zeros(image.rows, image.cols, CV_8U); // all 0
        //maskRect(rect) = 1;
        //cv::Mat mask2;
        //cv::bitwise_and(maskRect, mask, mask2); // maskRectangle & maskInRange
        //cv::Scalar color = cv::mean(imageHSV, mask2);

        //colors.push_back(color);//cv::Scalar(0,0,0));

        ///////////////////////////////////////////////////////////////////////////////////////////////
        // Debug: Draw rectangles on the image:
        cv::Point2f vertices2f[4];
        rect.points(vertices2f);

        // Convert them so we can use them in a fillConvexPoly
        cv::Point vertices[4];    
        for(int i = 0; i < 4; ++i){
            vertices[i] = vertices2f[i];
        }

        // Now we can fill the rotated rectangle with our specified color
        cv::fillConvexPoly(imageHSV, vertices, 4, cv::Scalar(0,0,0));
        ///////////////////////////////////////////////////////////////////////////////////////////////
    }

    // Debug
    cv::imshow(std::string(windowName) + " mask", mask);
    cv::imshow(std::string(windowName) + " contours", imageHSV);
	cv::waitKey(0);





    // convert to floating points
    const int size = 11;
    float* result = new float[1 + found.size() * size];
    result[0] = found.size();
    for (int i = 0; i < found.size(); ++i)
    {

        //std::vector<cv::Point2f> points;
        //found[i].points(points);

        result[1 + i * size + 0] = found[i].center.x;
        result[1 + i * size + 1] = found[i].center.y;
        result[1 + i * size + 2] = found[i].angle;//std::max(found[i].size.width, found[i].height) / 2.0f;
        //result[1 + i * size + 3] = points[0].x;
        //result[1 + i * size + 4] = points[0].y;
        //result[1 + i * size + 5] = points[1].x;
        //result[1 + i * size + 6] = points[1].y;
        //result[1 + i * size + 7] = points[2].x;
        //result[1 + i * size + 8] = points[2].y;
        //result[1 + i * size + 9] = points[3].x;
        //result[1 + i * size + 10] = points[3].y;
        
        //result[1 + i * size + 0] = found[i].x;
        //result[1 + i * size + 1] = found[i].y;
        //result[1 + i * size + 2] = std::max(found[i].size.width, found[i].height) / 2.0f;
        //result[1 + i * size + 3] = colors[i][0];
        //result[1 + i * size + 4] = colors[i][1];
        //result[1 + i * size + 5] = colors[i][2];
    }

    return result;
}

float* DetectFeatures(unsigned char* imgLeft, unsigned char* imgRight, int width, int height, int numChannels, int bitsPerChannel, float H_max, float H_min, float S_max, float S_min, float V_max, float V_min)
{
    // Convert to OpenCV readable
    cv::Mat left = createMat(imgLeft, width, height, numChannels, bitsPerChannel);
    cv::Mat right = createMat(imgRight, width, height, numChannels, bitsPerChannel);

    // Convert from RGB to HSV colorspace
    cv::Mat leftHSV, rightHSV;
    cv::cvtColor(left, leftHSV, cv::COLOR_RGB2HSV);
    cv::cvtColor(right, rightHSV, cv::COLOR_RGB2HSV);

    // Apply thresholding and find non-zero locations
    cv::Mat leftMask, rightMask;
    cv::inRange(leftHSV, cv::Scalar(H_min, S_min, V_min), cv::Scalar(H_max, S_max, V_max), leftMask);
    cv::inRange(rightHSV, cv::Scalar(H_min, S_min, V_min), cv::Scalar(H_max, S_max, V_max), rightMask);

    // Convert to grayscale
    cv::Mat leftGray, rightGray;
    cv::cvtColor(left, leftGray, cv::COLOR_RGB2GRAY);
    cv::cvtColor(right, rightGray, cv::COLOR_RGB2GRAY);

    // Detect features on the grayscale images
    cv::Ptr<cv::Feature2D> orb = cv::ORB::create(200);
    std::vector<cv::KeyPoint> TargetKeypoints, ReferenceKeypoints;
    cv::Mat TargetDescriptor, ReferenceDescriptor;

    orb->detectAndCompute(leftGray, leftMask, ReferenceKeypoints, ReferenceDescriptor);
    orb->detectAndCompute(rightGray, rightMask, TargetKeypoints, TargetDescriptor);

    // Draw resulting features
    cv::Mat Result0, Result1;
    cv::drawKeypoints(leftGray, ReferenceKeypoints, Result0);
    cv::drawKeypoints(rightGray, TargetKeypoints, Result1);
    cv::imwrite("C:\\Users\\robertini\\Documents\\coding\\GreifbAR_Unity_DFKI\\Output\\feature_left.png", Result0);
    cv::imwrite("C:\\Users\\robertini\\Documents\\coding\\GreifbAR_Unity_DFKI\\Output\\feature_right.png", Result1);

    // Match features
    std::vector<cv::DMatch> matches;
    cv::Ptr<cv::DescriptorMatcher> Matcher_ORB = cv::BFMatcher::create(cv::NORM_HAMMING);
    Matcher_ORB->match(TargetDescriptor, ReferenceDescriptor, matches);

    std::sort(matches.begin(), matches.end());
    const int match_size = matches.size();
    std::vector<cv::DMatch> good_matches(matches.begin(), matches.begin() + (int)(match_size * 0.5f));

    // Draw resulting matches
    cv::Mat Result;
    cv::drawMatches(rightGray, TargetKeypoints, leftGray, ReferenceKeypoints, matches, Result, cv::Scalar::all(-1), cv::Scalar(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::imwrite("C:\\Users\\robertini\\Documents\\coding\\GreifbAR_Unity_DFKI\\Output\\match.png", Result);

    return nullptr;
}