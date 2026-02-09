
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <iostream>
#include <vector>
#include <Windows.h>
#include "Varjo.h"
#include "Varjo_datastream.h"
#include "Varjo_mr_experimental.h"
#include "Pointcloud.h"
#include "UserData.h"

constexpr uint32_t c_floatsPerVertex = 9;  // 3 for position, 3 for normal, 3 for color
constexpr int32_t c_maxChunkUpdatesPerFrame = 64;

float gammaEncode(float color) { return (std::min)(1.0f, pow(color, 1.0f / 2.2f)); }

void checkVarjoError(varjo_Session* session)
{
    varjo_Error error = varjo_GetError(session);

    if (error != varjo_NoError) {
        const char* errorDesc = varjo_GetErrorDesc(error);
        std::cout << "Got varjo error " << error << ": '" << errorDesc << "'";
        throw std::runtime_error(errorDesc);
    }
}

void onPointcloudStreamFrame(varjo_Session* session, void* userData)
{
    // schedule capture of pointcloud snapshot (starts cumulating deltas)
    varjo_PointCloudSnapshotId id = varjo_MRBeginPointCloudSnapshot(session);
    checkVarjoError(session);
    Sleep(1000); ///

    varjo_PointCloudSnapshotStatus status = varjo_MRGetPointCloudSnapshotStatus(session, id);
    checkVarjoError(session);

    //myData.pc_vertices[0] = 1;
    //myData.pc_vertices[1] = id;
    //myData.pc_vertices[2] = myData.pc_conf.framerate;
    //myData.pc_vertices[3] = status;
     
    if (status == varjo_PointCloudSnapshotStatus_Ready)
    {
        // get pointcloud content
        struct varjo_PointCloudSnapshotContent content;
        varjo_MRGetPointCloudSnapshotContent(session, id, &content);
        checkVarjoError(session);

        // initializing the internal structures
        myData.pc_vertices[0] = std::min(content.pointCount, 100000 / 3 - 1);
        //myData.pc_vertices[3] = content.pointCount;

        for (int i = 0; i < myData.pc_vertices[0]; i++)
        {
            varjo_PointCloudPoint pt = content.points[i];

            uint32_t confidence = pt.indexConfidence;
            float x = pt.positionXY & ~((1 << 16) - 1);
            float y = pt.positionXY << 16;
            float z = pt.positionZradius & ~((1 << 16) - 1);
            float radius = pt.positionZradius << 16;
            float blue = pt.colorBG & ~((1 << 16) - 1);
            float green = pt.colorBG << 16;
            float red = pt.normalZcolorR << 16;
            float nx = pt.normalXY & ~((1 << 16) - 1);
            float ny = pt.normalXY << 16;
            float nz = pt.normalZcolorR & ~((1 << 16) - 1);

            myData.pc_vertices[1 + i * 3] = x;
            myData.pc_vertices[1 + i * 3 + 1] = y;
            myData.pc_vertices[1 + i * 3 + 2] = z;
            // TODO: store the remaining data as well
        }

        // release its content
        varjo_MRReleasePointCloudSnapshot(session, id);
        checkVarjoError(session);
    }

    // stop streaming of pointcloud snapshot and eventual deltas
    varjo_MREndPointCloudSnapshot(session, id);
    checkVarjoError(session);
}

void onMeshStreamFrame(varjo_Session* session, void* userData)
{
    //UserData* uData = (UserData*)userData;

    // If we are about to start the next update round, fetch latest chunk descriptions
    if (myData.pc_curChunkIndex == 0) {
        // Retrieve descriptions of all chunks that were updated since we last updated the chunks
        int64_t curTime = varjo_GetCurrentTime(session);
        varjo_MRGetMeshChunkDescriptions(session, myData.pc_lastChunkUpdateTime, myData.pc_chunkDescriptions.data(), myData.mr_conf.maxChunks, &myData.pc_chunkCount);
        checkVarjoError(session);

        // Store the last time there were actually updates to any chunks
        if (myData.pc_chunkCount > 0) {
            myData.pc_lastChunkUpdateTime = curTime;
        }
    }

    if (myData.pc_chunkCount > 0) {
        // Update only certain maximum number of chunks per frame
        int startIndex = myData.pc_curChunkIndex;
        int endIndex = std::min(myData.pc_curChunkIndex + c_maxChunkUpdatesPerFrame, myData.pc_chunkCount);

        std::vector<varjo_Vector3Di> chunksOfInterest;

        for (int i = startIndex; i < endIndex; ++i) {
            chunksOfInterest.push_back(myData.pc_chunkDescriptions[i].position);
        }

        // Lock the chunk contents buffers for the chunks of interest
        varjo_MeshChunkContentsBufferId bufferId =
            varjo_MRLockMeshChunkContentsBuffer(session, chunksOfInterest.data(), static_cast<int32_t>(chunksOfInterest.size()));
        checkVarjoError(session);

        if (bufferId != varjo_ChunkContentsBufferId_Invalid) {
            std::vector<varjo_MeshChunkContent> chunkContents(chunksOfInterest.size());

            // Access the chunk contents buffer
            varjo_MRGetMeshChunkContentsBufferData(session, bufferId, chunkContents.data());
            checkVarjoError(session);

            // First count number of vertices in total for the correct allocations
            myData.pc_vertices_size = 0;
            myData.pc_idx_size = 0;
            for (uint32_t i = 0; i < chunkContents.size(); ++i) {
                const auto& chunk = chunkContents[i];
                myData.pc_vertices_size += chunk.description.vertexCount;
                myData.pc_idx_size += chunk.description.triangleCount;
            }

            myData.pc_vertices[0] = myData.pc_vertices_size;
            myData.pc_normals[0] = myData.pc_vertices_size;
            myData.pc_colors[0] = myData.pc_vertices_size;
            myData.pc_idx[0] = myData.pc_idx_size;
            int index = 0, index_i = 0;

            for (uint32_t i = 0; i < chunkContents.size(); ++i) {
                const auto& chunk = chunkContents[i];

                // Fetch vertex data
                //m_data.vertexDataBuffer.resize(chunk.description.vertexCount * c_floatsPerVertex);
                //uint32_t offset = 0;

                for (uint32_t i = 0; i < chunk.description.vertexCount; ++i, ++index) {
                    const varjo_Vector3Df& pos = chunk.vertexPositions.positions32f[i];
                    const varjo_Vector3Df& normal = chunk.vertexNormals.normals32f[i];
                    const varjo_Vector3Df& color = chunk.vertexColors.colors32f[i];

                    myData.pc_vertices[1 + index * 3] = pos.x;
                    myData.pc_vertices[1 + index * 3 + 1] = pos.y;
                    myData.pc_vertices[1 + index * 3 + 2] = pos.z;

                    myData.pc_normals[1 + index * 3] = normal.x;
                    myData.pc_normals[1 + index * 3 + 1] = normal.y;
                    myData.pc_normals[1 + index * 3 + 2] = normal.z;

                    myData.pc_colors[1 + index * 3] = (int)std::lround(255 * gammaEncode(color.x));
                    myData.pc_colors[1 + index * 3 + 1] = (int)std::lround(255 * gammaEncode(color.y));
                    myData.pc_colors[1 + index * 3 + 2] = (int)std::lround(255 * gammaEncode(color.z));
                }

                // Fetch index data
                uint32_t indexCount = chunk.description.triangleCount * 3;
                //myData.pc_idx = new int[indexCount];
                for (uint32_t i = 0; i < indexCount; i++)
                    myData.pc_idx[1 + index_i + i] = chunk.triangleIndices[i] + index_i;
                index_i += indexCount;
                //memcpy(myData.pc_idx, chunk.triangleIndices, indexCount * sizeof(uint32_t));

                /*
                for (uint32_t i = 0; i < chunk.description.vertexCount; ++i) {
                    const varjo_Vector3Df& pos = chunk.vertexPositions.positions32f[i];
                    const varjo_Vector3Df& normal = chunk.vertexNormals.normals32f[i];
                    const varjo_Vector3Df& color = chunk.vertexColors.colors32f[i];

                    memcpy(&m_data.vertexDataBuffer[offset], &pos, 3 * sizeof(float));
                    offset += 3;
                    memcpy(&m_data.vertexDataBuffer[offset], &normal, 3 * sizeof(float));
                    offset += 3;
                    memcpy(&m_data.vertexDataBuffer[offset], &color, 3 * sizeof(float));
                    offset += 3;
                }

                // Fetch index data
                uint32_t indexCount = chunk.description.triangleCount * 3;
                m_data.indexDataBuffer.resize(indexCount);
                memcpy(m_data.indexDataBuffer.data(), chunk.triangleIndices, indexCount * sizeof(uint32_t));

                // Update chunk mesh with new data
                MeshChunk& meshChunk = getChunk(chunk.description.position);
                meshChunk.updateTimestamp = m_data.lastChunkUpdateTime;
                //m_renderer.updateMesh(*meshChunk.mesh, m_data.vertexDataBuffer, m_data.indexDataBuffer);
                */
            }

            // Unlock chunk contents buffer.
            varjo_MRUnlockMeshChunkContentsBuffer(session, bufferId);
            checkVarjoError(session);
        }

        // Update chunk index
        if (endIndex == myData.pc_chunkCount) {
            // Reset the chunk index counter if we got to the end of the chunk list
            myData.pc_curChunkIndex = 0;
        }
        else {
            myData.pc_curChunkIndex = endIndex;
        }
    }
}

void StartStreamingMesh(varjo_Session* session)
{
    try {
        // Retrieve mesh reconstruction configuration
        myData.mr_conf = varjo_MRGetMeshReconstructionConfig(session);
        checkVarjoError(session);

        //if (isValid(myData.mr_conf.maxChunks > 0)) { // if it is valid
        //    myData.m_data = {};
        //}

        // Set data config and pre-allocate arrays
        //m_data.config = m_config;
        //m_data.meshChunks.reserve(m_config.maxChunks);
        myData.pc_chunkDescriptions.resize(myData.mr_conf.maxChunks);

        // Delete eventually already initialized data structures
        delete[] myData.pc_vertices;
        delete[] myData.pc_normals;
        delete[] myData.pc_colors;
        delete[] myData.pc_idx;
        myData.pc_vertices_size = 0;
        myData.pc_idx_size = 0;

        // Initialize all data structures
        myData.pc_vertices = new float[1 + myData.mr_conf.maxChunks * myData.mr_conf.maxVerticesPerChunk * 3];
        myData.pc_normals = new float[1 + myData.mr_conf.maxChunks * myData.mr_conf.maxVerticesPerChunk * 3];
        myData.pc_colors = new int[1 + myData.mr_conf.maxChunks * myData.mr_conf.maxVerticesPerChunk * 3];
        myData.pc_idx = new int[1 + myData.mr_conf.maxChunks * myData.mr_conf.maxTrianglesPerChunk * 3];
        myData.pc_vertices[0] = 0; // the first index contains the actual number of vertices
        myData.pc_normals[0] = 0;
        myData.pc_colors[0] = 0;
        myData.pc_idx[0] = 0;
        myData.pc_vertices_size = 0; // nothing stored yet
        myData.pc_idx_size = 0; // nothing stored yet

        //std::cout << "OK (Meshing running)!\n";
    }
    catch (const std::runtime_error& e) {
        //LOG_ERROR("Initializing mesh scene failed: %s", e.what());
        //m_statusText = "Meshing initialization failed.";

        // Reset config to indicate invalid state
        //m_config = {};

        std::cout << "Error (Meshing initialization failed) =" << e.what() << "\n";

        //return false;
    }
}

//varjo_PointCloudSnapshotId id = varjo_PointCloudSnapshotId_Invalid;

void StartStreamingPointcloud(varjo_Session* session)
{
    try {
        // Retrieve mesh reconstruction configuration
        myData.pc_conf = varjo_MRGetReconstructionConfig(session);
        checkVarjoError(session);

        // Delete eventually already initialized data structures
        delete[] myData.pc_vertices;
        delete[] myData.pc_normals;
        delete[] myData.pc_colors;
        myData.pc_vertices_size = 0;

        // Initialize all data structures
        myData.pc_vertices = new float[100000 * 3]; // TODO: Use the correct maximum size!
        myData.pc_normals = new float[100000 * 3];
        myData.pc_colors = new int[100000 * 3];
        myData.pc_vertices[0] = 0; // the first index contains the actual number of vertices
        myData.pc_normals[0] = 0;
        myData.pc_colors[0] = 0;
        myData.pc_vertices_size = 0; // nothing stored yet
    }
    catch (const std::runtime_error& e) {

        std::cout << "Error (Pointcloud initialization failed) =" << e.what() << "\n";
    }
}