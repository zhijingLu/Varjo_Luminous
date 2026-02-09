#pragma once

#include "Varjo.h"
#include "Varjo_datastream.h"
#include "Varjo_mr_experimental.h"

void checkVarjoError(varjo_Session* session);

void onPointcloudStreamFrame(varjo_Session* session, void* userData);

void StartStreamingPointcloud(varjo_Session* session);

void onMeshStreamFrame(varjo_Session* session, void* userData);

void StartStreamingMesh(varjo_Session* session);