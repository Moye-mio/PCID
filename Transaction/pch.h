﻿// pch.h: 这是预编译标头文件。
// 下方列出的文件仅编译一次，提高了将来生成的生成性能。
// 这还将影响 IntelliSense 性能，包括代码完成和许多代码浏览功能。
// 但是，如果此处列出的文件中的任何一个在生成之间有更新，它们全部都将被重新编译。
// 请勿在此处添加要频繁更新的文件，这将使得性能优势无效。

#ifndef PCH_H
#define PCH_H

// 添加要在此处预编译的标头
#include "framework.h"

#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <cstdlib>
#include <variant>
#include <numbers>
#include <chrono>

/* CV */
#include <opencv2/opencv.hpp>

/* COMMON */
#include "Common.h"

/* IO */
#include "PCLoader.h"
#include <pcl/io/ply_io.h>

/* CORE */
#include "AABB.h"
#include "MapUtil.h"
#include "HeightMap.h"
#include "GradientMap.h"
#include "MaskMap.h"
#include "HeightMapGenerator.h"
#include "HeightMapSampler.h"
#include "MapWrapper.h"
#include "PCMapper.h"
#include "SolverBuilder.h"
#include "SparseLinearSolver.h"
#include "EigenUtil.h"
#include "PointCloudUtil.h"
#include "DuplicateRemover.h"
#include "Proj.h"
#include "SurfaceUtil.h"
#include "NurbsFitting.h"
#include "ProjManager.h"

/* PATCHMATCH */
#include "AlgCommon.h"
#include "ImageInpainting.h"
#include "ImageUtil.h"
#include "PoissonImageInpainting.h"

#endif //PCH_H
