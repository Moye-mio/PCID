#pragma once

/* STL */
#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <cstdlib>
#include <variant>
#include <chrono>

/* PCL */
#include <pcl/io/ply_io.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>

/* BOOST */
#include <boost/format.hpp>

/* CV */
#include <opencv2/opencv.hpp>

/* COMMON */
#include "Common.h"

/* CORE */
#include "Map.hpp"
#include "HeightMap.h"
#include "MaskMap.h"
#include "GradientMap.h"
#include "AABB.h"
#include "MapUtil.h"
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

/* IO */
#include "IOInterface.hpp"

/* DGI */
#include "PCID.h"
#include "DGI.h"

