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
#include <fstream>

/* PCL */
#include <pcl/io/ply_io.h>

/* BOOST */
#include <boost/format.hpp>

/* CV */
#include <opencv2/opencv.hpp>

/* COMMON */
#include "Common.h"

/* CORE */
#include "MapUtil.h"
#include "HeightMap.h"
#include "GradientMap.h"
#include "MaskMap.h"
#include "HeightMapGenerator.h"
#include "MapWrapper.h"
#include "SolverBuilder.h"
#include "SparseLinearSolver.h"
#include "EigenUtil.h"
#include "DensityEstimator.h"
#include "SurfaceUtil.h"
#include "Proj.h"
#include "PointCloudUtil.h"
#include "AABB.h"
#include "DuplicateRemover.h"

/* ALG */
#include "ImageUtil.h"
#include "PoissonImageInpainting.h"