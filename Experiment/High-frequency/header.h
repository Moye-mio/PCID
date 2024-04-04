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
#include "MapUtil.h"
#include "MapWrapper.h"
#include "SolverBuilder.h"
#include "SparseLinearSolver.h"
#include "EigenUtil.h"

/* ALG */
#include "AlgCommon.h"
#include "MixInpainting.h"
#include "PoissonImageInpainting.h"
