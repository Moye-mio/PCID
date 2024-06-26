//
// pch.h
//

#pragma once

#include "gtest/gtest.h"

/* STL */
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <variant>

/* BOOST */
#include <boost/format.hpp>

/* OPENCV */
#include <opencv2/opencv.hpp>

/* COMMON */
#include "Common.h"
#include "magic_enum.hpp"

/* CORE */
#include "HeightMap.h"
#include "MaskMap.h"
#include "GradientMap.h"
#include "MapUtil.h"
#include "SparseLinearSolver.h"
#include "SolverBuilder.h"
#include "EigenUtil.h"
#include "MapWrapper.h"
