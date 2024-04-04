#ifndef PCH_H
#define PCH_H

#include "framework.h"

/* STL */
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <cstdlib>
#include <memory>
#include <optional>
#include <numeric>
#include <numbers>
#include <variant>

/* OMP */
#include <omp.h>

/* BOOST */
#include <boost/format.hpp>

/* EIGEN */
#include <Eigen/eigen>

/* OPENCV */
#include <opencv2/opencv.hpp>

/* PCL */
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>

/* COMMON */
#include "Common.h"

#endif //PCH_H