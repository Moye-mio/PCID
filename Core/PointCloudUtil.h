#pragma once

namespace core {

struct SAABB;

using P_t = pcl::PointXYZ;
using PT_t = pcl::PointCloud<P_t>;

class PointCloudUtil {
public:
	static SAABB calcAABB(const PC_t::Ptr vCloud);
	static float calcChamferDistance(const PC_t::Ptr c1, const PC_t::Ptr c2);
	static void calcChamferDistance(const PC_t::Ptr c1, const PC_t::Ptr c2, bool isBoth = true);
	static bool calcNormal(const PC_t::Ptr vioCloud, std::uint32_t k);
	static void normalize(const PC_t::Ptr vioCloud);
	static void normalizeByFixScale(const PC_t::Ptr vioCloud, float vScale = 1.0f);
	static void normalizeByReference(const PC_t::Ptr vioCloud, const PC_t::Ptr vRefer);
	static void scale(const PC_t::Ptr vioCloud, float vScale = 1.0f);
	static void extractXYZpt(const PC_t::Ptr& vCloud, const PT_t::Ptr& voPts);
};

}
