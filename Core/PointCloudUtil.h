#pragma once

namespace core {

struct SAABB;

using P_t = pcl::PointXYZ;
using PT_t = pcl::PointCloud<P_t>;

class PointCloudUtil {
public:
	static SAABB calcAABB(const PC_t::Ptr vCloud);
	static bool calcNormal(const PC_t::Ptr vioCloud, std::uint32_t k);

private:
	static void __extractXYZpt(const PC_t::Ptr& vCloud, const PT_t::Ptr& voPts);
};

}
