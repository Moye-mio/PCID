#pragma once

#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>

namespace core {

class CNurbsFitting {
public:

	void setSavePath(const std::string& vPath) { m_Path = vPath; }
	bool run(const PC_t::Ptr& vCloud, int vDegree, int vRefinement, int vIters);
	const std::shared_ptr<pcl::on_nurbs::FittingSurface> getFit() const { return m_Fit; }

private:
	void __calcInitCorner(const PC_t::Ptr& vCloud, std::vector<ON_3dPoint>& voCorner);
	void __PCLPoint2Vector3d(const PC_t::Ptr& vCloud, pcl::on_nurbs::vector_vec3d& voData);
	void __saveControlPointsToDisk();

private:
	std::shared_ptr<pcl::on_nurbs::FittingSurface> m_Fit;
	std::string m_Path;

};

}
