#pragma once

#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include "Proj.h"

namespace core {

class CSurfaceUtil {
public:

	static PC_t::Ptr sampleSurface(const std::shared_ptr<pcl::on_nurbs::FittingSurface> vFit, int vSampleRate);
	static bool calcProjPoints(const std::shared_ptr<pcl::on_nurbs::FittingSurface> vFit, const PC_t::Ptr vCloud, std::vector<SProj>& voProjs);


private:
	static void __geneSampleSequence(int vSampleRate, std::vector<vec2f>& voSeq);
	static SProj __calcProj(const std::shared_ptr<pcl::on_nurbs::FittingSurface> vFit, const Point_t& vPt, const vec2f& vHint);
};

}
