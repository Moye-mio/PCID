#include "pch.h"
#include "SurfaceUtil.h"

using namespace core;

PC_t::Ptr CSurfaceUtil::sampleSurface(const std::shared_ptr<pcl::on_nurbs::FittingSurface> vFit, int vSampleRate) {
	PC_t::Ptr pCloud(new PC_t);
	_EARLY_RETURN(!vFit, "surface sample : surface is null", pCloud);
	_EARLY_RETURN(vSampleRate < 0, "surface sample : sample rate < 0", pCloud);

	std::vector<vec2f> Seq;
	__geneSampleSequence(vSampleRate, Seq);

	const auto& Nurbs = vFit->m_nurbs;
	for (const auto& e : Seq) {
		const auto p = Nurbs.PointAt(e.x, e.y);
		pCloud->emplace_back(Point_t((float)p.x, (float)p.y, (float)p.z, (std::uint8_t)0, (std::uint8_t)0, (std::uint8_t)0, e.x, e.y, 0.0f));		/* save uv in normal_xy */
	}

	std::cout << "surface sample: sample " << pCloud->size() << " points." << std::endl;

	return pCloud;
}

bool CSurfaceUtil::calcProjPoints(const std::shared_ptr<pcl::on_nurbs::FittingSurface> vFit, const PC_t::Ptr vCloud, std::vector<SProj>& voProjs) {
	_EARLY_RETURN(!vFit, "surface sample : surface is null", false);
	
	if (voProjs.size()) {
		voProjs.clear();
	}

	auto pSampleCloud = sampleSurface(vFit, 10);

	pcl::search::KdTree<Point_t>::Ptr pTree(new pcl::search::KdTree<Point_t>);
	pTree->setInputCloud(pSampleCloud);

	for (const auto& p : *vCloud) {
		Point_t nn;
		pcl::Indices Indices;
		std::vector<float> Dists;
		pTree->nearestKSearch(p, 2, Indices, Dists);

		if (isPointEqual(p, vCloud->at(Indices[0]))) {
			nn = pSampleCloud->at(Indices[1]);
		}
		else {
			nn = pSampleCloud->at(Indices[0]);
		}

		voProjs.emplace_back(__calcProj(vFit, p, vec2f {nn.normal_x, nn.normal_y}));
	}
	
	std::cout << "calc Proj Point size: " << voProjs.size() << std::endl;
	
	return true;
}

void CSurfaceUtil::__geneSampleSequence(int vSampleRate, std::vector<vec2f>& voSeq) {
	for (int i = 0; i <= vSampleRate; i++) {
		for (int k = 0; k <= vSampleRate; k++) {
			voSeq.emplace_back(vec2f { (float)i / (float)vSampleRate, (float)k / (float)vSampleRate });
		}
	}
}

SProj CSurfaceUtil::__calcProj(const std::shared_ptr<pcl::on_nurbs::FittingSurface> vFit, const Point_t& vPt, const vec2f& vHint) {
	Eigen::Vector3d Pt { vPt.x, vPt.y, vPt.z };
	Eigen::Vector3d p;
	Eigen::Vector2d Hint { vHint.x, vHint.y };
	Eigen::Vector2d uv = vFit->inverseMapping(vFit->m_nurbs, Pt, Hint, p, 100, 1e-6, true);
	const auto Sample = vFit->m_nurbs.PointAt(uv[0], uv[1]);
	p = Eigen::Vector3d(Sample.x, Sample.y, Sample.z);
	float Dist = (Pt - p).norm();
	Eigen::Vector3d Direc = Pt - p;

	return SProj { vec3f{(float)p.x(), (float)p.y(), (float)p.z()}, vec3f{(float)Direc.x(), (float)Direc.y(), (float)Direc.z()}, vec2f{(float)uv.x(), (float)uv.y()}, Dist };
}
