#include "pch.h"
#include "NurbsFitting.h"

using namespace core;

bool CNurbsFitting::run(const PC_t::Ptr& vCloud, int vDegree, int vRefinement, int vIters) {
	_EARLY_RETURN(!isPointCloudValid(vCloud), "nurbs fitting: input cloud is inValid", false);
	_EARLY_RETURN(vDegree < 2, "degree < 2", false);
	_EARLY_RETURN(vRefinement < 0, "Refinement < 0", false);
	_EARLY_RETURN(vIters < 1, "Iterations < 1", false);

	std::vector<ON_3dPoint> Corners;
	__calcInitCorner(vCloud, Corners);

	pcl::on_nurbs::NurbsDataSurface Data;
	__PCLPoint2Vector3d(vCloud, Data.interior);

	int Order = vDegree + 1;
	pcl::on_nurbs::FittingSurface::Parameter Params;
	Params.interior_smoothness = 1.0;
	Params.interior_weight = 1.0;
	Params.boundary_smoothness = 1.0;
	Params.boundary_weight = 0.0;

	std::cout << "Start Fitting Surface..." << std::endl;
	ON_NurbsSurface Nurbs = pcl::on_nurbs::FittingSurface::initNurbs4Corners(Order, Corners[0], Corners[1], Corners[2], Corners[3]);
	pcl::on_nurbs::FittingSurface Fit(&Data, Nurbs);

	for (int i = 0; i < vRefinement; i++) {
		Fit.refine(0);
		Fit.refine(1);
		Fit.assemble(Params);
		Fit.solve();

		std::cout << "refine " << i << ": control point numbers: [" << Fit.m_nurbs.m_cv_count[0] << ", " << Fit.m_nurbs.m_cv_count[1] << "]" << std::endl;
	}

	for (int i = 0; i < vIters; i++) {
		Fit.assemble(Params);
		Fit.solve();
		
		std::cout << "Iter " << i << ": control point numbers: [" << Fit.m_nurbs.m_cv_count[0] << ", " << Fit.m_nurbs.m_cv_count[1] << "]" << std::endl;
	}

	m_Fit = std::make_shared<pcl::on_nurbs::FittingSurface>(Fit);
	
	if (m_CVPath.size()) {
		__saveControlPointsToDisk();
	}

	if (m_KnotPath.size()) {
		__saveKnotToDisk();
	}

	std::cout << "Nurbs fitting finished: Success!" << std::endl;

	return true;
}

void CNurbsFitting::__calcInitCorner(const PC_t::Ptr& vCloud, std::vector<ON_3dPoint>& voCorner) {
	if (voCorner.size()) voCorner.clear();

	voCorner.emplace_back(ON_3dPoint(FLT_MAX, FLT_MAX, 0));
	voCorner.emplace_back(ON_3dPoint(-FLT_MAX, FLT_MAX, 0));
	voCorner.emplace_back(ON_3dPoint(-FLT_MAX, -FLT_MAX, 0));
	voCorner.emplace_back(ON_3dPoint(FLT_MAX, -FLT_MAX, 0));

	for (const auto& p : vCloud->points) {
		voCorner[0].x = std::fmin(p.x, voCorner[0].x);
		voCorner[0].y = std::fmin(p.y, voCorner[0].y);
		voCorner[1].x = std::fmax(p.x, voCorner[1].x);
		voCorner[1].y = std::fmin(p.y, voCorner[1].y);
		voCorner[2].x = std::fmax(p.x, voCorner[2].x);
		voCorner[2].y = std::fmax(p.y, voCorner[2].y);
		voCorner[3].x = std::fmin(p.x, voCorner[3].x);
		voCorner[3].y = std::fmax(p.y, voCorner[3].y);
	}

	std::vector<float> ds;
	ds.emplace_back(FLT_MAX);
	ds.emplace_back(FLT_MAX);
	ds.emplace_back(FLT_MAX);
	ds.emplace_back(FLT_MAX);

	for (const auto& p : vCloud->points) {
		for (int i = 0; i < ds.size(); i++) {
			float d = (p.x - voCorner[i].x) * (p.x - voCorner[i].x) + (p.y - voCorner[i].y) * (p.y - voCorner[i].y) + (p.z - voCorner[i].z) * (p.z - voCorner[i].z);
			if (ds[i] > d) {
				ds[i] = d;
				voCorner[i].z = p.z;
			}
		}
	}

	{
		std::cout << "Nurbs init corner: " << std::endl;
		for (auto e : voCorner) {
			std::cout << "[" << e.x << ", " << e.y << ", " << e.z << "]" << std::endl;
		}
	}
}

void CNurbsFitting::__PCLPoint2Vector3d(const PC_t::Ptr& vCloud, pcl::on_nurbs::vector_vec3d& voData) {
	if (voData.size() > 0) voData.clear();

	for (const auto& p : vCloud->points) {
		voData.emplace_back(Eigen::Vector3d(p.x, p.y, p.z));
	}
}

void CNurbsFitting::__saveControlPointsToDisk() {
	std::ofstream File(m_CVPath);
	for (int i = 0; i < m_Fit->m_nurbs.m_cv_count[0]; i++) {
		std::string sx = "";
		std::string sy = "";
		std::string sz = "";
		for (int k = 0; k < m_Fit->m_nurbs.m_cv_count[1]; k++) {
			ON_3dPoint p;
			m_Fit->m_nurbs.GetCV(i, k, p);

			sx += std::to_string(p.x) + " ";
			sy += std::to_string(p.y) + " ";
			sz += std::to_string(p.z) + " ";
		}
		File << sx << std::endl << sy << std::endl << sz << std::endl << std::endl;
	}
	File.close();
}

void core::CNurbsFitting::__saveKnotToDisk() {
	std::ofstream File(m_KnotPath);

	auto pKnotsU = m_Fit->m_nurbs.Knot(0);
	auto pKnotsV = m_Fit->m_nurbs.Knot(1);
	std::string su = "";
	std::string sv = "";
	for (int i = 0; i < m_Fit->m_nurbs.KnotCount(0); i++) {
		su += std::to_string(pKnotsU[i]) + " ";
	}
	for (int i = 0; i < m_Fit->m_nurbs.KnotCount(1); i++) {
		sv += std::to_string(pKnotsV[i]) + " ";
	}
	File << su << std::endl << sv << std::endl;

	File.close();
}
