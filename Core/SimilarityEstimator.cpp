#include "pch.h"
#include "SimilarityEstimator.h"
//#include "Geometric_distortion.h"
#include "HausdorffDistance.h"
#include "PointCloudUtil.h"

using namespace core;

std::optional<float> CSimilarityEstimator::compute(const PC_t::Ptr& vLhs, const PC_t::Ptr& vRhs, ESimilarityMode vMode) {
	_EARLY_RETURN(vLhs == nullptr || vRhs == nullptr, "Cloud is nullptr", std::nullopt);
	_EARLY_RETURN(vLhs->empty() || vRhs->empty(), "Cloud is empty", std::nullopt);

	float Dist = 0.0f;
	switch (vMode) {
	case core::ESimilarityMode::GPSNR:
		Dist = __calcGPSNR(vLhs, vRhs);
		break;
	case core::ESimilarityMode::NSHD:
		Dist = __calcNSHD(vLhs, vRhs);
		break;
	default:
		break;
	}

	return Dist;
}

float CSimilarityEstimator::__calcGPSNR(const PC_t::Ptr& vLhs, const PC_t::Ptr& vRhs) {
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr p1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr p2(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudUtil::extractXYZpt(vLhs, p1);
	PointCloudUtil::extractXYZpt(vLhs, p2);
	pcl::geometric_quality::commandPar Command;
	pcl::geometric_quality::qMetric Metric;
	pcl::geometric_quality::computeGeometricQualityMetric(*vLhs, *vRhs, Command, Metric);
	return Metric.c2c_psnr;*/
	return 0;
}

float CSimilarityEstimator::__calcNSHD(const PC_t::Ptr& vLhs, const PC_t::Ptr& vRhs) {
	CHausDorffDistance Haus;
	auto r = Haus.compute(vLhs, vRhs);
	_EARLY_RETURN(r.has_value() == false, "Hausdorff Distance calc failed", FLT_MAX);

	return r.value();
}