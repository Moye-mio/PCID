#include "pch.h"
#include "OutlierDetector.h"
#include <pcl/filters/statistical_outlier_removal.h>

using namespace core;

bool COutlierDetecter::run(const PC_t::Ptr& vCloud, int k, float vThres) {
    _EARLY_RETURN(vCloud->empty(), "Cloud is empty.", false);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto extractPositions = ([=](const PC_t::Ptr& vCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& voCloud) {
        for (const auto& p : *vCloud) {
            voCloud->emplace_back(pcl::PointXYZ(p.x, p.y, p.z));
        }
        }
    );

    extractPositions(vCloud, pCloud);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> Detecter;
    Detecter.setInputCloud(pCloud);
    Detecter.setMeanK(k);
    Detecter.setStddevMulThresh(vThres);
    Detecter.filter(m_Inliers);
    Detecter.setNegative(true);
    Detecter.filter(m_Outliers);

   std::cout << "Inliers size: " << m_Inliers.size() << ", Outliers size: " << m_Outliers.size() << std::endl;

    return true;
}
