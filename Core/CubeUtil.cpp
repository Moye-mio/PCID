#include "pch.h"
#include "CubeUtil.h"
#include "PointCloudUtil.h"

using namespace core;

vec3f CubeUtil::calcDC(const PC_t::Ptr vCloud, const SCube& c) {
    vec3f tn {0.0f, 0, 0};
    for (auto e : c.indices) {
        auto p = vCloud->at(e);
        tn.x += p.normal_x;
        tn.y += p.normal_y;
        tn.z += p.normal_z;
    }

    float nl = std::sqrtf(tn.x * tn.x + tn.y * tn.y + tn.z * tn.z);

    return vec3f { tn.x / nl, tn.y / nl, tn.z / nl };
}

float CubeUtil::calcAGTV(const PC_t::Ptr vCloud, const SCube& c, int k) {
    auto pCloud = extractPtsFromCube(vCloud, c);

    pcl::search::KdTree<Point_t>::Ptr pTree(new pcl::search::KdTree<Point_t>);
    pTree->setInputCloud(pCloud);

    float agtv = 0.0f;

    for (size_t i = 0; i < pCloud->size(); i++) {
        std::vector<int> Indices;
        std::vector<float> Dists;

        pTree->nearestKSearch(pCloud->at(i), k + 1, Indices, Dists);	/* Indices[0] is point itself. */

        vec3f nik { pCloud->at(i).normal_x, pCloud->at(i).normal_y, pCloud->at(i).normal_z };
        for (auto e : Indices) {
            if (e == i) {
                continue;
            }
            else {
                vec3f nil { pCloud->at(e).normal_x, pCloud->at(e).normal_y, pCloud->at(e).normal_z };
                agtv += std::fabsf(nik.x * nil.x + nik.y * nil.y + nik.z * nil.z);
            }
        }
    }
    
    return agtv / (k * (k - 1));
}

float CubeUtil::calcCubeSimilarity(const PC_t::Ptr vCloud, const SCube& a, const SCube& b) {
    vec3f DCa = calcDC(vCloud, a);
    vec3f DCb = calcDC(vCloud, b);
    float Dab = std::fabsf(DCa.x * DCb.x + DCa.y * DCb.y + DCa.z * DCb.z);

    float AGTVa = calcAGTV(vCloud, a, std::sqrtf(a.indices.size()));
    float AGTVb = calcAGTV(vCloud, b, std::sqrtf(b.indices.size()));
    float Vab = std::fabsf(AGTVa - AGTVb);

    return std::powf(std::numbers::e, -(Dab + Vab));
}

PC_t::Ptr CubeUtil::extractPtsFromCube(const PC_t::Ptr vCloud, const SCube& c) {
    PC_t::Ptr pCloud(new PC_t);
    
    for (auto e : c.indices) {
        auto p = vCloud->at(e);
        pCloud->emplace_back(p);
    }

    return pCloud;
}
