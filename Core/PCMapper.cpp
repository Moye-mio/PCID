#include "pch.h"
#include "PCMapper.h"

using namespace core;

PC_t::Ptr CPCMapper::map(const SAABB& vBox, const std::vector<vec3f>& vSamples) {
	_EARLY_RETURN(!vBox.isValid(), "pc mapper error: aabb is invalid.", nullptr);
	
	float SpanX = vBox.maxx - vBox.minx;
	float SpanY = vBox.maxy - vBox.miny;

	PC_t::Ptr pCloud(new PC_t);
	for (const auto e : vSamples) {
		float x = e.x * SpanX + vBox.minx;
		float y = e.y * SpanY + vBox.miny;
		pCloud->emplace_back(Point_t(x, y, e.z));
	}

	log("generate new cloud size: " + std::to_string(pCloud->size()));
	
	return pCloud;
}
