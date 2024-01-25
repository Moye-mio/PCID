#pragma once

#include "Map.hpp"

namespace core {

class CHeightMap : public CMap<float> {
public:
	CHeightMap();
	CHeightMap(uint w, uint h);
	CHeightMap(uint w, uint h, float v);

	float getMax() const;
	float getMin() const;
	float bisample(float i, float k);
	std::pair<uint, uint> getMaxId() const;
	std::pair<uint, uint> getMinId() const;

	std::optional<float> calcGradient(uint i, uint k, uint axis);

private:


};

}