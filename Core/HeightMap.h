#pragma once

#include "Map.hpp"
#include<optional>

namespace core {

	class CHeightMap : public CMap<float> {
	public:
		CHeightMap();
		CHeightMap(uint vWidth, uint vHeight);
		CHeightMap(uint vWidth, uint vHeight, float vValue);

		float getMax() const;
		float getMin() const;
		float bisample(float vXPosition, float vYPosition);
		std::pair<uint, uint> getMaxId() const;
		std::pair<uint, uint> getMinId() const;

		std::optional<float> calcGradient(uint vXPosition, uint vYPosition, uint vAxis);

	private:


	};

}