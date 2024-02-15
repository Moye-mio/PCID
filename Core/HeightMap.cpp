#include "pch.h"
#include "HeightMap.h"
#include<optional>

using namespace core;

core::CHeightMap::CHeightMap()
	: CMap<float>(0, 0, -FLT_MAX)
{
	m_Empty = -FLT_MAX;
}

core::CHeightMap::CHeightMap(uint vWidth, uint vHeight)
	: CMap<float>(vWidth, vHeight, -FLT_MAX)
{
	m_Empty = -FLT_MAX;
}

core::CHeightMap::CHeightMap(uint vWidth, uint vHeight, float vVlaue)
	: CMap<float>(vWidth, vHeight, vVlaue)
{
	m_Empty = -FLT_MAX;
}

float CHeightMap::getMax() const {
	const auto Id = getMaxId();
	return m_Data[Id.first][Id.second];
}

float CHeightMap::getMin() const {
	const auto Id = getMinId();
	return m_Data[Id.first][Id.second];
}

float CHeightMap::bisample(float vXPosition, float vYPosition) {		/* 0 < i, k < width, height */
	if (vXPosition > m_Width || vYPosition > m_Height || vXPosition < 0 || vYPosition < 0) {
		return m_Empty;
	}

	int x2 = (int)(std::roundf(vXPosition));
	int y2 = (int)(std::roundf(vYPosition));
	int x1 = (x2 == 0) ? 0 : x2 - 1;
	int y1 = (y2 == 0) ? 0 : y2 - 1;
	x2 = (x2 == m_Width) ? x2 - 1 : x2;
	y2 = (y2 == m_Height) ? y2 - 1 : y2;

	float v1 = m_Data[x1][y1];
	float v2 = m_Data[x2][y1];
	float v3 = m_Data[x1][y2];
	float v4 = m_Data[x2][y2];

	float r = MathUtil::bilinearInterpolate(v1, v2, v3, v4, -x2 + 0.5f + vXPosition, -y2 + 0.5f + vYPosition);

	return r;
}


std::pair<uint, uint> CHeightMap::getMaxId() const {
	float Tmax = -FLT_MAX;
	uint m = 0, n = 0;
	for (uint i = 0; i < m_Width; i++) {
		for (uint k = 0; k < m_Height; k++) {
			if (isEmpty(i, k)) continue;
			if (Tmax < m_Data[i][k]) {
				Tmax = m_Data[i][k];
				m = i;
				n = k;
			}
		}
	}
	return std::make_pair(m, n);
}

std::pair<uint, uint> CHeightMap::getMinId() const {
	float Tmin = FLT_MAX;
	uint m = 0, n = 0;
	for (uint i = 0; i < m_Width; i++) {
		for (uint k = 0; k < m_Height; k++) {
			if (isEmpty(i, k)) continue;

			if (Tmin > m_Data[i][k]) {
				Tmin = m_Data[i][k];
				m = i;
				n = k;
			}
		}
	}
	return std::make_pair(m, n);
}

std::optional<float> CHeightMap::calcGradient(uint vXPosition, uint vYPosition, uint vAxis) {
	_EARLY_RETURN(vXPosition >= m_Width || vYPosition >= m_Height || isValid() == false, "calc gradient error: vXPosition or vYPosition out of scale or map is invalid.", std::nullopt);

	if (isEmpty(vXPosition, vYPosition)) {
		return std::nullopt;
	}

	std::vector<float> Heights;
	if (vAxis == 0) {
		if (vXPosition == 0 && !isEmpty(vXPosition, vYPosition) && !isEmpty(vXPosition + 1, vYPosition))								return m_Data[vXPosition + 1][vYPosition] - m_Data[vXPosition][vYPosition];
		else if (vXPosition == m_Width - 1 && !isEmpty(vXPosition, vYPosition) && !isEmpty(vXPosition - 1, vYPosition))				    return m_Data[vXPosition][vYPosition] - m_Data[vXPosition - 1][vYPosition];
		else if (vXPosition > 0 && vXPosition < m_Width - 1) {
			if (!isEmpty(vXPosition - 1, vYPosition) && !isEmpty(vXPosition + 1, vYPosition))								            return (m_Data[vXPosition + 1][vYPosition] - m_Data[vXPosition - 1][vYPosition]) / 2;
			else if (!isEmpty(vXPosition, vYPosition) && !isEmpty(vXPosition + 1, vYPosition))								            return m_Data[vXPosition + 1][vYPosition] - m_Data[vXPosition][vYPosition];
			else if (!isEmpty(vXPosition, vYPosition) && !isEmpty(vXPosition - 1, vYPosition))								            return m_Data[vXPosition][vYPosition] - m_Data[vXPosition - 1][vYPosition];
		}
	}
	else {
		if (vYPosition == 0 && !isEmpty(vXPosition, vYPosition) && !isEmpty(vXPosition, vYPosition + 1))								return m_Data[vXPosition][vYPosition + 1] - m_Data[vXPosition][vYPosition];
		else if (vYPosition == m_Height - 1 && !isEmpty(vXPosition, vYPosition) && !isEmpty(vXPosition, vYPosition - 1))				return m_Data[vXPosition][vYPosition] - m_Data[vXPosition][vYPosition - 1];
		else if (vYPosition > 0 && vYPosition < m_Height - 1) {
			if (!isEmpty(vXPosition, vYPosition - 1) && !isEmpty(vXPosition, vYPosition + 1))								            return (m_Data[vXPosition][vYPosition + 1] - m_Data[vXPosition][vYPosition - 1]) / 2;
			else if (!isEmpty(vXPosition, vYPosition) && !isEmpty(vXPosition, vYPosition + 1))								            return m_Data[vXPosition][vYPosition + 1] - m_Data[vXPosition][vYPosition];
			else if (!isEmpty(vXPosition, vYPosition) && !isEmpty(vXPosition, vYPosition - 1))								            return m_Data[vXPosition][vYPosition] - m_Data[vXPosition][vYPosition - 1];
		}
	}

	return std::nullopt;
}

