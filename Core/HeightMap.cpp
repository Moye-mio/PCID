#include "pch.h"
#include "HeightMap.h"

using namespace core;

core::CHeightMap::CHeightMap()
	: CMap<float>(0, 0, -FLT_MAX)
{
	m_Empty = -FLT_MAX;
}

core::CHeightMap::CHeightMap(uint w, uint h)
	: CMap<float>(w, h, -FLT_MAX)
{
	m_Empty = -FLT_MAX;
}

core::CHeightMap::CHeightMap(uint w, uint h, float v)
	: CMap<float>(w, h, v) 
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

float CHeightMap::bisample(float i, float k) {		/* 0 < i, k < width, height */
	if (i > m_Width|| k > m_Height|| i < 0 || k < 0) {
		return m_Empty;
	}

	int x2 = (int)(std::roundf(i));
	int y2 = (int)(std::roundf(k));
	int x1 = (x2 == 0) ? 0 : x2 - 1;
	int y1 = (y2 == 0) ? 0 : y2 - 1;
	x2 = (x2 == m_Width) ? x2 - 1 : x2;
	y2 = (y2 == m_Height) ? y2 - 1 : y2;

	float v1 = m_Data[x1][y1];
	float v2 = m_Data[x2][y1];
	float v3 = m_Data[x1][y2];
	float v4 = m_Data[x2][y2];

	float r = MathUtil::bilinearInterpolate(v1, v2, v3, v4, -x2 + 0.5f + i, -y2 + 0.5f + k);

	return r;
}

std::pair<uint, uint> CHeightMap::getMaxId() const {
	float Tmax = -FLT_MAX;
	int m = 0, n = 0;
	for (int i = 0; i < m_Width; i++) {
		for (int k = 0; k < m_Height; k++) {
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
	int m = 0, n = 0;
	for (int i = 0; i < m_Width; i++) {
		for (int k = 0; k < m_Height; k++) {
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

std::optional<float> CHeightMap::calcGradient(uint i, uint k, uint axis) {
	_EARLY_RETURN(i >= m_Width || k >= m_Height || isValid() == false, "calc gradient error: i or k out of scale or map is invalid.", std::nullopt);

	if (isEmpty(i, k)) {
		return std::nullopt;
	}

	std::vector<float> Heights;
	if (axis == 0) {
		if (i == 0 && !isEmpty(i, k) && !isEmpty(i + 1, k))								return m_Data[i + 1][k] - m_Data[i][k];
		else if (i == m_Width - 1 && !isEmpty(i, k) && !isEmpty(i - 1, k))				return m_Data[i][k] - m_Data[i - 1][k];
		else if (i > 0 && i < m_Width - 1 ) {
			if (!isEmpty(i - 1, k) && !isEmpty(i + 1, k))								return (m_Data[i + 1][k] - m_Data[i - 1][k]) / 2;
			else if (!isEmpty(i, k) && !isEmpty(i + 1, k))								return m_Data[i + 1][k] - m_Data[i][k];
			else if (!isEmpty(i, k) && !isEmpty(i - 1, k))								return m_Data[i][k] - m_Data[i - 1][k];
		}
	}
	else {
		if (k == 0 && !isEmpty(i, k) && !isEmpty(i, k + 1))								return m_Data[i][k + 1] - m_Data[i][k];
		else if (k == m_Height - 1 && !isEmpty(i, k) && !isEmpty(i, k - 1))				return m_Data[i][k] - m_Data[i][k - 1];
		else if (k > 0 && k < m_Height - 1) {
			if (!isEmpty(i, k - 1) && !isEmpty(i, k + 1))								return (m_Data[i][k + 1] - m_Data[i][k - 1]) / 2;
			else if (!isEmpty(i, k) && !isEmpty(i, k + 1))								return m_Data[i][k + 1] - m_Data[i][k];
			else if (!isEmpty(i, k) && !isEmpty(i, k - 1))								return m_Data[i][k] - m_Data[i][k - 1];
		}
	}

	return std::nullopt;
}

