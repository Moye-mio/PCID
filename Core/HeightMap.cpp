#include "pch.h"
#include "HeightMap.h"
#include<optional>
#include<iostream>
using namespace std;

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

float CHeightMap::bisample(float vXPosition, float vYPosition) {		
	if (vXPosition > m_Width || vYPosition > m_Height) {
		return m_Empty;
	}

	int X1 = vXPosition > 0 ? (int)(vXPosition) : 0;
	int Y1 = vYPosition > 0 ? (int)(vYPosition) : 0;
	int X2;
	if (X1 == m_Width - 1)
		X2 = X1;
	else if (vXPosition<0)
		X2 = 0;
	else
		X2 = X1 + 1;
	int Y2 ;
	if (Y1 == m_Height - 1)
		Y2 = Y1;
	else if (vYPosition<0)
		Y2 = 0;
	else
		Y2 = Y1 + 1;

	float V1 = m_Data[X1][Y1];
	float V2 = m_Data[X2][Y1];
	float V3 = m_Data[X1][Y2];
	float V4 = m_Data[X2][Y2];

	float U = vXPosition?vXPosition - X1:0;
	float V = vYPosition?vYPosition - Y1:0;
	float Result = MathUtil::bilinearInterpolate(V1, V2, V3, V4, U, V);

	return Result;
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

