#include "pch.h"
#include "MapUtil.h"
#include "Map.hpp"
#include "Heightmap.h"
#include "GradientMap.h"
#include "MaskMap.h"
#include "IOUtil.h"

using namespace core;
MapUtil& MapUtil::GetInstance()
{
	static MapUtil vMapUtil;
	return vMapUtil;
}

void MapUtil::Print()
{
	std::cout << "我的实例内存地址是：" << this << std::endl;
}

MapUtil::MapUtil()
{

}

MapUtil::~MapUtil()
{

}

ptr<CGradientMap> MapUtil::geneGradient(const ptr<CHeightMap> vHeightMap) {
	_EARLY_RETURN(vHeightMap->isValid() == false, "generate gradient error: height map is invalid.", nullptr);

	ptr<CGradientMap> pGradient(new CGradientMap(vHeightMap->getWidth(), vHeightMap->getHeight(), vec2f{ 0, 0 }));
	for (uint i = 0; i < vHeightMap->getWidth(); i++) {
		for (uint k = 0; k < vHeightMap->getHeight(); k++) {
			auto r1 = vHeightMap->calcGradient(i, k, 0);
			auto r2 = vHeightMap->calcGradient(i, k, 1);
			if (r1.has_value() && r2.has_value()) {
				pGradient->setValue(i, k, vec2f{ r1.value(), r2.value() });
			}
			else {
				pGradient->setEmpty(i, k);
			}
		}
	}

	return pGradient;
}

ptr<CGradientMap> MapUtil::geneGradient(const ptr<CGradientMap> vGradientMap) {
	_EARLY_RETURN(vGradientMap->isValid() == false, "generate gradient error: gradient map is invalid.", nullptr);

	ptr<CGradientMap> pGradient(new CGradientMap(vGradientMap->getWidth(), vGradientMap->getHeight(), vec2f{ 0, 0 }));
	ptr<CHeightMap> pHeightX = getHeightMapFromGradientMap(vGradientMap, 0);
	ptr<CHeightMap> pHeightY = getHeightMapFromGradientMap(vGradientMap, 1);

	for (uint i = 0; i < vGradientMap->getWidth(); i++) {
		for (uint k = 0; k < vGradientMap->getHeight(); k++) {
			auto r1 = pHeightX->calcGradient(i, k, 0);
			auto r2 = pHeightY->calcGradient(i, k, 1);
			if (r1.has_value() && r2.has_value()) {
				pGradient->setValue(i, k, vec2f{ r1.value(), r2.value() });
			}
			else {
				pGradient->setEmpty(i, k);
			}
		}
	}

	return pGradient;
}

ptr<CHeightMap> MapUtil::getHeightMapFromGradientMap(const ptr<CGradientMap> vGradientMap, uint vAxis) {
	_EARLY_RETURN(vGradientMap->isValid() == false, "get height map from gradient map error: gradient map is invalid.", nullptr);

	ptr<CHeightMap> pHeight(new CHeightMap(vGradientMap->getWidth(), vGradientMap->getHeight(), 0.0f));
	for (uint i = 0; i < vGradientMap->getWidth(); i++) {
		for (uint k = 0; k < vGradientMap->getHeight(); k++) {
			if (vAxis) {
				pHeight->setValue(i, k, vGradientMap->getValue(i, k).x);
			}
			else {
				pHeight->setValue(i, k, vGradientMap->getValue(i, k).y);
			}
		}
	}

	return pHeight;
}

ptr<CHeightMap> MapUtil::resize(const ptr<CHeightMap> vHeightMap, uint  RescaledWidth, uint  RescaledHeight) {
	_EARLY_RETURN(vHeightMap->isValid() == false, "map resize error: height map is invalid.", nullptr);
	_EARLY_RETURN( RescaledWidth *  RescaledHeight == 0, "map resize error:  RescaledWidth *  RescaledHeight == 0.", nullptr);

	std::cout << "map resize: from [" << vHeightMap->getWidth() << ", " << vHeightMap->getHeight() << "] to [" <<  RescaledWidth << ", " <<  RescaledHeight << "]" << std::endl;

	ptr<CHeightMap> pHeight(new CHeightMap( RescaledWidth,  RescaledHeight, 0.0f));
	for (int i = 0; i <  RescaledWidth; i++) {
		for (int k = 0; k <  RescaledHeight; k++) {
			float sx = (float)(i + 0.5f) / (float) RescaledWidth * (float)vHeightMap->getWidth();
			float sy = (float)(k + 0.5f) / (float) RescaledHeight * (float)vHeightMap->getHeight();
			float r = vHeightMap->bisample(sx, sy);

			_EARLY_RETURN(MathUtil::isFloatNan(r), "bisample nan: [" << i << ", " << k << "]", nullptr);

			pHeight->setValue(i, k, r);
		}
	}

	return pHeight;
}


