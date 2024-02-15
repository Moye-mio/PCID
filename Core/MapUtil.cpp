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

ptr<CGradientMap> MapUtil::geneGradient(const ptr<CHeightMap> vpH) {
	_EARLY_RETURN(vpH->isValid() == false, "generate gradient error: height map is invalid.", nullptr);

	ptr<CGradientMap> pGradient(new CGradientMap(vpH->getWidth(), vpH->getHeight(), vec2f{ 0, 0 }));
	for (uint i = 0; i < vpH->getWidth(); i++) {
		for (uint k = 0; k < vpH->getHeight(); k++) {
			auto r1 = vpH->calcGradient(i, k, 0);
			auto r2 = vpH->calcGradient(i, k, 1);
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

ptr<CGradientMap> MapUtil::geneGradient(const ptr<CGradientMap> vpG) {
	_EARLY_RETURN(vpG->isValid() == false, "generate gradient error: gradient map is invalid.", nullptr);

	ptr<CGradientMap> pGradient(new CGradientMap(vpG->getWidth(), vpG->getHeight(), vec2f{ 0, 0 }));
	ptr<CHeightMap> pHeightX = getHeightMapFromGradientMap(vpG, 0);
	ptr<CHeightMap> pHeightY = getHeightMapFromGradientMap(vpG, 1);

	for (uint i = 0; i < vpG->getWidth(); i++) {
		for (uint k = 0; k < vpG->getHeight(); k++) {
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

ptr<CHeightMap> MapUtil::getHeightMapFromGradientMap(const ptr<CGradientMap> vpG, uint vAxis) {
	_EARLY_RETURN(vpG->isValid() == false, "get height map from gradient map error: gradient map is invalid.", nullptr);

	ptr<CHeightMap> pHeight(new CHeightMap(vpG->getWidth(), vpG->getHeight(), 0.0f));
	for (uint i = 0; i < vpG->getWidth(); i++) {
		for (uint k = 0; k < vpG->getHeight(); k++) {
			if (vAxis) {
				pHeight->setValue(i, k, vpG->getValue(i, k).x);
			}
			else {
				pHeight->setValue(i, k, vpG->getValue(i, k).y);
			}
		}
	}

	return pHeight;
}

ptr<CHeightMap> MapUtil::resize(const ptr<CHeightMap> h, uint rx, uint ry) {
	_EARLY_RETURN(h->isValid() == false, "map resize error: height map is invalid.", nullptr);
	_EARLY_RETURN(rx * ry == 0, "map resize error: rx * ry == 0.", nullptr);

	std::cout << "map resize: from [" << h->getWidth() << ", " << h->getHeight() << "] to [" << rx << ", " << ry << "]" << std::endl;

	ptr<CHeightMap> pHeight(new CHeightMap(rx, ry, 0.0f));
	for (int i = 0; i < rx; i++) {
		for (int k = 0; k < ry; k++) {
			float sx = (float)(i + 0.5f) / (float)rx * (float)h->getWidth();
			float sy = (float)(k + 0.5f) / (float)ry * (float)h->getHeight();
			float r = h->bisample(sx, sy);

			_EARLY_RETURN(MathUtil::isFloatNan(r), "bisample nan: [" << i << ", " << k << "]", nullptr);

			pHeight->setValue(i, k, r);
		}
	}

	return pHeight;
}


