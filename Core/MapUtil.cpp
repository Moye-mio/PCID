#include "pch.h"
#include "MapUtil.h"
#include "Map.hpp"
#include "Heightmap.h"
#include "GradientMap.h"
#include "IOUtil.h"

using namespace core;

ptr<CGradientMap> MapUtil::geneGradient(const ptr<CHeightMap> h) {
	_EARLY_RETURN(h->isValid() == false, "generate gradient error: height map is invalid.", nullptr);

	ptr<CGradientMap> pGradient(new CGradientMap(h->getWidth(), h->getHeight(), vec2f { 0, 0 }));
	for (uint i = 0; i < h->getWidth(); i++) {
		for (uint k = 0; k < h->getHeight(); k++) {
			auto r1 = h->calcGradient(i, k, 0);
			auto r2 = h->calcGradient(i, k, 1);
			if (r1.has_value() && r2.has_value()) {
				pGradient->setValue(i, k, vec2f { r1.value(), r2.value() });
			}
			else {
				pGradient->setEmpty(i, k);
			}
		}
	}

	return pGradient;
}

ptr<CGradientMap> MapUtil::geneGradient(const ptr<CGradientMap> g) {
	_EARLY_RETURN(g->isValid() == false, "generate gradient error: gradient map is invalid.", nullptr);

	ptr<CGradientMap> pGradient(new CGradientMap(g->getWidth(), g->getHeight(), vec2f { 0, 0 }));
	ptr<CHeightMap> pHeightX = getHeightMapFromGradientMap(g, 0);
	ptr<CHeightMap> pHeightY = getHeightMapFromGradientMap(g, 1);

	for (uint i = 0; i < g->getWidth(); i++) {
		for (uint k = 0; k < g->getHeight(); k++) {
			auto r1 = pHeightX->calcGradient(i, k, 0);
			auto r2 = pHeightY->calcGradient(i, k, 1);
			if (r1.has_value() && r2.has_value()) {
				pGradient->setValue(i, k, vec2f { r1.value(), r2.value() });
			}
			else {
				pGradient->setEmpty(i, k);
			}
		}
	}

	return pGradient;
}

ptr<CHeightMap> MapUtil::getHeightMapFromGradientMap(const ptr<CGradientMap> g, uint axis) {
	_EARLY_RETURN(g->isValid() == false, "get height map from gradient map error: gradient map is invalid.", nullptr);
	
	ptr<CHeightMap> pHeight(new CHeightMap(g->getWidth(), g->getHeight(), 0.0f));
	for (uint i = 0; i < g->getWidth(); i++) {
		for (uint k = 0; k < g->getHeight(); k++) {
			if (axis) {
				pHeight->setValue(i, k, g->getValue(i, k).x);
			}
			else {
				pHeight->setValue(i, k, g->getValue(i, k).y);
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

float MapUtil::calcRMSE(const cv::Mat& a, cv::Mat& b) {
	int Channels = a.channels();
	int Rows = a.rows;
	int Cols = a.cols * Channels;
	float Sigma = 0.0;
	float MSE = 0.0;
	for (int i = 0; i < Rows; i++) {
		for (int j = 0; j < Cols; j++) {
			MSE += (a.ptr<uchar>(i)[j] - b.ptr<uchar>(i)[j]) * (a.ptr<uchar>(i)[j] - b.ptr<uchar>(i)[j]);
		}
	}
	MSE = MSE / (Rows * Cols);
	return MSE;
}

float core::MapUtil::calcPSNR(const cv::Mat& a, cv::Mat& b) {
	int Channels = a.channels();
	int Rows = a.rows;
	int Cols = a.cols * Channels;

	float Sigma = 0.0;
	float MSE = 0.0;
	float PSNR = 0.0;
	for (int i = 0; i < Rows; i++) {
		for (int j = 0; j < Cols; j++) {
			Sigma += (a.ptr<uchar>(i)[j]) * (a.ptr<uchar>(i)[j]);
			MSE += (a.ptr<uchar>(i)[j] - b.ptr<uchar>(i)[j]) * (a.ptr<uchar>(i)[j] - b.ptr<uchar>(i)[j]);
		}
	}
	PSNR = 10 * log10(Sigma / MSE);
	return PSNR;
}
