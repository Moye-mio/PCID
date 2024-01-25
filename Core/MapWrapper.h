#pragma once

namespace core {

class CMapWrapper {
public:

	template <typename T>
	static cv::Mat castMap2CVMat(const std::shared_ptr<CMap<T>> vMap);

	template <>
	static cv::Mat castMap2CVMat<float>(const std::shared_ptr<CMap<float>> vMap);

	template <>
	static cv::Mat castMap2CVMat<vec2f>(const std::shared_ptr<CMap<vec2f>> vMap);

	template <>
	static cv::Mat castMap2CVMat<std::uint8_t>(const std::shared_ptr<CMap<std::uint8_t>> vMap);

	template<typename T>
	static bool saveMapToLocal(const ptr<CMap<T>> m);

	static std::variant<std::shared_ptr<CHeightMap>, std::shared_ptr<CGradientMap>, std::shared_ptr<CMaskMap>> castCVMat2Map(const cv::Mat& vImage) {
		switch (vImage.type()) {
		case CV_8UC1:
		{
			std::shared_ptr<CMaskMap> pMask(new CMaskMap(vImage.rows, vImage.cols));
			for (int i = 0; i < pMask->getWidth(); i++) {
				for (int k = 0; k < pMask->getHeight(); k++) {
					pMask->setValue(i, k, vImage.at<uchar>(i, k));
				}
			}

			return pMask;
		}
		case CV_32FC1:
		{
			std::shared_ptr<CHeightMap> pHeight(new CHeightMap(vImage.rows, vImage.cols));
			for (int i = 0; i < pHeight->getWidth(); i++) {
				for (int k = 0; k < pHeight->getHeight(); k++) {
					pHeight->setValue(i, k, vImage.at<float>(i, k));
				}
			}

			return pHeight;
		}
		case CV_32FC2:
		{
			std::shared_ptr<CGradientMap> pGradient(new CGradientMap(vImage.rows, vImage.cols));
			for (int i = 0; i < pGradient->getWidth(); i++) {
				for (int k = 0; k < pGradient->getHeight(); k++) {
					cv::Vec2f Value = vImage.at<cv::Vec2f>(i, k);
					pGradient->setValue(i, k, vec2f { Value[0], Value[1] });
				}
			}

			return pGradient;
		}
		default:
			break;
		}

		return std::shared_ptr<CHeightMap>(new CHeightMap);
	}

private:

};

template <typename T>
inline cv::Mat CMapWrapper::castMap2CVMat(const std::shared_ptr<CMap<T>> vMap) {
	return cv::Mat();
}

template <>
inline cv::Mat CMapWrapper::castMap2CVMat<float>(const std::shared_ptr<CMap<float>> vMap) {
	_EARLY_RETURN(vMap->isValid() == false, "map wrapper error: map is invalid.", cv::Mat());

	cv::Mat Image(vMap->getWidth(), vMap->getHeight(), CV_32FC1);
	for (int i = 0; i < vMap->getWidth(); i++) {
		for (int k = 0; k < vMap->getHeight(); k++) {
			Image.at<float>(i, k) = vMap->getValue(i, k);
		}
	}

	return Image;
}

template <>
inline cv::Mat CMapWrapper::castMap2CVMat<vec2f>(const std::shared_ptr<CMap<vec2f>> vMap) {
	_EARLY_RETURN(vMap->isValid() == false, "map wrapper error: map is invalid.", cv::Mat());

	cv::Mat Image(vMap->getWidth(), vMap->getHeight(), CV_32FC2);
	for (int i = 0; i < vMap->getWidth(); i++) {
		for (int k = 0; k < vMap->getHeight(); k++) {
			vec2f Value = vMap->getValue(i, k);
			Image.at<cv::Vec2f>(i, k) = cv::Vec2f { Value.x, Value.y };
		}
	}

	return Image;
}

template <>
inline cv::Mat CMapWrapper::castMap2CVMat<std::uint8_t>(const std::shared_ptr<CMap<std::uint8_t>> vMap) {
	_EARLY_RETURN(vMap->isValid() == false, "map wrapper error: map is invalid.", cv::Mat());

	cv::Mat Image(vMap->getWidth(), vMap->getHeight(), CV_8UC1);
	for (int i = 0; i < vMap->getWidth(); i++) {
		for (int k = 0; k < vMap->getHeight(); k++) {
			Image.at<std::uint8_t>(i, k) = vMap->getValue(i, k);
		}
	}

	return Image;
}

template<typename T>
inline bool CMapWrapper::saveMapToLocal(const ptr<CMap<T>> m) {
	_EARLY_RETURN(m->isValid() == false, "save map error: map is invalid.", nullptr);

	cv::Mat Image(m->getWidth(), m->getHeight(), CV_8UC1);
	for (int i = 0; i < Image.rows; i++) {

	}

}

}


