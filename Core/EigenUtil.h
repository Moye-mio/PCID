#pragma once

#include <Eigen/Eigen>
#include "MathUtil.h"

namespace core {

class EigenUtil {
public:

	static bool isEigenValid(const Eigen::MatrixXf& vData);
	static bool isEigenHasEmptyValue(const Eigen::MatrixXf& vData);


private:


};

inline bool EigenUtil::isEigenValid(const Eigen::MatrixXf& vData) {
	for (int i = 0; i < vData.rows(); i++) {
		for (int k = 0; k < vData.cols(); k++) {
			if (MathUtil::isFloatNan(vData(i, k))) {
				return false;
			}
		}
	}

	return true;
}

inline bool EigenUtil::isEigenHasEmptyValue(const Eigen::MatrixXf& vData) {
	for (int i = 0; i < vData.rows(); i++) {
		for (int k = 0; k < vData.cols(); k++) {
			if (MathUtil::isEqual(vData(i, k), -FLT_MAX)) {
				return true;
			}
		}
	}

	return false;
}

}
