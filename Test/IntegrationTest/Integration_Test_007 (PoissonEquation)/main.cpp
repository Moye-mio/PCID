#include "header.h"

template <typename T>
using ptr = std::shared_ptr<T>;
using hmptr = ptr<core::CHeightMap>;
using mmptr = ptr<core::CMaskMap>;
using gmptr = ptr<core::CGradientMap>;

void saveMatrix(const std::string& vPath, const Eigen::MatrixXf& m) {
	std::ofstream File(vPath);

	for (int i = 0; i < m.rows(); i++) {
		for (int k = 0; k < m.cols(); k++) {
			File << m(i, k) << " ";
		}
		File << std::endl;
	}

	File.close();
}

void saveVector2f(const std::string& vPath, const std::vector<Eigen::Vector2f>& m) {
	std::ofstream File(vPath);

	for (const auto& e : m) {
		File << e.x() << " " << e.y() << std::endl;
	}

	File.close();
}

int main() {
	cv::Mat Raw = cv::imread("Images/1.png", cv::IMREAD_GRAYSCALE);
	cv::Mat Mask = cv::imread("Images/1-mask.png", cv::IMREAD_GRAYSCALE);
	cv::Mat Gt(Raw.size(), CV_32FC1);
	Raw.convertTo(Gt, CV_32FC1);

	hmptr pHeight(new core::CHeightMap), pGT(new core::CHeightMap);
	pGT = std::get<0>(core::CMapWrapper::castCVMat2Map(Gt));
	pHeight = std::get<0>(core::CMapWrapper::castCVMat2Map(Gt));

	for (uint i = 0; i < pHeight->getWidth(); i++) {
		for (uint k = 0; k < pHeight->getWidth(); k++) {
			if (Mask.at<uchar>(i, k) == 0) {
				pHeight->setEmpty(i, k);
			}
		}
	}

	gmptr pGradient = core::MapUtil::geneGradient(pGT);
	gmptr pGoG = core::MapUtil::geneGradient(pGradient);

	core::CSolverBuilder Builder;
	bool r = Builder.run(pHeight, pGoG, pGradient);

	Eigen::MatrixXf Coeff, ConstNumbers, Solutions;
	std::vector<Eigen::Vector2f> Unknowns;
	Builder.dumpMatrix(Coeff, ConstNumbers);
	Builder.dumpUnknowns(Unknowns);

	saveMatrix("Matrix/coeff.txt", Coeff);
	saveMatrix("Matrix/constnumbers.txt", ConstNumbers);
	saveVector2f("Matrix/Unknowns.txt", Unknowns);

	core::CSparseLinearSolver Solver(Coeff, ConstNumbers);
	for (int i = 0; i < 7; i++) {
		Solutions = Solver.solve(static_cast<core::ESolverMode>(i));
		//Solutions = Solver.solve(core::ESolverMode::FullPivLU);

		for (int i = 0; i < Unknowns.size(); i++) {
			pHeight->setValue(Unknowns[i][0], Unknowns[i][1], Solutions(i, 0));
		}

		cv::Mat Height;
		core::CMapWrapper::castMap2CVMat<float>(pHeight).convertTo(Height, CV_8UC1);
		std::cout << i << ": " << core::MapUtil::calcRMSE(Height, Raw) << ", " << core::MapUtil::calcPSNR(Height, Raw) << std::endl;

		core::CMapWrapper::saveMapToLocal(pHeight, "Images/1-pmpe-" + std::to_string(i) + ".png");
	}
	core::CMapWrapper::saveGMapToLocal(pGradient, "Images/1-pmpe");

	return 0;
}
