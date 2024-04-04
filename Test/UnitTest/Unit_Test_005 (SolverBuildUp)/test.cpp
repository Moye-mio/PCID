#include "pch.h"

template <typename T>
using ptr = std::shared_ptr<T>;
using hmptr = ptr<core::CHeightMap>;
using mmptr = ptr<core::CMaskMap>;
using gmptr = ptr<core::CGradientMap>;

class TestSetUpSolver: public testing::Test
{
protected:
	void SetUp() override {

	}

	void TearDown() override {
	}

	bool SolveEquations(const Eigen::MatrixXf& vMat, const Eigen::MatrixXf& vGT, Eigen::MatrixXf& voCoeff, Eigen::MatrixXf& voConstNumbers, std::vector<Eigen::Vector2f>& voUnknowns) {
		hmptr pHeight(new core::CHeightMap), pGT(new core::CHeightMap);
		setHeightMap(vMat, pHeight);
		setHeightMap(vGT, pGT);

		gmptr pGradient = core::MapUtil::geneGradient(pGT);
		gmptr pGoG = core::MapUtil::geneGradient(pGradient);
		
		core::CSolverBuilder Builder;
		bool r = Builder.run(pHeight, pGoG, pGradient);
		if (r == false) {
			return r;
		}

		Builder.dumpMatrix(voCoeff, voConstNumbers);
		Builder.dumpUnknowns(voUnknowns);

		return r;
	}

	bool setHeightMap(const Eigen::MatrixXf& vData, const hmptr h) {
		if (vData.size() == 0) {
			return false;
		}

		h->setSize(vData.rows(), vData.cols());
		for (int i = 0; i < vData.rows(); i++) {
			for (int k = 0; k < vData.cols(); k++) {
				h->setValue(i, k, vData(i, k));
			}
		}
		return true;
	}
};

TEST_F(TestSetUpSolver, DT_MismatchingMaps) {
	Eigen::Matrix<float, 3, 4> Mat;
	Mat << 0, 0, 0, 0,
		1, -FLT_MAX, -FLT_MAX, 1,
		2, 2, 2, 2;

	Eigen::MatrixXf Coeff, ConstNumbers, GTCoeff(2, 2), GTConst(2, 1);
	std::vector<Eigen::Vector2f> Unknowns, GTUnknowns;
	EXPECT_FALSE(SolveEquations(Mat, Mat, Coeff, ConstNumbers, Unknowns));
}

TEST_F(TestSetUpSolver, NT_InnerUnknowns) {
	Eigen::Matrix<float, 3, 4> Mat, GT;
	Mat << 0, 0, 0, 0,
		1, -FLT_MAX, -FLT_MAX, 1,
		2, 2, 2, 2;

	GT << 0, 0, 0, 0,
		1, 1, 1, 1,
		2, 2, 2, 2;
	Eigen::MatrixXf Coeff, ConstNumbers, GTCoeff(2, 2), GTConst(2, 1);
	std::vector<Eigen::Vector2f> Unknowns, GTUnknowns;
	EXPECT_TRUE(SolveEquations(Mat, GT, Coeff, ConstNumbers, Unknowns));

	GTCoeff << -4, 1, 1, -4;
	GTConst << -3, -3;
	GTUnknowns.push_back({ 1, 1 });
	GTUnknowns.push_back({ 1, 2 });
	ASSERT_EQ(Coeff, GTCoeff);
	ASSERT_EQ(ConstNumbers, GTConst);

	for (int i = 0; i < Unknowns.size(); i++)
		ASSERT_EQ(Unknowns[i], GTUnknowns[i]);
}

TEST_F(TestSetUpSolver, NT_AllUnknowns) {
	Eigen::Matrix<float, 3, 4> Mat, GT;
	Mat << 0, 0, -FLT_MAX, -FLT_MAX,
		1, 1, -FLT_MAX, -FLT_MAX,
		2, 2, 2, 2;

	GT << 0, 0, 0, 0,
		1, 1, 1, 1,
		2, 2, 2, 2;
	Eigen::MatrixXf Coeff, ConstNumbers, GTCoeff(4, 4), GTConst(4, 1);
	std::vector<Eigen::Vector2f> Unknowns, GTUnknowns;
	EXPECT_TRUE(SolveEquations(Mat, GT, Coeff, ConstNumbers, Unknowns));

	GTCoeff << -2, 1, 0, 0,
		1, -2, 0, 1,
		1, 0, -4, 1,
		0, 1, 0, -2;
	GTConst << 0, 1, -3, -2;
	GTUnknowns.push_back({ 0, 2 });
	GTUnknowns.push_back({ 0, 3 });
	GTUnknowns.push_back({ 1, 2 });
	GTUnknowns.push_back({ 1, 3 });
	ASSERT_EQ(Coeff, GTCoeff);
	ASSERT_EQ(ConstNumbers, GTConst);

	for (int i = 0; i < Unknowns.size(); i++)
		ASSERT_EQ(Unknowns[i], GTUnknowns[i]);
}
