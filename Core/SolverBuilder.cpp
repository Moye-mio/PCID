#include "pch.h"
#include "SolverBuilder.h"
#include "HeightMap.h"
#include "GradientMap.h"
#include "MaskMap.h"
#include "MapUtil.h"

using namespace core;

bool CSolverBuilder::run(const ptr<CHeightMap> vRaw, const ptr<CGradientMap> vGoG, const ptr<CGradientMap> vGradient) {
	_EARLY_RETURN(!vRaw->isValid() || !vGoG->isValid() || !vGradient->isValid(), "solver builder error: input is not valid.", false);
	_EARLY_RETURN(vRaw->getWidth() != vGoG->getWidth() || vRaw->getWidth() != vGradient->getWidth() || vRaw->getHeight() != vGoG->getHeight() || vRaw->getHeight() != vGradient->getHeight(), "solver builder error: input size is not equal.", false);
	_EARLY_RETURN(!vGoG->isNoEmpty() || !vGradient->isNoEmpty(), "solver builder error: gradient has empty value.", false);

	m_Raw = vRaw;
	m_GoG = vGoG;
	m_Gradient = vGradient;
	m_Mask = MapUtil::GetInstance().geneMask<float>(m_Raw);
	_EARLY_RETURN(!m_Mask->isValid(), "solver builder error: mask is not valid.", false);

	__init();

	for (int i = 0; i < m_Unknowns.size(); i++)
		__setUpEachEquation(i);
	return true;

}

void CSolverBuilder::__init() {
	for (int i = 0; i < m_Mask->getWidth(); i++)
		for (int k = 0; k < m_Mask->getHeight(); k++)
			if (m_Mask->isEmpty(i, k))
				m_Unknowns.push_back({ i, k });

	m_Coeff = Eigen::MatrixXf(m_Unknowns.size(), m_Unknowns.size());
	m_Const = Eigen::MatrixXf(m_Unknowns.size(), 1);
	m_Coeff.setZero();
	m_Const.setZero();
}

void CSolverBuilder::__setUpEachEquation(int vIndex) {
	int X = m_Unknowns[vIndex][0];
	int Y = m_Unknowns[vIndex][1];

	if ((X == 0 || X == m_Mask->getWidth() - 1) && (Y == 0 || Y == m_Mask->getHeight() - 1))			/* Corner */
	{
		float ConstNumber = m_Gradient->getValue(X, Y).x + m_Gradient->getValue(X, Y).y;
		ConstNumber = __handleNeighbor({ X, Y - 1 }, ConstNumber, vIndex, 1);
		ConstNumber = __handleNeighbor({ X - 1, Y }, ConstNumber, vIndex, 1);
		ConstNumber = __handleNeighbor({ X + 1, Y }, ConstNumber, vIndex, 1);
		ConstNumber = __handleNeighbor({ X, Y + 1 }, ConstNumber, vIndex, 1);
		m_Coeff.coeffRef(vIndex, vIndex) = -2;
		m_Const.coeffRef(vIndex, 0) = ConstNumber;

	}
	else if (X == 0 || X == m_Mask->getWidth() - 1 || Y == 0 || Y == m_Mask->getHeight() - 1)			/* Border */
	{
		float ConstNumber = m_GoG->getValue(X, Y).x + m_GoG->getValue(X, Y).y;
		if (X == 0 || X == m_Mask->getWidth() - 1) {
			ConstNumber = __handleNeighbor({ X, Y - 1 }, ConstNumber, vIndex, 1);
			ConstNumber = __handleNeighbor({ X, Y + 1 }, ConstNumber, vIndex, 1);

		}
		else {
			ConstNumber = __handleNeighbor({ X - 1, Y }, ConstNumber, vIndex, 1);
			ConstNumber = __handleNeighbor({ X + 1, Y }, ConstNumber, vIndex, 1);
		}

		m_Coeff.coeffRef(vIndex, vIndex) = -2;
		m_Const.coeffRef(vIndex, 0) = ConstNumber;
	}
	else {
		// D(x,y-1)+D(x-1,y)+D(x+1,y)+D(x,y+1)-4D(x,y)=GoG(x,y)[0]+GoG(x,y)[1]
		float ConstNumber = m_GoG->getValue(X, Y).x + m_GoG->getValue(X, Y).y;
		ConstNumber = __handleNeighbor({ X, Y - 1 }, ConstNumber, vIndex, 1);
		ConstNumber = __handleNeighbor({ X - 1, Y }, ConstNumber, vIndex, 1);
		ConstNumber = __handleNeighbor({ X + 1, Y }, ConstNumber, vIndex, 1);
		ConstNumber = __handleNeighbor({ X, Y + 1 }, ConstNumber, vIndex, 1);
		m_Coeff.coeffRef(vIndex, vIndex) = -4;
		m_Const.coeffRef(vIndex, 0) = ConstNumber;
	}
}

std::optional<int> CSolverBuilder::__findUnknownIndex(const Eigen::Vector2f& vPos) {
	auto Iter = std::find(m_Unknowns.begin(), m_Unknowns.end(), vPos);
	if (Iter == m_Unknowns.end()) return std::nullopt;
	else return std::distance(m_Unknowns.begin(), Iter);
}

float CSolverBuilder::__handleNeighbor(const Eigen::Vector2f& vPos, float vConstNumber, int vIndex, float vCoeff) {
	if (vPos[0] < 0 || vPos[0] > m_Mask->getWidth() - 1 || vPos[1] < 0 || vPos[1] > m_Mask->getHeight() - 1)
		return vConstNumber;

	if (m_Mask->isEmpty(vPos[0], vPos[1]) == false)
		return vConstNumber - m_Raw->getValue(vPos[0], vPos[1]);
	else {
		auto r = __findUnknownIndex(Eigen::Vector2f(vPos[0], vPos[1]));
		_EARLY_RETURN(r.has_value() == false, "solver builder error: unknown index bug.", false);
		m_Coeff.coeffRef(vIndex, r.value()) = vCoeff;
	}
	return vConstNumber;
}
