#pragma once

namespace core {

class COutlierDetecter {
public:
	COutlierDetecter() = default;
	~COutlierDetecter() = default;

	bool run(const PC_t::Ptr& vCloud, int k, float vThres);
	void dumpOutliers(std::vector<int>& voOutliers) { voOutliers = m_Outliers; }
	void dumpInliers(std::vector<int>& voInliers) { voInliers = m_Inliers; }

private:
	std::vector<int> m_Outliers;
	std::vector<int> m_Inliers;
};

}