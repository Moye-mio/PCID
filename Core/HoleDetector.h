#pragma once

namespace core {

class CHoleDetector {

public:

	bool run(const PC_t::Ptr vCloud, float vRadius, float vThreshold);
	void dumpBoundaryIndices(std::vector<int>& voIndices) { voIndices = m_BounIndices; }

private:
	void __removeBorder(const PC_t::Ptr vCloud);
	void __removeOutlier(const PC_t::Ptr vCloud, float vRadius);
	void __removeFinal(const PC_t::Ptr vCloud);

private:
	std::vector<int> m_BounIndices;


};

}