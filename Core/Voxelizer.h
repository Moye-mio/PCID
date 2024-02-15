#pragma once

namespace core {

class CVoxelizer {
public:

	PC_t::Ptr run(const PC_t::Ptr vCloud);
	float getScale() { return m_Scale; }

private:
	Point_t __calcNearestCenter(const Point_t& vPt);

private:
	float m_Scale;

};

}
