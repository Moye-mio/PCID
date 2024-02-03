#pragma once

#include "Proj.h"

namespace core {
class CProjManager {
public:

	bool calcDirection(std::vector<SProj>& vioProj);

private:
	void __castProj2PCLpt(const std::vector<SProj>& vProj, const PC_t::Ptr& voCloud);
	bool __isSameDirection(const SProj& vProj, const Point_t& vPt);
};
}
