#pragma once

#include "AABB.h"

namespace core {

class CPCMapper {
public:

	PC_t::Ptr map(const SAABB& vBox, const std::vector<vec3f>& vSamples);


private:


};

}
