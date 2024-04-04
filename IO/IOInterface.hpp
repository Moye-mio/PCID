#pragma once

#include "PCLoader.h"

namespace io {

static PC_t::Ptr loadData(const std::string& vPath) {
	CPCLoader Loader;
	return Loader.loadDataFromFile(vPath);
}

}