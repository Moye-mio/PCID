#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <cstdlib>
#include <variant>

/* PCL */
#include <pcl/io/ply_io.h>

/* COMMON */
#include "Common.h"

/* IO */
#include "PCLoader.h"

/* TRANS */
#include "NBFM.h"

int main() {
	const std::string InputLoadPath = MAINEXPERIMENT_DIR + std::string("hole/6.ply"); 
	io::CPCLoader Loader;
	PC_t::Ptr pInput = Loader.loadDataFromFile(InputLoadPath);
	
	CNBFM NBFM;
	NBFM.run(pInput);


	return 0;
}


