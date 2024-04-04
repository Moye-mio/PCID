#pragma once

namespace alg {

enum class ECVInpaintingType {
	TELEA,
	NS
};

enum class EInpaintMode {
	NS,
	TEALA,
	PM,
	Mix,
};

enum class EPoissonGradient {
	PM,
	MIX,
	NS,
	TELEA,
	EB,
	GB,
};

}

