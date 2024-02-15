#pragma once

#include "Map.hpp"

namespace core {

	/*
	 *	255	->	empty
	 *	0	->	filled
	 */

	class CMaskMap : public CMap<std::uint8_t> {
	public:
		CMaskMap();
		CMaskMap(uint vWidth, uint vHeight);
		CMaskMap(uint vWidth, uint vHeight, std::uint8_t vValue);

	private:

	};

}