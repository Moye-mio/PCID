#include "pch.h"
#include "MaskMap.h"

using namespace core;

core::CMaskMap::CMaskMap()
	: CMap<std::uint8_t>(0, 0, 0) {
	m_Empty = 255;
}

core::CMaskMap::CMaskMap(uint vWidth, uint vHeight)
	: CMap<std::uint8_t>(vWidth, vHeight, 0) {
	m_Empty = 255;
}

core::CMaskMap::CMaskMap(uint vWidth, uint vHeight, std::uint8_t vValue)
	: CMap<std::uint8_t>(vWidth, vHeight, vValue) {
	m_Empty = 255;
}