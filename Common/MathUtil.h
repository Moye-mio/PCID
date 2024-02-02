#pragma once

#include <cmath>
#include <random>
#include "GlobalValue.h"
#include "BasicMath.h"

class MathUtil {
public:
	static bool isFloatEqual(float a, float b) {
		if (std::fabsf(a - b) < EPSILON) {
			return true;
		}
		else {
			return false;
		}
	}

	template <typename T>
	static bool isEqual(T a, T b) {
		return a == b;
	}

	template <>
	static bool isEqual<float>(float a, float b) {
		return isFloatEqual(a, b);
	}

	template <>
	static bool isEqual<vec2f>(vec2f a, vec2f b) {
		return isFloatEqual(a.x, b.x) && isFloatEqual(a.y, b.y);
	}

	template <>
	static bool isEqual<std::pair<std::uint32_t, std::uint32_t>>(std::pair<std::uint32_t, std::uint32_t> a, std::pair<std::uint32_t, std::uint32_t> b) {
		return a.first == b.first && a.second == b.second;
	}

	static bool isFloatNan(float a) {
		return std::isnan(a);
	}

	template <typename T>
	static bool isNan(T a) {
		return false;
	}

	template <>
	static bool isNan<float>(float a) {
		return isFloatNan(a);
	}

	template <>
	static bool isNan<vec2f>(vec2f a) {
		return isFloatNan(a.x) || isFloatNan(a.y);
	}

	static float linearInterpolate(float a, float b, float i) {
		return (1 - i) * a + i * b;
	}

	static float bilinearInterpolate(float a, float b, float c, float d, float i, float k) {
		return linearInterpolate(linearInterpolate(a, b, i), linearInterpolate(c, d, i), k);
	}

	/* [from, to] */
	static int geneRandomInt(int from, int to);
	static float geneRandomReal(float from, float to);

private:

};





