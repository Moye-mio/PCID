#pragma once

namespace core {

template <typename T>
using ptr = std::shared_ptr<T>;
using uint = std::uint32_t;

class CHeightMap;
class CHeightMapGenerator {
public:

	ptr<CHeightMap> generate(const PC_t::Ptr vCloud, uint vResX, uint vResY, bool vIsDetailed = false);
	ptr<CHeightMap> generate(const std::vector<vec3f>& vPts, uint vResX, uint vResY, bool vIsDetailed = false);
	void dumpProjCoor(std::vector<vec2i>& voProjCoor) { voProjCoor = m_ProjCoor; }

private:
	ptr<CHeightMap> __generate(const std::vector<vec3f>& vPts, uint vResX, uint vResY, bool vIsDetailed = false);
	vec2i __mapUV2Pixel(const vec2f& vUV, uint vResX, uint vResY);

private:
	std::vector<vec2i> m_ProjCoor;
};

}
