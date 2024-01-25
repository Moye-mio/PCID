#pragma once

namespace core {

class CHeightMap;
class CHeightMap2PCMapper {
public:

	PC_t::Ptr map2PC(const std::shared_ptr<CHeightMap> vHole, const std::shared_ptr<CHeightMap> vFilled, int vPointNumberPerPixel);


private:
	void __geneSamplePoints(const std::shared_ptr<CHeightMap> vHole, std::vector<vec2f>& voSampleSequence, int vPointNumberPerPixel);
	PC_t::Ptr __geneNewCloud(const std::shared_ptr<CHeightMap> vFilled, std::vector<vec2f>& vSampleSequence);
};

}

