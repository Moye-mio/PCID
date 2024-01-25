#pragma once

namespace core {

class CHeightMap;
class CHeightMapSampler {
public:

	bool sample(const std::shared_ptr<CHeightMap> vHole, const std::shared_ptr<CHeightMap> vFilled, int vPointNumberPerPixel, std::vector<vec3f>& voSamples);

private:
	void __geneSamplePoints(const std::shared_ptr<CHeightMap> vHole, std::vector<vec3f>& voSampleSequence, int vPointNumberPerPixel);
	void __sample(const std::shared_ptr<CHeightMap> vFilled, std::vector<vec3f>& vioSampleSequence);
};

}

