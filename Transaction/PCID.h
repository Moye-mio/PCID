#pragma once

namespace core {
struct SAABB;
}

template <typename T>
using ptr = std::shared_ptr<T>;

struct SProj;

class CPCID {
public:

	bool setResolution(uint vWorkRes, uint vRecoRes);
	bool run(const PC_t::Ptr vInput, const PC_t::Ptr vSub, PC_t::Ptr& voOutput);

private:
	void __prepareHeightMapData(const std::vector<SProj>& vProjs, std::vector<vec3f>& voBriefProjs);
	ptr<core::CGradientMap> __inpaintImage(const ptr<core::CGradientMap> vRaw, const ptr<core::CMaskMap> vMask);
	ptr<core::CHeightMap> __solveEquations(const ptr<core::CHeightMap> vInput, const ptr<core::CGradientMap> vGradientFilled, const ptr<core::CGradientMap> vGog);
	PC_t::Ptr __genePointCloud(const std::shared_ptr<pcl::on_nurbs::FittingSurface> vFit, const ptr<core::CHeightMap> vInput, const ptr<core::CHeightMap> vFilled, const core::SAABB& vBox, int vPointNumberPerPixel);
	ptr<core::CHeightMap> __denoiseHeightMap(const ptr<core::CHeightMap> vHeightMap, uint vThres);
	void __recoverHeightMap(const ptr<core::CHeightMap> vRawHeightMap, ptr<core::CHeightMap>& vioFilledHeightMap);
	bool __removeExcessPoints(const PC_t::Ptr vInput, PC_t::Ptr& vioFilled);

private:
	uint m_WorkRes;				/* resolution for image inpainting */
	uint m_RecoRes;				/* resolution for reconstruction */

};
