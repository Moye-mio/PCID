#pragma once

namespace core {
class CHeightMap;
class CGradientMap;
struct SAABB;
}

template <typename T>
using ptr = std::shared_ptr<T>;

class CDGI {
public:

	bool setResolution(uint vWorkRes, uint vRecoRes);
	bool run(const PC_t::Ptr vInput, PC_t::Ptr& voOutput);

private:
	ptr<core::CGradientMap> __inpaintImage(const ptr<core::CGradientMap> vRaw, const ptr<core::CMaskMap> vMask);
	ptr<core::CHeightMap> __solveEquations(const ptr<core::CHeightMap> vInput, const ptr<core::CGradientMap> vGradientFilled, const ptr<core::CGradientMap> vGog);
	PC_t::Ptr __genePointCloud(const ptr<core::CHeightMap> vInput, const ptr<core::CHeightMap> vFilled, const core::SAABB& vBox, int vPointNumberPerPixel);

private:
	uint m_WorkRes;				/* resolution for image inpainting */
	uint m_RecoRes;				/* resolution for reconstruction */

};
