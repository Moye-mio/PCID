#pragma once

namespace core {
class CHeightMap;
class CGradientMap;
}

template <typename T>
using ptr = std::shared_ptr<T>;

class CDGI {
public:

	bool setResolution(uint vRes);
	bool run(const PC_t::Ptr vInput, PC_t::Ptr voOutput);

private:
	ptr<core::CGradientMap> __inpaintImage(const ptr<core::CGradientMap> vRaw, const ptr<core::CMaskMap> vMask);
	ptr<core::CHeightMap> __solveEquations(const ptr<core::CHeightMap> vInput, const ptr<core::CGradientMap> vGradientFilled, const ptr<core::CGradientMap> vGog);
	PC_t::Ptr __genePointCloud(const ptr<core::CHeightMap> vInput, const ptr<core::CHeightMap> vFilled, int vPointNumberPerPixel);

private:
	uint m_Res;

};
