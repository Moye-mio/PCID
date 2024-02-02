#pragma once

#include "MaskMap.h"

namespace core {

template <typename T>
using ptr = std::shared_ptr<T>;
using uint = std::uint32_t;

template <typename T>
class CMap;
class CHeightMap;
class CGradientMap;

class MapUtil {
public:

	template <typename T>
	static ptr<CMaskMap> geneMask(const ptr<CMap<T>> m);

	static ptr<CGradientMap> geneGradient(const ptr<CHeightMap> h);
	static ptr<CGradientMap> geneGradient(const ptr<CGradientMap> g);
	static ptr<CHeightMap> getHeightMapFromGradientMap(const ptr<CGradientMap> g, uint axis);
	static ptr<CHeightMap> resize(const ptr<CHeightMap> h, uint rx, uint ry);

private:

};

template<typename T>
inline ptr<CMaskMap> MapUtil::geneMask(const ptr<CMap<T>> m) {
	_EARLY_RETURN(m->isValid() == false, "generate mask error: map is invalid.", nullptr);

	ptr<CMaskMap> pMask(new CMaskMap(m->getWidth(), m->getHeight(), 0));
	for (uint i = 0; i < m->getWidth(); i++) {
		for (uint k = 0; k < m->getHeight(); k++) {
			if (m->isEmpty(i, k)) {
				pMask->setEmpty(i, k);
			}
		}
	}

	return pMask;
}



}