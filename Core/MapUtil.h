#pragma once

namespace core {

	template <typename T>
	using ptr = std::shared_ptr<T>;
	using uint = std::uint32_t;

	template <typename T>
	class CMap;
	class CHeightMap;
	class CGradientMap;
	class CMaskMap;

	class MapUtil {
	public:
		static MapUtil& GetInstance();
		void Print();

		template <typename T>
		ptr<CMaskMap> geneMask(const ptr<CMap<T>>vMaskMap);

		ptr<CGradientMap> geneGradient(const ptr<CHeightMap> vHeightMap);
		ptr<CGradientMap> geneGradient(const ptr<CGradientMap> vGradientMap);
		ptr<CHeightMap> getHeightMapFromGradientMap(const ptr<CGradientMap> vGradientMap, uint vAxis);
		ptr<CHeightMap> resize(const ptr<CHeightMap> vHeightMap, uint  RescaledWidth, uint  RescaledHeight);

	private:
		MapUtil();
		~MapUtil();
		MapUtil(const MapUtil& vMapUtil) = delete;
		const MapUtil& operator=(const MapUtil& vMapUtil) = delete;
	};

	template<typename T>
	inline ptr<CMaskMap> MapUtil::geneMask(const ptr<CMap<T>> vMaskMap) {
		_EARLY_RETURN(vMaskMap->isValid() == false, "generate mask error: map is invalid.", nullptr);

		ptr<CMaskMap> pMask(new CMaskMap(vMaskMap->getWidth(), vMaskMap->getHeight(), 0));
		for (uint i = 0; i < vMaskMap->getWidth(); i++) {
			for (uint k = 0; k < vMaskMap->getHeight(); k++) {
				if (vMaskMap->isEmpty(i, k)) {
					pMask->setEmpty(i, k);
				}
			}
		}

		return pMask;
	}

}