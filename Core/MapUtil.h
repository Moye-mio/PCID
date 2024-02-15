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
		ptr<CMaskMap> geneMask(const ptr<CMap<T>>vpM);

		ptr<CGradientMap> geneGradient(const ptr<CHeightMap> vpH);
		ptr<CGradientMap> geneGradient(const ptr<CGradientMap> vpG);
		ptr<CHeightMap> getHeightMapFromGradientMap(const ptr<CGradientMap> vpG, uint vAxis);
		ptr<CHeightMap> resize(const ptr<CHeightMap> h, uint rx, uint ry);

	private:
		MapUtil();
		~MapUtil();
		MapUtil(const MapUtil& vMapUtil) = delete;
		const MapUtil& operator=(const MapUtil& vMapUtil) = delete;
	};

	template<typename T>
	inline ptr<CMaskMap> MapUtil::geneMask(const ptr<CMap<T>> vpM) {
		_EARLY_RETURN(vpM->isValid() == false, "generate mask error: map is invalid.", nullptr);

		ptr<CMaskMap> pMask(new CMaskMap(vpM->getWidth(), vpM->getHeight(), 0));
		for (uint i = 0; i < vpM->getWidth(); i++) {
			for (uint k = 0; k < vpM->getHeight(); k++) {
				if (vpM->isEmpty(i, k)) {
					pMask->setEmpty(i, k);
				}
			}
		}

		return pMask;
	}

}