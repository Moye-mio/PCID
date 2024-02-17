#pragma once

#include <cmath>
#include <memory>
#include<MathUtil.h>
#include <cstdint>

namespace core {

	using uint = std::uint32_t;
	template <typename T>
	using ptr = std::shared_ptr<T>;

	template <typename T>
	class CMap {
	public:
		virtual ~CMap();

		void setWidth(uint vWidth);
		void setHeight(uint vHeight);
		void setSize(uint vWidth, uint vHeight);
		void setEmptyValue(T vValue);
		bool setValue(uint vXPosition, uint vYPosition, T vValue);
		bool setEmpty(uint vXPosition, uint vYPosition);
		bool set(const ptr<CMap> vMap);

		uint getArea() const { return m_Width * m_Height; }
		uint getEmptyCount() const;
		uint getWidth() const { return m_Width; }
		uint getHeight() const { return m_Height; }
		T getValue(uint vXPosition, uint vYPosition) const;

		bool isValid() const;
		bool isValid(uint vXPosition, uint vYPosition) const;
		bool isEmpty(uint vXPosition, uint vYPosition) const;
		bool isNoEmpty() const;

	public:

		T* operator[](uint vXPosition) {
			if (vXPosition >= m_Width) {
				throw std::out_of_range("Input is not valid");
			}
			return m_Data[vXPosition];
		}

		const T* operator[](uint vXPosition) const {
			_EARLY_RETURN(vXPosition >= m_Width, "Input is not valid", false);
			return m_Data[vXPosition];
		}

		CMap& operator=(CMap& vMap) {
			if (m_Data != nullptr) {
				delete[] m_Data;
				m_Data = nullptr;
			}

			_initSize(vMap.getSize().first, vMap.getSize().second);

			for (uint i = 0; i < m_Width; i++) {
				for (uint k = 0; k < m_Height; k++) {
					m_Data[i][k] = vMap[i][k];
				}
			}

			return *this;
		}

	protected:
		CMap(uint vWidth, uint vHeight, T vValue);

		void _release();
		void _initSize(uint vWidth, uint vHeight);
		void _initValue(T vValue);

	protected:
		T** m_Data;
		T		m_Empty;
		uint	m_Width;
		uint	m_Height;
	};

	template<typename T>
	inline CMap<T>::CMap(uint vWidth, uint vHeight, T vValue)
		: m_Width(vWidth)
		, m_Height(vHeight)
	{
		if (vWidth && vHeight) {
			_initSize(vWidth, vHeight);
			_initValue(vValue);
		}
	}

	template<typename T>
	inline CMap<T>::~CMap() {
		_release();
	}

	template<typename T>
	inline void CMap<T>::setWidth(uint vWidth) {
		m_Width = vWidth;
	}

	template<typename T>
	inline void CMap<T>::setHeight(uint vHeight) {
		m_Height = vHeight;
	}

	template<typename T>
	inline void CMap<T>::setSize(uint vWidth, uint vHeight) {
		if (m_Width * m_Height != 0) {
			_release();
		}
		_initSize(vWidth, vHeight);
	}

	template<typename T>
	inline void CMap<T>::setEmptyValue(T vValue) {
		m_Empty = vValue;
	}

	template<typename T>
	inline bool CMap<T>::setValue(uint vXPosition, uint vYPosition, T vValue) {
		_ASSERTE(vXPosition < m_Width && vYPosition < m_Height);
		_EARLY_RETURN(vXPosition >= m_Width || vYPosition >= m_Height, "map set value index invalid.", false);

		m_Data[vXPosition][vYPosition] = vValue;
		return true;
	}

	template<typename T>
	inline bool CMap<T>::setEmpty(uint vXPosition, uint vYPosition) {
		return setValue(vXPosition, vYPosition, m_Empty);
	}

	template<typename T>
	inline bool CMap<T>::set(const ptr<CMap> vMap) {
		_EARLY_RETURN(!vMap->isValid(), "Input is not valid", false);

		setSize(vMap->getWidth(), vMap->getHeight());
		for (uint i = 0; i < vMap->getWidth(); i++) {
			for (uint k = 0; k < vMap->getHeight(); k++) {
				m_Data[i][k] = vMap->getValue(i, k);
			}
		}
		return true;
	}

	template<typename T>
	inline uint CMap<T>::getEmptyCount() const {
		uint Count = 0;
		for (uint i = 0; i < m_Width; i++) {
			for (uint k = 0; k < m_Height; k++) {
				if (isEmpty(i, k)) {
					Count++;
				}
			}
		}
		return Count;
	}

	template<typename T>
	inline T CMap<T>::getValue(uint vXPosition, uint vYPosition) const {
		assert(vXPosition < m_Width && vYPosition < m_Height && "map index invalid: vXPosition or vYPosition out of scale.");
		return m_Data[vXPosition][vYPosition];
	}

	template<typename T>
	inline bool CMap<T>::isValid() const {


		for (uint i = 0; i < m_Width; i++) {
			for (uint k = 0; k < m_Height; k++) {
				if (!isValid(i, k)) {
					return false;
				}
			}
		}

		return true;
	}

	template<typename T>
	inline bool CMap<T>::isValid(uint vXPosition, uint vYPosition) const {
		_EARLY_RETURN(vXPosition >= m_Width || vYPosition >= m_Height, "map index invalid: vXPosition or vYPosition out of scale.", false);

		if (MathUtil::isNan(m_Data[vXPosition][vYPosition])) {
			return false;
		}

		return true;
	}

	template<typename T>
	inline bool CMap<T>::isEmpty(uint vXPosition, uint vYPosition) const {
		_EARLY_RETURN(vXPosition >= m_Width || vYPosition >= m_Height, "map index invalid: vXPosition or vYPosition out of scale.", false);
		_EARLY_RETURN(!isValid(vXPosition, vYPosition), "map invalid: " + std::to_string(vXPosition) + ", " + std::to_string(vYPosition), false);

		if (MathUtil::isEqual(m_Data[vXPosition][vYPosition], m_Empty)) {
			return true;
		}

		return false;
	}

	template<typename T>
	inline bool CMap<T>::isNoEmpty() const {


		for (uint i = 0; i < m_Width; i++) {
			for (uint k = 0; k < m_Height; k++) {
				if (isEmpty(i, k)) {
					return false;
				}
			}
		}

		return true;
	}

	template<typename T>
	inline void CMap<T>::_release() {
		for (uint i = 0; i < m_Width; i++)
			delete[] m_Data[i];
		delete[] m_Data;
		m_Width = 0;
		m_Height = 0;
	}

	template<typename T>
	inline void CMap<T>::_initSize(uint vWidth, uint vHeight) {
		m_Width = vWidth;
		m_Height = vHeight;

		m_Data = new T * [vWidth];
		for (uint i = 0; i < vWidth; i++) {
			m_Data[i] = new T[vHeight]();
		}
	}

	template<typename T>
	inline void CMap<T>::_initValue(T vValue) {
		for (uint i = 0; i < m_Width; i++) {
			for (uint k = 0; k < m_Height; k++) {
				m_Data[i][k] = vValue;
			}
		}
	}

}


