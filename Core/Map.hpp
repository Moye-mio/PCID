#pragma once

#include <cmath>
#include <memory>

namespace core {

using uint = std::uint32_t;
template <typename T>
using ptr = std::shared_ptr<T>;

template <typename T>
class CMap {
public:
	virtual ~CMap();

	void setWidth(uint w);
	void setHeight(uint h);
	void setSize(uint w, uint h);
	void setEmptyValue(T v);
	bool setValue(uint i, uint k, T v);
	bool setEmpty(uint i, uint k);
	bool set(const ptr<CMap> m);

	uint getArea() const { return m_Width * m_Height; }
	uint getEmptyCount() const;
	uint getWidth() const { return m_Width; }
	uint getHeight() const { return m_Height; }
	T getValue(uint i, uint k) const;

	bool isValid() const;
	bool isValid(uint i, uint k) const;
	bool isEmpty(uint i, uint k) const;
	bool isNoEmpty() const;

public:
	/* TODO: Safety Check */
	T* operator[](uint i) {
		return m_Data[i];
	}

	const T* operator[](uint i) const {
		return m_Data[i];
	}

	CMap& operator=(CMap& m) {
		if (m_Data != NULL) {
			delete m_Data;
			m_Data = NULL;
		}

		__initSize(m.getSize().first, m.getSize().second);

		for (uint i = 0; i < m_Width; i++) {
			for (uint k = 0; k < m_Height; k++) {
				m_Data[i][k] = m[i][k];
			}
		}

		return *this;
	}

protected:
	CMap(uint w, uint h, T v);

	void __release();
	void __initSize(uint w, uint h);
	void __initValue(T v);

protected:
	T**		m_Data;
	T		m_Empty;
	uint	m_Width;
	uint	m_Height;
};

template<typename T>
inline CMap<T>::CMap(uint w, uint h, T v)
	: m_Width(w)
	, m_Height(h)
{
	if (w && h) {
		__initSize(w, h);
		__initValue(v);
	}
}

template<typename T>
inline CMap<T>::~CMap() {
	__release();
}

template<typename T>
inline void CMap<T>::setWidth(uint w) {
	m_Width = w;
}

template<typename T>
inline void CMap<T>::setHeight(uint h) {
	m_Height = h;
}

template<typename T>
inline void CMap<T>::setSize(uint w, uint h) {
	if (m_Width * m_Height != 0) {
		__release();
	}
	__initSize(w, h);
}

template<typename T>
inline void CMap<T>::setEmptyValue(T v) {
	m_Empty = v;
}

template<typename T>
inline bool CMap<T>::setValue(uint i, uint k, T v) {
	_ASSERTE(i < m_Width || k < m_Height);
	_EARLY_RETURN(i >= m_Width || k >= m_Height, "map set value index invalid.", false);

	m_Data[i][k] = v;
	return true;
}

template<typename T>
inline bool CMap<T>::setEmpty(uint i, uint k) {
	return setValue(i, k, m_Empty);
}

template<typename T>
inline bool CMap<T>::set(const ptr<CMap> m) {
	_EARLY_RETURN(!m->isValid(), "Input is not valid", false);

	setSize(m->getWidth(), m->getHeight());
	for (uint i = 0; i < m->getWidth(); i++) {
		for (uint k = 0; k < m->getHeight(); k++) {
			m_Data[i][k] = m->getValue(i, k);
		}
	}
}

template<typename T>
inline uint CMap<T>::getEmptyCount() const {
	uint Count = 0;
	for (int i = 0; i < m_Width; i++) {
		for (int k = 0; k < m_Height; k++) {
			if (isEmpty(i, k)) {
				Count++;
			}
		}
	}
	return Count;
}

template<typename T>
inline T CMap<T>::getValue(uint i, uint k) const {
	return m_Data[i][k];
}

template<typename T>
inline bool CMap<T>::isValid() const {
	_EARLY_RETURN(m_Width && m_Height == 0, "map invalid: width or height == 0.", false);

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
inline bool CMap<T>::isValid(uint i, uint k) const {
	_EARLY_RETURN(i >= m_Width || k >= m_Height, "map index invalid: i or k out of scale.", false);

	if (MathUtil::isNan(m_Data[i][k])) {
		return false;
	}

	return true;
}

template<typename T>
inline bool CMap<T>::isEmpty(uint i, uint k) const {
	_EARLY_RETURN(i >= m_Width || k >= m_Height, "map index invalid: i or k out of scale.", false);

	if (isValid(i, k) && MathUtil::isEqual(m_Data[i][k], m_Empty)) {
		return true;
	}

	return false;
}

template<typename T>
inline bool CMap<T>::isNoEmpty() const {
	_EARLY_RETURN(m_Width && m_Height == 0, "map invalid: width or height == 0.", false);

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
inline void CMap<T>::__release() {
	for (int i = 0; i < m_Width; i++)
		delete[] m_Data[i];
	delete[] m_Data;
	m_Width = 0;
	m_Height = 0;
}

template<typename T>
inline void CMap<T>::__initSize(uint w, uint h) {
	m_Width = w;
	m_Height = h;

	m_Data = new T* [w];
	for (int i = 0; i < w; i++) {
		m_Data[i] = new T[h]();
	}
}

template<typename T>
inline void CMap<T>::__initValue(T v) {
	for (int i = 0; i < m_Width; i++) {
		for (int k = 0; k < m_Height; k++) {
			m_Data[i][k] = v;
		}
	}
}

}


