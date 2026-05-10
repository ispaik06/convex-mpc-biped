#ifndef DEBUG_VISUALIZATION_H
#define DEBUG_VISUALIZATION_H

#include <string>

#include "cppTypes.h"

enum class TouchdownMarkerContactState {
	Swing,
	Stance,
	EarlyContact,
	LateContact,
};

template <typename T>
struct DebugVizMarker {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	std::string name;
	Vec3<T> position_W = Vec3<T>::Zero();
	Quat<T> orientation_W = Quat<T>::Identity();
	bool hasRgba{false};
	Vec4<T> rgba = Vec4<T>::Ones();
	bool active{true};
};

template <typename T>
Vec4<T> touchdownMarkerRgba(const TouchdownMarkerContactState state) {
	switch (state) {
		case TouchdownMarkerContactState::Stance:
			return Vec4<T>(1.0, 0.82, 0.05, 0.95);
		case TouchdownMarkerContactState::Swing:
			return Vec4<T>(0.2, 0.8, 0.25, 0.95);
		case TouchdownMarkerContactState::EarlyContact:
			return Vec4<T>(1.0, 0.0, 0.83, 0.95);
		case TouchdownMarkerContactState::LateContact:
			return Vec4<T>(0.0, 0.1, 1.0, 0.95);
	}

	return Vec4<T>(1.0, 1.0, 1.0, 0.95);
}

template <typename T>
Vec4<T> touchdownMarkerRgba(const bool isStance) {
	return touchdownMarkerRgba<T>(isStance ? TouchdownMarkerContactState::Stance
	                                       : TouchdownMarkerContactState::Swing);
}

template <typename T>
struct DebugVizState {
	vectorAligned<DebugVizMarker<T>> markers;

	void clear() {
		markers.clear();
	}
};

#endif  // DEBUG_VISUALIZATION_H
