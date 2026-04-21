#ifndef DEBUG_VISUALIZATION_H
#define DEBUG_VISUALIZATION_H

#include <string>

#include "cppTypes.h"

template <typename T>
struct DebugVizMarker {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	std::string name;
	Vec3<T> position_W = Vec3<T>::Zero();
	Quat<T> orientation_W = Quat<T>::Identity();
	bool active{true};
};

template <typename T>
struct DebugVizState {
	vectorAligned<DebugVizMarker<T>> markers;

	void clear() {
		markers.clear();
	}
};

#endif  // DEBUG_VISUALIZATION_H
