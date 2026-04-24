#ifndef VIEWER_SYNC_THROTTLE_H
#define VIEWER_SYNC_THROTTLE_H

#include <chrono>

namespace sim {

inline bool shouldSyncViewer(std::chrono::steady_clock::time_point& next_sync,
                             const std::chrono::steady_clock::duration& period) {
	const auto now = std::chrono::steady_clock::now();
	if (now < next_sync) {
		return false;
	}

	next_sync = now + period;
	return true;
}

}  // namespace sim

#endif  // VIEWER_SYNC_THROTTLE_H
