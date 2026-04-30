#ifndef CONTACT_SCHEDULE_H
#define CONTACT_SCHEDULE_H

#include <vector>

struct ContactScheduleOverrideStep {
    bool enabled{false};
    bool leftContact{true};
    bool rightContact{true};
    double leftNormalForceScale{1.0};
    double rightNormalForceScale{1.0};
};

struct ContactScheduleOverride {
    std::vector<ContactScheduleOverrideStep> steps;
};

#endif  // CONTACT_SCHEDULE_H
