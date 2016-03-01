#pragma once

void lvez_init(const sonarHardware_t *sonarHardware, sonarRange_t *sonarRange);

int32_t lvez_get_distance(void);
