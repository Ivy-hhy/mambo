
#include <zephyr/device.h>

#define DMOTOR_NODE1 DT_NODELABEL(dm_motor000)
#define DMOTOR_NODE2 DT_NODELABEL(dm_motor001)
#define DMOTOR_NODE3 DT_NODELABEL(dm_motor002)
#define DMOTOR_NODE4 DT_NODELABEL(dm_motor003)

const struct device *dm_motor000 = DEVICE_DT_GET(DMOTOR_NODE1);
const struct device *dm_motor001 = DEVICE_DT_GET(DMOTOR_NODE2);
const struct device *dm_motor002 = DEVICE_DT_GET(DMOTOR_NODE3);
const struct device *dm_motor003 = DEVICE_DT_GET(DMOTOR_NODE4);


