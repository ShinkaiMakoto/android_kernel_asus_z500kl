#include <linux/notifier.h>

/* Events from the battery sub system */
#define POWER_CAPACITY_CHANGE          0x0001
#define POWER_TEMPERATURE_CHANGE       0x0002
extern void power_register_notify(struct notifier_block *nb);
extern void power_unregister_notify(struct notifier_block *nb);
