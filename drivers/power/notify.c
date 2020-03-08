/*
 * All the power notify logic
 *
 * notifier functions originally based on those in kernel/sys.c
 * but fixed up to not be so broken.
 *
 */


#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/power/notify.h>

static BLOCKING_NOTIFIER_HEAD(power_notifier_list);

/**
 * power_register_notify - register a notifier callback whenever a power change happens
 * @nb: pointer to the notifier block for the callback events.
 *
 * These changes are either POWER devices or busses being added or removed.
 */
void power_register_notify(struct notifier_block *nb)
{
	blocking_notifier_chain_register(&power_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(power_register_notify);

/**
 * power_unregister_notify - unregister a notifier callback
 * @nb: pointer to the notifier block for the callback events.
 *
 * power_register_notify() must have been previously called for this function
 * to work properly.
 */
void power_unregister_notify(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&power_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(power_unregister_notify);

/**
 * Notify on battery capacity change.
 * @capacity Valid range: 0~100, others will be reported in -EOVERFLOW.
 */
void power_notify_capacity_device(int capacity)
{
	uintptr_t c = (uintptr_t)capacity;
	if (capacity < 0 || capacity > 100)
		capacity = -EOVERFLOW;
	blocking_notifier_call_chain(&power_notifier_list, POWER_CAPACITY_CHANGE, (void*)c);
}
