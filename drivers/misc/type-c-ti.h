#ifndef __TYPE_C_TI_H__
#define __TYPE_C_TI_H__

#define DBG_ERR		0x0001  /* error returns */
#define DBG_INIT	0x0002	/* debug initialization */
#define DBG_HW		0x0004	/* debug hardware access, e.g i2c access. */
#define DBG_SM		0x0008	/* debug state machine */
#define DBG_ALL		0xffff
#define DBG_NONE	0x0000

#ifdef DEBUG
#define DEBUG_LEVEL	(DBG_ALL)
#else
#define DEBUG_LEVEL	(DBG_ERR|DBG_INT)
#endif

#define DRV_NAME "TUSB320"
#define PIN_NAME(p) (DRV_NAME"-"p)

#define DBG(level, fmt, ...)						\
	do {								\
		if ((level) & DEBUG_LEVEL)                              \
			pr_info(DRV_NAME": " fmt, ## __VA_ARGS__);	\
	} while (0)

#define DBG_RL(level, fmt, ...)						\
	do {								\
		if ((level) & DEBUG_LEVEL)                              \
			pr_info_ratelimited(DRV_NAME": " fmt, ## __VA_ARGS__); \
	} while (0)

#endif
