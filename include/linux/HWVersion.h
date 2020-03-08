#ifndef __HWVERSION_H
#define __HWVERSION_H

#define HW_ID_EVB	0x00000000
#define HW_ID_SR	0x00000001
#define HW_ID_ER	0x00000002
#define HW_ID_ER2   0x00000003
#define HW_ID_PR	0x00000004
#define HW_ID_MP	0x00000005

#define PROJ_ID_ZT581KL	0x00000000
#define PROJ_ID_ZT500KL	0x00000001
#define PROJ_ID_Z581KL	0x00000002
#define PROJ_ID_Z500KL  0x00000003

#define PROJ_ID_ZT582KL	0x00000000
#define PROJ_ID_Z582KL	0x00000001

#define HW_ID_SR_STR        "SR"
#define HW_ID_ER_STR        "ER"
#define HW_ID_ER2_STR       "ER2"
#define HW_ID_PR_STR        "PR"
#define HW_ID_UNKNOWN_STR   "unknown"

extern int Read_HW_ID(void);
extern int Read_PROJ_ID(void);

#endif
