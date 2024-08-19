#ifndef _LINUX_PSM_MEMIF_H
#define _LINUX_PSM_MEMIF_H

#define EMMC_STOP_LEN 9
#define KILL_VMS_LEN 8
#define LAUNCH_LEN 6
#define ALPHA_LEN 5

#define HV_STATUS_LEN 9
#define HV_FLAG	0
#define ALPHA_VM_FLAG	1

enum Mcm_hv_status {
	MCM_HV_STATUS_UNKNOWN = 0,
	MCM_HV_STATUS_ON = 1,
	MCM_HV_STATUS_ONFF= 3,
};

enum Mcm_commands_client {
	MCM_CMD_NONE = 0,
	MCM_CMD_LAUNCH = 1,
	MCM_CMD_KILL= 3,
	MCM_CMD_EMMCSTOP= 7,
};

enum Mcm_cmd_api_version {
	Mcm_cmd_api_version = 1,
};

#define MCM_CMD_HEADER_MAGIC "MCM_dAtA"

struct Mcm_cmd_header {
	/* MCM_CMD_HEADER_MAGIC */
	char	magic[8];
	/* Mcm_cmd_api_version */
	uint32_t api_version;
	uint32_t _pad[5];
};

struct Mcm_cmd_client {
	uint32_t status;
	uint32_t command;
	/* error and command args not used, padding used instead here */
	uint32_t _pad[6];
};
#endif
