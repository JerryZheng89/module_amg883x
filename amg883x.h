#ifndef _UAPI_MODULETEST_AMG883X_H
#define _UAPI_MODULETEST_AMG883X_H

#include <linux/types.h>

enum amg883x_mode {
	AMG883x_MODE_NORMAL	= 0x00,
	AMG883x_MODE_SLEEP	= 0x10,
	AMG883x_MODE_SDB_60	= 0x20,
	AMG883x_MODE_SDB_10	= 0x21,
};

struct amg883x_read_data {
	__u8 mode;
	__u8 fps;	
	union {
		struct {
			__u8 reserved	: 5;
			__u8 intMode	: 1;
			__u8 intEN	: 1;
		}int_control_reg;
		__u8 int_control;
	};
	union {
		struct {
			__u8 reserved1	: 4;
			__u8 temper_over	: 1;
			__u8 ir_over	: 1;
			__u8 intf		: 1;
			__u8 reserved2	: 1;
		}status_reg;
		__u8 status;
	};
	__u16 int_high_level;
	__u16 int_low_level;
	__u16 int_hysteresis_level;
	__u16 thermistor;
	__u8 int_pixel_table[8];
	__u16 pixel_value_table[64];
};

enum write_command {
	AMG883X_WR_FLAG_RESET 	= 1 << 0,	
	AMG883X_WR_FLAG_CLEAR 	= 1 << 1,	
	AMG883X_WR_FLAG_FPS	= 1 << 2,	
	AMG883X_WR_FLAG_INTC	= 1 << 3,	
};

enum reset_mode {
	AMG883X_RESETMODE_FLAG		= 0x30,
	AMG883X_RESETMODE_INITIAL	= 0x3F,
};

struct amg883x_write_data {
	__u8 reset_reg;
	__u8 fps;
	union {
		struct {
			__u8 reserved1	: 1; /* bit 0 */
			__u8 int_clear	: 1; /* bit 1 */
			__u8 ovs_clear	: 1; /* bit 2 */
			__u8 ovt_clear	: 1; /* bit 3 */
			__u8 reserved2	: 4; /* bit 4~7 */
		}clear_reg_bit;
		__u8 clear_reg;
	};

	union {
		struct {
			__u8 int_en	: 1; /* bit 0 */
			__u8 int_mod	: 1; /* bit 1 */
			__u8 reserved 	: 5; /* bit 2~7 */
		}int_ctrl_bit;
		__u8 int_ctrl_reg;
	};
	__u8 wr_flag;
};

#define AMG_CMD_PW_ON		_IO('m', 1)
#define AMG_CMD_PW_OFF		_IO('m', 2)
#define AMG_CMD_RD_TEMP 	_IOR('m', 3, int)
#define AMG_CMD_WR_FPS 		_IOW('m', 4, int)
#define AMG_CMD_RD_FPS 		_IOW('m', 5, int)
#define AMG_CMD_WR_ALL		_IOW('m', 6, struct amg883x_write_data)
#define AMG_CMD_RD_ALL		_IOR('m', 7, struct amg883x_read_data)

#endif