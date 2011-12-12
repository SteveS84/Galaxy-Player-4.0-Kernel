#ifndef __SEC_BATTERY_H
#define __SEC_BATTERY_H

struct sec_bat_platform_data {
	void (*register_callbacks)(struct max8998_charger_callbacks *ptr);
	bool (*jig_cb) (void);
	enum cable_type_t (*cable_cb) (void);
};

#endif
