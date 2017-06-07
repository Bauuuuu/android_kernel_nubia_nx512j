#ifndef __LINUX_BQ27520_CHARGER_H__
#define __LINUX_BQ27520_CHARGER_H__



int bq27520_get_batt_voltage(void);
int bq27520_get_batt_temp(void);
int bq27520_get_batt_soc(int *battery_soc);
int bq27520_report_batt_capacity(void);
int bq27520_get_ibatt_now(void);
int bq27520_set_power_supply(struct power_supply *batt_psy);

#endif
