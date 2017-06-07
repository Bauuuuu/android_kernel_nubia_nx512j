#ifndef __LINUX_ZTEMT_SPMI_CHARGER_H__
#define __LINUX_ZTEMT_SPMI_CHARGER_H__

int  ztemt_qpnp_spmi_init(void);
bool pm_battery_present(void);
int qpnp_read_shutdown_ocv_soc(int *soc, int *vbatt_mv);
int qpnp_backup_ocv_soc(int soc, int vbatt_mv);

#endif
