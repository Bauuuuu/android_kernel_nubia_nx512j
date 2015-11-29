#ifndef ZTE_LCD_DSI_H
#define ZTE_LCD_DSI_H

int zte_dsi_panel_reg_read(struct mdss_dsi_ctrl_pdata *ctrl, u8 reg, u8 *rbuf, u8 len);
/* EXTERN FUNCTION */
extern void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds);

#endif //ZTE_LCD_DSI_H
