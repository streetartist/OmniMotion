/**
 * @file app.h
 * @brief OmniMotion DC电机应用层 C接口
 */

#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
extern "C" {
#endif

void app_setup(void);
void app_loop(void);
void app_control_loop(void);  /* 由TIM4中断调用, 1kHz */

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */
