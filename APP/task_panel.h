#ifndef __TASK_PANEL_H
#define __TASK_PANEL_H

/* ===========================================================================
 * 双面板任务
 *
 * PANEL0: 纯显示面板 (无按键)
 *   - 柜温显示
 *   - 除霜图标 (Def)
 *   - 满水图标 (Humi)
 *   - 蒸发风机图标 (Fan)
 *   - 报警指示 (Ref 闪烁)
 *
 * PANEL1: 操作面板 (8按键 + 显示)
 *   1. Reset键     一键复位
 *   2. Set键       设置目标温度
 *   3. 调温上键    设置温度及翻页
 *   4. 调温下键    设置温度及翻页
 *   5. 一键除霜键  手动除霜
 *   6. 照明键      照明灯手动开关
 *   7. 点检键      保留键
 *   8. 电源开关键  系统电源开关
 * =========================================================================== */

void Task_Panel0_Process(void const *argument);  /* PANEL0 显示任务 */
void Task_Panel1_Process(void const *argument);  /* PANEL1 操作任务 */

/* 兼容旧接口: 默认调用 PANEL0 + PANEL1 */
void Task_Panel_Process(void const *argument);

#endif
