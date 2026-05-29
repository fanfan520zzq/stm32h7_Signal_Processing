#ifndef __SERIAL_CMD_TASK_H
#define __SERIAL_CMD_TASK_H

#include <stdint.h>

/**
 * @brief 处理串口接收到的自定义命令
 *        建议在主循环中轮询调用
 */
void SerialCmdTask_Poll(void);

#endif /* __SERIAL_CMD_TASK_H */
