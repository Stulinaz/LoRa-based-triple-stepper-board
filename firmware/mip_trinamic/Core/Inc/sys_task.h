#ifndef CORE_INC_SYS_TASK_H_
#define CORE_INC_SYS_TASK_H_

/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "FreeRTOS.h"
#include "task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FW_VERSION        (0x01U)

#define CLI_TASK_PRIORITY       (0x02U)
#define DIAGNOSIS_TASK_PRIORITY (0x02U)
#define LORA_TASK_PRIORITY      (0x02U)

/*******************************************************************************
 * API
 ******************************************************************************/
extern TaskHandle_t cli_task_handle;
extern TaskHandle_t stepper_diagnosis_task_handle;
extern TaskHandle_t lora_task_handle;

#endif
