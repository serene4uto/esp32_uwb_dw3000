/**
 * @file    header for task_tag.h
 *
 * @brief   Tag Application Layer
 *          RTOS tag implementation
 *
 * @attention //TODO: Add attention here
 *
 * @author Nguyen Ha Trung
 */

#ifndef __TAG_TASK__H__
#define __TAG_TASK__H__ 1

#include "app.h"

#ifdef __cplusplus
 extern "C" {
#endif



void tag_helper(void const *argument);
void tag_terminate(void);


// extern portMUX_TYPE tagTaskMux;

#ifdef __cplusplus
}
#endif

#endif /* __TWR_TASK__H__ */