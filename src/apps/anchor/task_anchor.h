


#ifndef __ANCHOR__H__
#define __ANCHOR__H__ 1

#ifdef __cplusplus
 extern "C" {
#endif

#include "app.h"

void anchor_helper(void const *argument);
void anchor_terminate(void);

extern portMUX_TYPE anchorTaskMux;

#ifdef __cplusplus
}
#endif

#endif /* __ANCHOR__H__ */