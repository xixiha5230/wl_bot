#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialise the default NVS partition once, formatting it if the layout
 * changed or it has no free pages. Safe to call from several modules; only the
 * first call does the work. */
esp_err_t nvs_store_init(void);

#ifdef __cplusplus
}
#endif