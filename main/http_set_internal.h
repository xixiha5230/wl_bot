#pragma once

/*
 * Private interface between the /api/set translation units.
 *
 * http_set.c owns the request loop and the "applied" note buffer; the key
 * handlers are grouped by intent so each file only needs the headers for what
 * it touches: http_set_tune.c for balance/yaw/filter tuning, http_set_motion.c
 * for height, legs, jumps and get-up.
 */

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *key;
    void (*apply)(const char *query, const char *value, char *applied, size_t cap);
} http_set_entry_t;

const http_set_entry_t *http_set_tune_entries(size_t *count);
const http_set_entry_t *http_set_motion_entries(size_t *count);

/* Append "key=value " to the applied note buffer (truncates at cap). */
void http_set_note(char *applied, size_t cap, const char *fmt, ...);

#ifdef __cplusplus
}
#endif
