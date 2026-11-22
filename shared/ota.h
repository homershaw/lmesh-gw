#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_http_server.h"

/* Call once after start-up. If the running firmware is a fresh update on
 * trial, ok_fn is polled every 5 s; the first true confirms the firmware.
 * No confirmation within timeout_s restarts into the previous firmware. */
void ota_watch(bool (*ok_fn)(void), int timeout_s);
bool ota_pending(void);              /* running firmware is still on trial */
int  ota_trial_left_s(void);
void ota_confirm(void);              /* keep the running firmware (manual override) */
const char *ota_slot(void);          /* label of the running app partition */

/* streamed update into the idle slot */
esp_err_t ota_begin(void);
esp_err_t ota_write(const uint8_t *data, size_t len);   /* first call needs >= 512 bytes */
esp_err_t ota_finish(void);          /* verify + select for next boot; caller restarts */
void ota_abort(void);
const char *ota_error(void);

/* HTTP handler body: POST with the .bin as the request body */
esp_err_t ota_http_upload(httpd_req_t *r);
