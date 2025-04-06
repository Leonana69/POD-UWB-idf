#ifndef __WEB_SERVICE_H__
#define __WEB_SERVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_http_server.h"
httpd_handle_t webServiceInit();

#ifdef __cplusplus
}
#endif

#endif // __WEB_SERVICE_H__