#include <stdio.h>
#include "esp_log.h"
#include "CANopenNode_ESP32.h"

static const char *TAG = "main";

void app_main(void)
{
#if CONFIG_USE_CANOPENNODE
    CO_ESP32_init();
#endif
}
