#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

#ifndef CONFIG_LOOP_STACK_SIZE
#define CONFIG_LOOP_STACK_SIZE 2048
#endif

TaskHandle_t loopTaskHandle = NULL;

extern void setup(void);
extern void loop(void);

bool loopTaskWDTEnabled;

void loopTask(void *pvParameters)
{
    setup();
    for(;;) {
        if(loopTaskWDTEnabled){
            esp_task_wdt_reset();
        }
        loop();
//        if (serialEventRun) serialEventRun();
    }
}

void app_main()
{
    loopTaskWDTEnabled = false;
    xTaskCreatePinnedToCore(loopTask, "loopTask", CONFIG_LOOP_STACK_SIZE, NULL, 1, &loopTaskHandle, 1);
}
