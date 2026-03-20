#include "threeeyes_task.h"

const static char *TAG = "three_eyes";

portTASK_FUNCTION(Threeeyes, args)
{
    ThreeEyes_Init();
	//ThreeEyes_DisableLeft();
    //ThreeEyes_DisableRight();
    ultrasonic_value_t sensor[3];
    char *near_sensor_name;
    char *sensor_name[] = {"left", "middle", "right"};
	while(1)
	{
        ThreeEyes_TrigAndWait(portMAX_DELAY);
        ThreeEyes_Read(&sensor[0], &sensor[1], &sensor[2]);
        
        uint32_t min_ticks = 0xFFFFFFFF; 
        near_sensor_name = "none";

        for ( int i = 0; i < 3; i++ )
        {
            if (sensor[i].isUpdated == pdTRUE && sensor[i].tof_ticks < min_ticks) 
            {
                min_ticks = sensor[i].tof_ticks;
                near_sensor_name = sensor_name[i];
            }
        }

        float distance = (min_ticks * (1000000.0 / esp_clk_apb_freq())) / 58.0;
        ESP_LOGI(TAG, "The sensor with the nearest detected object was: %s (Distance: %.2f cm)", near_sensor_name, distance);
        //printf("The sensor with the nearest detected object was: %s (Distance: %"PRIu32" ticks)\n", near_sensor_name, min_ticks);
    
        vTaskDelay(pdMS_TO_TICKS(500));
    }
	
}