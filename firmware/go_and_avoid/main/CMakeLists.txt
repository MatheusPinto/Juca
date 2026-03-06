idf_component_register(SRCS "wheel.c" "main.c"  "mpu6050.c" "treeeyes.c" "test_tasks/treeeyes_task.c" "test_tasks/imu_task.c" "test_tasks/wheel_task.c"
                    INCLUDE_DIRS "." "./test_tasks"
                    REQUIRES bdc_motor driver esp_adc esp_timer)
