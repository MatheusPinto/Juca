idf_component_register(SRCS "main.c" "test_tasks/ir_line_task" "test_tasks/threeeyes_task.c" "test_tasks/imu_task.c" "test_tasks/wheel_task.c" "test_tasks/diff_drive_task.c"
                    INCLUDE_DIRS "." "./test_tasks"
                    REQUIRES bdc_motor driver esp_adc esp_timer wheel diff_drive threeeyes mpu6050)
