idf_component_register(SRCS "main.c" "example_tasks/ir_line_task" "example_tasks/threeeyes_task.c" "example_tasks/imu_task.c" "example_tasks/wheel_task.c" "example_tasks/diff_drive_task.c"
                    INCLUDE_DIRS "." "./example_tasks"
                    REQUIRES bdc_motor driver esp_adc esp_timer wheel diff_drive threeeyes mpu6050)
