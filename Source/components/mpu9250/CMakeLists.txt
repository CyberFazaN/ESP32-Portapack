idf_component_register(SRCS         "ak8963.c"
                                    "calibrate.c"
                                    "common.c"
                                    "mpu9250.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver i2cdev esp_idf_lib_helpers)