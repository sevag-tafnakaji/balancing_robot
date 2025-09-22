# idf_component_register(SRCS "main.c mpu6050.c webpage.c"
#                        INCLUDE_DIRS ". include")

set(COMPONENT_SRCS "main.c mpu6050.c tb6612fng.c")
set(COMPONENT_INCLUDES ". include")


register_component()
