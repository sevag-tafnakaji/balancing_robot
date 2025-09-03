# idf_component_register(SRCS "main.c mpu6050.c webpage.c"
#                        INCLUDE_DIRS ". include")

set(COMPONENT_SRCS "main.c mpu6050.c webpage.c")
set(COMPONENT_INCLUDES ". include")
# Embed the server root certificate into the final binary
#
# (If this was a component, we would set COMPONENT_EMBED_TXTFILES here.)
set(COMPONENT_EMBED_TXTFILES "resources/server.pem" "resources/server.key" "resources/ca.pem")

register_component()
