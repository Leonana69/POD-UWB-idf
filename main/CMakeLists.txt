file(GLOB BOSCH_DRIVER "drivers/Bosch/Src/*.c")

idf_component_register(
    SRCS "main.c"
         "src/button.c"
         "src/wifi_link.c"
         "src/imu.c"
         "src/filter.c"
         ${BOSCH_DRIVER}
        
    INCLUDE_DIRS "include"
               "drivers/Bosch/Inc")
