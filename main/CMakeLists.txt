file(GLOB BOSCH_DRIVER "drivers/Bosch/src/*.c")
file(GLOB DWM1000_DRIVER "drivers/DWM1000/src/*.c")

idf_component_register(
    SRCS "main.cpp"
         "src/kalman_core.cpp"
         "src/estimator.cpp"
         "src/button.c"
         "src/wifi_link.c"
         "src/web_service.c"
         "src/imu.c"
         "src/filter.c"
         "src/loco.c"
         "src/tdoa2.c"
         "src/utils.c"
         ${BOSCH_DRIVER}
         ${DWM1000_DRIVER}
        
    INCLUDE_DIRS "include"
               "drivers/Bosch/include"
               "drivers/DWM1000/include")
