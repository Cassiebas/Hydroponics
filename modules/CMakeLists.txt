idf_component_register(SRCS "adc.cpp" "uart.cpp" "state_machine.cpp"
                            "sensors/tds.cpp" "sensors/ntc.cpp" "sensors/ultrasonic.cpp" "sensors/ph.cpp"
                      INCLUDE_DIRS "../includes"
                      REQUIRES driver esp_adc)