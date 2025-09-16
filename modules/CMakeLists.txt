idf_component_register(SRCS "adc/adc.cpp" "uart/uart.cpp" "state_machine/state_machine.cpp"
                            "sensors/tds.cpp" "sensors/ntc.cpp" "sensors/ultrasonic.cpp" "sensors/ph.cpp"
                      INCLUDE_DIRS "../include"
                      REQUIRES driver esp_adc)