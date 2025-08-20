idf_component_register(SRCS "nvs.c" "bq25620.c" "timergate_pole.c"
                    INCLUDE_DIRS ".")
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/led_strip)