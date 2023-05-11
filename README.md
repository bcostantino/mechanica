# Mechanica


Control firmware for robotic arm



### ESP IDF Modifications*

To fit the project structure, the env variable `COMPONENT_DIRS` in `$IDF_PATH/make/project.mk` must be modified to the following (see [ESP8266_RTOS_SDK - Build System](https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/api-guides/build-system.html)). 

```make
COMPONENT_DIRS := $(PROJECT_PATH)/src $(PROJECT_PATH)/src/components $(EXTRA_COMPONENT_DIRS) $(IDF_PATH)/components
```
