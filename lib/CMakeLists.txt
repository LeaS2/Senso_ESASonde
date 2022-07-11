add_library(mbed-sensorboard INTERFACE)

target_include_directories(mbed-sensorboard 
    INTERFACE
        .
)

target_sources(mbed-sensorboard
    	INTERFACE
	stm32f7_eth_init.c
	system_clock.c
	PeripheralPins.c
)