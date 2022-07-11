add_library(MS5611_01BA03 INTERFACE)

target_include_directories(MS5611_01BA03
    INTERFACE
        .
)

target_sources(MS5611_01BA03
    	INTERFACE
	MS5611_01BA03.cpp
)