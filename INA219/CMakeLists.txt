add_library(INA219 INTERFACE)

target_include_directories(INA219
    INTERFACE
        .
)

target_sources(INA219
    	INTERFACE
	INA219.cpp
)