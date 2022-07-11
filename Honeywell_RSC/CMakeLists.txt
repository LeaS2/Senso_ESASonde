add_library(Honeywell_RSC INTERFACE)

target_include_directories(Honeywell_RSC
    INTERFACE
        .
)

target_sources(Honeywell_RSC
    	INTERFACE
	Honeywell_RSC.cpp
)