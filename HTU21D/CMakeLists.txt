add_library(HTU21D INTERFACE)

target_include_directories(HTU21D
    INTERFACE
        .
)

target_sources(HTU21D
    	INTERFACE
	HTU21D.cpp
)