CMAKE_MINIMUM_REQUIRED (VERSION 2.4)

PROJECT( Filtros )

SET ( CMAKE_DEBUG_POSTFIX d )

INCLUDE_DIRECTORIES(GnecFilter/	
					eigen/	
)

SET (SRC GnecFilter/Filtros.h 
		 GnecFilter/Filtros.cpp
		 GnecFilter/GnecFilters.h
		 GnecFilter/GnecFilters.cpp 
		 GnecFilter/GazeFilters.h
		 GnecFilter/GazeFilters.cpp 
		 GnecFilter/hf_01.h
		 GnecFilter/hf_01.cpp 
		 GnecFilter/hf_02.h
		 GnecFilter/hf_02.cpp 
		 GnecFilter/hf_03.h
		 GnecFilter/hf_03.cpp 
		 GnecFilter/PointD.h
		 GnecFilter/PointD.cpp 
	)

ADD_LIBRARY( ${PROJECT_NAME} SHARED ${SRC} )

SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS FILTERS_EXPORT )

