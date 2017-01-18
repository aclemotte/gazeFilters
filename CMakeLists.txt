CMAKE_MINIMUM_REQUIRED (VERSION 2.4)

PROJECT( Filtros )

SET ( CMAKE_DEBUG_POSTFIX d )
#libraries get created as expected :
# - named.dll in Debug and
# - name.dll in others configurations

INCLUDE_DIRECTORIES(GnecFilter/	
					eigen/	
)

SET (SRC GnecFilter/Filtros.cpp
		 GnecFilter/GnecFilters.cpp 
		 GnecFilter/GazeFilters.cpp 
		 GnecFilter/hf_01.cpp 
		 GnecFilter/hf_02.cpp 
		 GnecFilter/hf_03.cpp 
		 GnecFilter/PointD.cpp 
		 GnecFilter/GazeStateClassifier.cpp 
		 GnecFilter/BufferPointD.cpp 
)

SET (HDRS 	GnecFilter/Filtros.h 
			GnecFilter/GnecFilters.h
			GnecFilter/GazeFilters.h
			GnecFilter/hf_01.h
			GnecFilter/hf_02.h
			GnecFilter/hf_03.h
			GnecFilter/PointD.h
			GnecFilter/GazeStateClassifier.h
			GnecFilter/BufferPointD.h
)
		

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SRC} ${HDRS})

SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS FILTERS_EXPORT )

