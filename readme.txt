# Proyecto INTERAAC #

Proyecto de la libreria de los filtros en c++ con la api para introducir datos al PRIMMA

filtros:
8 funciones de filtrado, del 1 al 8
la 9 filtra con el filtro del primma actual
la 0 no filtra

arquitectura:

Eye-tracker  provee los datos de la mirada a través de Filtros.cpp que proporciona GazeX y GazeY
Algun filtro de los 8 posibles retorna los datos GazeX_Filtered, GazeY_Filtered
Los filtros son seleccionados mediante teclado on-line