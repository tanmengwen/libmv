PROJECT(testTS)


#########################
# Qt
#########################
FIND_PACKAGE(Qt4)
ADD_DEFINITIONS( ${QT_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${QT_INCLUDES} )
LINK_DIRECTORIES( ${QT_LIBRARY_DIR} )
#########################



#########################
# vxl
#########################
FIND_PACKAGE(VXL)
IF(VXL_FOUND)
  INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)
ELSE(VXL_FOUND)
  MESSAGE("VXL_DIR should be set to the VXL build directory.")
ENDIF(VXL_FOUND)
#########################


INCLUDE_DIRECTORIES(
	${VXL_OXL_INCLUDE_DIR}	
	..
#	${testTS_SOURCE_DIR}/..
)


SET( GUI_SOURCES TSWidget.cc )

QT_WRAP_CPP(testTS GUI_SOURCES
              TSWidget.h)

ADD_EXECUTABLE( testTrackedSequence
	${GUI_SOURCES}
	main.cc
	TSWidget.h

	../Frame.cc           
	../PointFeature.cc   
	../SiftFeature.cc     
	../PointMatch.cc      
	../PointTrack.cc      
	../TwoViewMatches.cc
	../TrackedSequence.cc 

	../Frame.h             
	../PointFeature.h      
	../SiftFeature.h       
	../PointMatch.h        
	../PointTrack.h        
	../TwoViewMatches.h
	../TrackedSequence.h
)

TARGET_LINK_LIBRARIES( testTrackedSequence
	mvl
	vgl vgl_algo
	vnl vnl_algo v3p_netlib
	vul vbl vcl
	${QT_LIBRARIES}
	${QT_QTCORE_LIBRARY}
	${QT_QTGUI_LIBRARY}
	${QT_QTOPENGL_LIBRARY}
	GL GLU
)
