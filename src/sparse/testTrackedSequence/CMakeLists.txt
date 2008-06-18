PROJECT(testTS)


#########################
# Qt
#########################
FIND_PACKAGE(Qt4)
ADD_DEFINITIONS( ${QT_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${QT_INCLUDES} )
LINK_DIRECTORIES( ${QT_LIBRARY_DIR} )
#########################


INCLUDE_DIRECTORIES(
	${VXL_OXL_INCLUDE_DIR}
	..
	../..
)


SET( GUI_SOURCES TSWidget.cc )

QT_WRAP_CPP(testTS GUI_SOURCES TSWidget.h)

ADD_EXECUTABLE( testTrackedSequence
	${GUI_SOURCES}
	main.cc
)

TARGET_LINK_LIBRARIES( testTrackedSequence
	MVsparse MVui
	mvl
	vgl vgl_algo
	vnl vnl_algo v3p_netlib
	vul vbl vcl
	${QT_LIBRARIES}
	${QT_QTCORE_LIBRARY}
	${QT_QTGUI_LIBRARY}
	${QT_QTOPENGL_LIBRARY}
	${OPENGL_LIBRARIES}
)
