#########################
# Qt
#########################
FIND_PACKAGE(Qt4)
ADD_DEFINITIONS( ${QT_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${QT_INCLUDES} )
LINK_DIRECTORIES( ${QT_LIBRARY_DIR} )
#########################


ADD_EXECUTABLE( MultiImageViewer_example
	MultiImageViewer_example.cc
)

TARGET_LINK_LIBRARIES( MultiImageViewer_example
	MVui
	${QT_LIBRARIES}
	${QT_QTCORE_LIBRARY}
	${QT_QTGUI_LIBRARY}
	${QT_QTOPENGL_LIBRARY}
	${OPENGL_LIBRARIES}
)
