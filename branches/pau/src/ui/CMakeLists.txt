ADD_SUBDIRECTORY( examples )



#########################
# Qt
FIND_PACKAGE(Qt4)
ADD_DEFINITIONS( ${QT_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${QT_INCLUDES} )
LINK_DIRECTORIES( ${QT_LIBRARY_DIR} )
#########################




SET( GUI_SOURCES MultiImageViewer.cc multi_view_stereo_viewer.cc)

QT_WRAP_CPP(mv GUI_SOURCES
              MultiImageViewer.h
              multi_view_stereo_viewer.h)

ADD_LIBRARY( MVui
	${GUI_SOURCES}
)

