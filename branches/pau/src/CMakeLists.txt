# src/CMakeLists.txt

PROJECT( mv )

SUBDIRS( dense opengl sparse ui )


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
# OpenGL
#########################
FIND_PACKAGE(OpenGL)


