ADD_LIBRARY(descriptor
            daisy_descriptor.cc
            simpliest_descriptor.cc
            surf_descriptor.cc
            dipole_descriptor.cc
            descriptor_factory.cc)

IF (CMAKE_SYSTEM MATCHES "Darwin-10.6.0")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
ENDIF (CMAKE_SYSTEM MATCHES "Darwin-10.6.0")

LIBMV_INSTALL_LIB(descriptor)
LIBMV_TEST(daisy_descriptor "descriptor;image;daisy")
