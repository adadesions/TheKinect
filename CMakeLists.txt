cmake_minimum_required(VERSION 3.1)
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
# SET(OpenCV_DIR /use/local/lib/cmake/opencv4)
SET(OpenCV_DIR ./opencv)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

if(WIN32 AND NOT MINGW)
  if(NOT DEFINED CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX "d")
  endif()
endif()

IF(NOT DEFINED CMAKE_BUILD_TYPE)
  # No effect for multi-configuration generators (e.g. for Visual Studio)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose: RelWithDebInfo Release Debug MinSizeRel None")
ENDIF()

PROJECT(TheKinect)

# SET(MY_DIR ${libfreenect2_examples_SOURCE_DIR})
SET(DEPENDS_DIR "./libfreenect2/depends" CACHE STRING "Dependency directory")

OPTION(ENABLE_OPENGL "Enable OpenGL support" ON)

# The example build system is standalone and will work out-of-tree with these files copied
SET(freenect2_ROOT_DIR ./libfreenect2)
SET(flextGL_SOURCES ${freenect2_ROOT_DIR}/src/flextGL.cpp)
SET(flextGL_INCLUDE_DIRS ${freenect2_ROOT_DIR}/src) # for flextGL.h

FIND_PACKAGE(PkgConfig)    # try find PKGConfig as it will be used if found
LIST(APPEND CMAKE_MODULE_PATH ${freenect2_ROOT_DIR}/cmake_modules) # FindGLFW3.cmake

IF(TARGET freenect2)
  MESSAGE(STATUS "Using in-tree freenect2 target")
  SET(freenect2_LIBRARIES freenect2)
  SET(freenect2_DLLS ${LIBFREENECT2_DLLS})
ELSE()
  FIND_PACKAGE(freenect2 REQUIRED)
  # Out-of-tree build will have to have DLLs manually copied.
ENDIF()

INCLUDE_DIRECTORIES(
  ${freenect2_INCLUDE_DIR}
)

SET(TheKinect_src
  TheKinect.cpp
)

SET(TheKinect_LIBRARIES
  ${freenect2_LIBRARIES}
)

SET(TheKinect_DLLS
  ${freenect2_DLLS}
)

IF(ENABLE_OPENGL)
  FIND_PACKAGE(GLFW3)
  FIND_PACKAGE(OpenGL)
  IF(GLFW3_FOUND AND OPENGL_FOUND)
    INCLUDE_DIRECTORIES(
      ${GLFW3_INCLUDE_DIRS}
      ${flextGL_INCLUDE_DIRS}
    )

    LIST(APPEND TheKinect_DLLS ${GLFW3_DLL})
    LIST(APPEND TheKinect_src
      viewer.cpp
      ${flextGL_SOURCES}
    )
    LIST(APPEND TheKinect_LIBRARIES
      ${GLFW3_LIBRARIES}
      ${OPENGL_gl_LIBRARY}
    )
    ADD_DEFINITIONS(-DEXAMPLES_WITH_OPENGL_SUPPORT=1)
  ENDIF()
ENDIF(ENABLE_OPENGL)

ADD_EXECUTABLE(TheKinect
  ${TheKinect_src}
)

TARGET_LINK_LIBRARIES(TheKinect
  ${TheKinect_LIBRARIES}
  ${OpenCV_LIBS}
)

IF(WIN32)
  INSTALL(TARGETS TheKinect DESTINATION bin)
  LIST(REMOVE_DUPLICATES TheKinect_DLLS)
  FOREACH(FILEI ${TheKinect_DLLS})
    ADD_CUSTOM_COMMAND(TARGET TheKinect POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FILEI} $<TARGET_FILE_DIR:TheKinect>
    )
  ENDFOREACH(FILEI)
  INSTALL(FILES ${TheKinect_DLLS} DESTINATION bin)
ENDIF()
