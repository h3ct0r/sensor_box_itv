find_path(O3D3XX_CAMERA_INCLUDE_DIRS
  NAMES o3d3xx_camera.h
  PATHS /opt/libo3d3xx/include /usr/include /usr/local/include /usr/local/libo3d3xx/include
  NO_DEFAULT_PATH
  DOC "o3d3xx_camera Include directory"
  )

find_library(O3D3XX_CAMERA_LIBRARIES
  NAMES o3d3xx_camera libo3d3xx_camera_static.a
  PATHS /opt/libo3d3xx/lib /usr/lib /usr/local/lib /usr/local/libo3d3xx/lib
  NO_DEFAULT_PATH
  DOC "o3d3xx_camera shared object file"
  )

get_filename_component(
  O3D3XX_CAMERA_LIBRARY_DIR
  ${O3D3XX_CAMERA_LIBRARIES}
  DIRECTORY
  )

# kind of hacky
include(
 "${O3D3XX_CAMERA_LIBRARY_DIR}/o3d3xx_camera/o3d3xx_camera-config-version.cmake"
 )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(O3D3XX_CAMERA
  FOUND_VAR O3D3XX_CAMERA_FOUND
  REQUIRED_VARS O3D3XX_CAMERA_LIBRARIES
                O3D3XX_CAMERA_INCLUDE_DIRS
  VERSION_VAR O3D3XX_CAMERA_VERSION
  )

set(_version_major ${o3d3xx_camera_FIND_VERSION_MAJOR})
set(_version_minor ${o3d3xx_camera_FIND_VERSION_MINOR})
set(_version_patch ${o3d3xx_camera_FIND_VERSION_PATCH})
set(O3D3XX_CAMERA_FIND_VERSION
    "${_version_major}.${_version_minor}.${_version_patch}")

if("${O3D3XX_CAMERA_VERSION}" VERSION_LESS "${O3D3XX_CAMERA_FIND_VERSION}")
  set(O3D3XX_CAMERA_VERSION_COMPATIBLE FALSE)
  set(O3D3XX_CAMERA_VERSION_EXACT FALSE)
else()
  set(O3D3XX_CAMERA_VERSION_COMPATIBLE TRUE)
  if ("${O3D3XX_CAMERA_VERSION}" VERSION_EQUAL "${O3D3XX_CAMERA_FIND_VERSION}")
    set(O3D3XX_CAMERA_VERSION_EXACT TRUE)
  else()
    set(O3D3XX_CAMERA_VERSION_EXACT FALSE)
  endif()
endif()

if(${o3d3xx_camera_FIND_VERSION_EXACT})
  if(NOT ${O3D3XX_CAMERA_VERSION_EXACT})
    message(FATAL_ERROR
     "o3d3xx_camera: ${o3d3xx_camera_FIND_VERSION} != ${O3D3XX_CAMERA_VERSION}")
  endif()
endif()

if(NOT ${O3D3XX_CAMERA_VERSION_COMPATIBLE})
  message(FATAL_ERROR
   "o3d3xx_camera: ${O3D3XX_CAMERA_VERSION} is incompatible with ${o3d3xx_camera_FIND_VERSION}")
endif()

mark_as_advanced(
  O3D3XX_CAMERA_LIBRARIES
  O3D3XX_CAMERA_INCLUDE_DIRS
  )

# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()
