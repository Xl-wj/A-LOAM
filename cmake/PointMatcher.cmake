# Pointmatcher
# find_package(libpointmatcher REQUIRED)
set(GEARS_ARCH /home/xl/gears/x86_64)
set(libpointmatcher_INCLUDE_DIRS "${GEARS_ARCH}/include")
set(GEARS_LIBRARY_DIR ${GEARS_ARCH}/lib CACHE PATH "GEARS_LIBRARY_DIR")

file(GLOB LIBRARIES "${GEARS_LIBRARY_DIR}/libpointmatcher.so")
# message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")
set(libpointmatcher_LIBRARIES ${LIBRARIES} CACHE INTERNAL "libpointmatcher_LIBRARIES")

message(STATUS "=======libpointmatcher_INCLUDE_DIRS: ${libpointmatcher_INCLUDE_DIRS}")
include_directories(${libpointmatcher_INCLUDE_DIRS})

message(STATUS "=======libpointmatcher_INCLUDE_DIRS: ${libpointmatcher_INCLUDE_DIRS}")
message(STATUS "=======libpointmatcher_LIBRARIES: ${libpointmatcher_LIBRARIES}")
