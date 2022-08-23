cmake_minimum_required (VERSION 3.5.0 FATAL_ERROR)

set(CXX_STANDARD 17)

set(TARGET AZtest)

project(${TARGET})

set (COMMON_MINI_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../EnvironmentSimulator/Modules/CommonMini")
set (ROADMANAGER_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../EnvironmentSimulator/Modules/RoadManager")

add_subdirectory(${COMMON_MINI_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/build/commonMini)
add_subdirectory(${ROADMANAGER_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/build/roadManager)
set(PUGIXML_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../externals/pugixml")

add_executable(${TARGET} 
  src/CalculateReferenceLine.cpp 
  src/main.cpp
  src/OverlappingReferenceLines.cpp
)
include_directories (
  ${COMMON_MINI_INCLUDE_DIR}
  ${ROADMANAGER_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/inc  
  "${ROADMANAGER_INCLUDE_DIR}/inc"
  ${PUGIXML_INCLUDE_DIR}
)
  target_link_libraries(${TARGET} RoadManager CommonMini pthread)

