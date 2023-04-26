cmake_minimum_required(VERSION 3.5)
project(uned_khepera_description)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

#=============
# Generate N urdf files, each with unique robot and frame names
# Source file is in ${CMAKE_CURRENT_SOURCE_DIR}/urdf/khepera.xml
# Generated files are in ${CMAKE_CURRENT_BINARY_DIR}/urdf/kheperaX.urdf
#=============

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/urdf")

foreach (INDEX RANGE 1 40)
  if (${INDEX} EQUAL 0)
    set(SUFFIX "00")
    set(TOPIC_NS "khepera00")
  elseif (${INDEX} LESS 10)
    set(SUFFIX "0${INDEX}")
    set(TOPIC_NS "khepera0${INDEX}")
  else ()
    set(SUFFIX "${INDEX}")
    set(TOPIC_NS "khepera${INDEX}")
  endif ()
  set(URDF_FILE "${CMAKE_CURRENT_BINARY_DIR}/urdf/khepera${SUFFIX}.urdf")
  message(STATUS "creating rules for ${URDF_FILE}")
  add_custom_command(
    OUTPUT ${URDF_FILE}
    COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/src/replace.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/urdf/khepera.xml" "suffix=${SUFFIX}" "topic_ns=${TOPIC_NS}" ">" "${URDF_FILE}"
    DEPENDS urdf/khepera.xml
    COMMENT "Generate ${URDF_FILE}"
    VERBATIM
  )
  # TODO investigate why configure_file isn't working
  # configure_file("urdf/khepera.xml" ${URDF_FILE})
  add_custom_target(generate_urdf${SUFFIX} ALL DEPENDS ${URDF_FILE})
endforeach ()

install(
  DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}/urdf"
    rviz
    launch
    meshes
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  PROGRAMS
  src/replace.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
