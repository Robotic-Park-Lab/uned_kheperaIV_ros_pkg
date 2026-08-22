cmake_minimum_required(VERSION 3.8)
project(uned_kheperaiv_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

#=============
# Generate N urdf files, each with unique robot and frame names, from
# urdf/khepera.xml (merged in from the former uned_khepera_description
# package -- it wasn't worth keeping as a standalone package, same call as
# the uned_crazyflie_common -> uned_crazyflie_driver absorption in the
# Crazyflie repo).
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
  add_custom_command(
    OUTPUT ${URDF_FILE}
    COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/src/replace.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/urdf/khepera.xml" "suffix=${SUFFIX}" "topic_ns=${TOPIC_NS}" ">" "${URDF_FILE}"
    DEPENDS urdf/khepera.xml
    COMMENT "Generate ${URDF_FILE}"
    VERBATIM
  )
  add_custom_target(generate_urdf${SUFFIX} ALL DEPENDS ${URDF_FILE})
endforeach ()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

install(
  DIRECTORY
    launch
    model
    worlds
    resources
    rviz
    rqt
    urdf
    meshes
    "${CMAKE_CURRENT_BINARY_DIR}/urdf"
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  src/replace.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
