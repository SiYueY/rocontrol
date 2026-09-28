cmake_minimum_required(VERSION 3.16)
project(rocontrol VERSION 0.1.0 LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)
option(ROCONTROL_BUILD_ROMUJOCO_EXAMPLE "Build the optional RoMuJoCo integration example" OFF)
option(ROCONTROL_BUILD_BENCHMARK "Build the 500 Hz control benchmark" ON)
set(ROCONTROL_INTEGRATION_URDF "" CACHE FILEPATH
    "Optional Mobile FR3 Duo URDF for real-description integration testing")
set(ROCONTROL_FR3_URDF "" CACHE FILEPATH
    "Optional generated Franka FR3 URDF for single-arm integration testing")
set(ROCONTROL_FR3_DUO_URDF "" CACHE FILEPATH
    "Optional generated Franka FR3 Duo URDF for dual-arm integration testing")

add_library(rocontrol
    src/model/robot_model.cpp
    src/task/joint_position_task.cpp
    src/task/cartesian_pose_task.cpp
    src/solver/qp_solver.cpp
    src/controller.cpp
    src/trajectory/joint_position_trajectory.cpp
    src/trajectory/cartesian_pose_trajectory.cpp
    src/fsm/state_machine.cpp
)
add_library(rocontrol::rocontrol ALIAS rocontrol)
target_compile_features(rocontrol PUBLIC cxx_std_17)
target_include_directories(rocontrol PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(rocontrol PUBLIC Eigen3::Eigen pinocchio::pinocchio)
target_compile_definitions(rocontrol PUBLIC EIGEN_RUNTIME_NO_MALLOC)
target_compile_options(rocontrol PRIVATE -Wall -Wextra -Wpedantic)

add_executable(rocontrol_runtime_example examples/runtime_contract.cpp)
target_link_libraries(rocontrol_runtime_example PRIVATE rocontrol::rocontrol)
target_compile_definitions(rocontrol_runtime_example PRIVATE
    TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/simple.urdf")

install(TARGETS rocontrol EXPORT rocontrolTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/rocontrol DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT rocontrolTargets NAMESPACE rocontrol::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rocontrol)
configure_package_config_file(cmake/rocontrolConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/rocontrolConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rocontrol)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/rocontrolConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/rocontrolConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/rocontrolConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rocontrol)

if(BUILD_TESTING)
    add_test(NAME rocontrol_runtime_example COMMAND rocontrol_runtime_example)
    add_executable(rocontrol_model_test tests/model_test.cpp)
    target_link_libraries(rocontrol_model_test PRIVATE rocontrol::rocontrol)
    target_compile_definitions(rocontrol_model_test PRIVATE TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/simple.urdf")
    add_test(NAME rocontrol_model_test COMMAND rocontrol_model_test)
    add_executable(rocontrol_controller_test tests/controller_test.cpp)
    target_link_libraries(rocontrol_controller_test PRIVATE rocontrol::rocontrol)
    target_compile_definitions(rocontrol_controller_test PRIVATE TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/simple.urdf")
    add_test(NAME rocontrol_controller_test COMMAND rocontrol_controller_test)
    add_executable(rocontrol_cartesian_test tests/cartesian_test.cpp)
    target_link_libraries(rocontrol_cartesian_test PRIVATE rocontrol::rocontrol)
    target_compile_definitions(rocontrol_cartesian_test PRIVATE TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/simple.urdf")
    add_test(NAME rocontrol_cartesian_test COMMAND rocontrol_cartesian_test)
    add_executable(rocontrol_behavior_test tests/behavior_test.cpp)
    target_link_libraries(rocontrol_behavior_test PRIVATE rocontrol::rocontrol)
    target_compile_definitions(rocontrol_behavior_test PRIVATE TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/simple.urdf")
    add_test(NAME rocontrol_behavior_test COMMAND rocontrol_behavior_test)
    add_executable(rocontrol_dual_arm_test tests/dual_arm_test.cpp)
    target_link_libraries(rocontrol_dual_arm_test PRIVATE rocontrol::rocontrol)
    target_compile_definitions(rocontrol_dual_arm_test PRIVATE TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/dual_arm_spine.urdf")
    add_test(NAME rocontrol_dual_arm_test COMMAND rocontrol_dual_arm_test)
    add_executable(rocontrol_fsm_trajectory_test tests/fsm_trajectory_test.cpp)
    target_link_libraries(rocontrol_fsm_trajectory_test PRIVATE rocontrol::rocontrol)
    target_compile_definitions(rocontrol_fsm_trajectory_test PRIVATE TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/simple.urdf")
    add_test(NAME rocontrol_fsm_trajectory_test COMMAND rocontrol_fsm_trajectory_test)
    add_executable(rocontrol_fsm_failure_test tests/fsm_failure_test.cpp)
    target_link_libraries(rocontrol_fsm_failure_test PRIVATE rocontrol::rocontrol)
    target_compile_definitions(rocontrol_fsm_failure_test PRIVATE
        TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/simple.urdf")
    add_test(NAME rocontrol_fsm_failure_test COMMAND rocontrol_fsm_failure_test)
    if(ROCONTROL_INTEGRATION_URDF)
        if(NOT EXISTS "${ROCONTROL_INTEGRATION_URDF}")
            message(FATAL_ERROR "ROCONTROL_INTEGRATION_URDF does not exist: ${ROCONTROL_INTEGRATION_URDF}")
        endif()
        add_executable(rocontrol_real_description_test tests/real_description_test.cpp)
        target_link_libraries(rocontrol_real_description_test PRIVATE rocontrol::rocontrol)
        target_compile_definitions(rocontrol_real_description_test PRIVATE
            TEST_URDF_PATH="${ROCONTROL_INTEGRATION_URDF}")
        add_test(NAME rocontrol_real_description_test COMMAND rocontrol_real_description_test)
    endif()
    if(ROCONTROL_FR3_URDF)
        if(NOT EXISTS "${ROCONTROL_FR3_URDF}")
            message(FATAL_ERROR "ROCONTROL_FR3_URDF does not exist: ${ROCONTROL_FR3_URDF}")
        endif()
        add_executable(rocontrol_fr3_description_test tests/fr3_description_test.cpp)
        target_link_libraries(rocontrol_fr3_description_test PRIVATE rocontrol::rocontrol)
        target_compile_definitions(rocontrol_fr3_description_test PRIVATE
            TEST_URDF_PATH="${ROCONTROL_FR3_URDF}")
        add_test(NAME rocontrol_fr3_description_test COMMAND rocontrol_fr3_description_test)
    endif()
    if(ROCONTROL_FR3_DUO_URDF)
        if(NOT EXISTS "${ROCONTROL_FR3_DUO_URDF}")
            message(FATAL_ERROR "ROCONTROL_FR3_DUO_URDF does not exist: ${ROCONTROL_FR3_DUO_URDF}")
        endif()
        add_executable(rocontrol_fr3_duo_description_test tests/real_description_test.cpp)
        target_link_libraries(rocontrol_fr3_duo_description_test PRIVATE rocontrol::rocontrol)
        target_compile_definitions(rocontrol_fr3_duo_description_test PRIVATE TEST_DUO_ONLY
            TEST_URDF_PATH="${ROCONTROL_FR3_DUO_URDF}")
        add_test(NAME rocontrol_fr3_duo_description_test COMMAND rocontrol_fr3_duo_description_test)
    endif()
endif()

if(ROCONTROL_BUILD_BENCHMARK)
    add_executable(rocontrol_benchmark benchmarks/control_benchmark.cpp)
    target_link_libraries(rocontrol_benchmark PRIVATE rocontrol::rocontrol)
    target_compile_definitions(rocontrol_benchmark PRIVATE
        TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/dual_arm_spine.urdf")
endif()

if(ROCONTROL_BUILD_ROMUJOCO_EXAMPLE)
    find_package(romujoco CONFIG REQUIRED)
    add_executable(rocontrol_romujoco_example examples/romujoco_closed_loop.cpp)
    target_link_libraries(rocontrol_romujoco_example PRIVATE rocontrol::rocontrol romujoco::romujoco)
    target_compile_definitions(rocontrol_romujoco_example PRIVATE
        TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/single_joint.urdf"
        TEST_MJCF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/single_joint.xml")
    if(BUILD_TESTING)
        add_test(NAME rocontrol_romujoco_example COMMAND rocontrol_romujoco_example)
    endif()
    add_executable(rocontrol_romujoco_cartesian_example examples/romujoco_cartesian.cpp)
    target_link_libraries(rocontrol_romujoco_cartesian_example PRIVATE
        rocontrol::rocontrol romujoco::romujoco)
    target_compile_definitions(rocontrol_romujoco_cartesian_example PRIVATE
        TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/cartesian_arm.urdf"
        TEST_MJCF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/cartesian_arm.xml")
    if(BUILD_TESTING)
        add_test(NAME rocontrol_romujoco_cartesian_example
            COMMAND rocontrol_romujoco_cartesian_example)
    endif()
endif()
