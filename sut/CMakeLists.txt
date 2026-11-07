cmake_minimum_required(VERSION 3.0.2)
project(bare_metal_mpc C CXX)

# 이 패키지의 core 는 ROS 없이 C99 만 쓴다. catkin 은 워크스페이스에서 함께 빌드하고
# bmpc_bridge 가 라이브러리를 찾게 하는 데만 쓴다.
find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS core/include platform/linux/include
  LIBRARIES bmpc_core bmpc_linux
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(BMPC_C_FLAGS -std=c99 -Wall -Wextra -Wno-unused-parameter -Wdouble-promotion)

# ---- core: MCU 에 그대로 올라가는 코드 ----
add_library(bmpc_core STATIC
  core/src/bmpc_config.c
  core/src/bmpc_protocol.c
  core/src/bmpc_delay.c
  core/src/bmpc_weights.c
  core/src/bmpc_qp.c
  core/src/bmpc_mpc.c
  core/src/bmpc_post.c
  core/src/bmpc_controller.c
  core/src/bmpc_app.c
)
target_include_directories(bmpc_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/core/include)
target_compile_options(bmpc_core PRIVATE ${BMPC_C_FLAGS})
set_target_properties(bmpc_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(bmpc_core m)

# ---- 리눅스 플랫폼: UDP 가상 CAN (가상 MCU 와 PC 브리지가 같이 쓴다) ----
add_library(bmpc_linux STATIC platform/linux/src/bmpc_udp.c)
target_include_directories(bmpc_linux PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/core/include ${CMAKE_CURRENT_SOURCE_DIR}/platform/linux/include)
target_compile_options(bmpc_linux PRIVATE ${BMPC_C_FLAGS})
set_target_properties(bmpc_linux PROPERTIES POSITION_INDEPENDENT_CODE ON)

# SILS 가상 MCU: MCU 펌웨어와 같은 앱 코드 + 리눅스 HAL
add_executable(bmpc_sim_mcu platform/linux/src/sim_mcu_main.c)
target_compile_options(bmpc_sim_mcu PRIVATE ${BMPC_C_FLAGS})
target_link_libraries(bmpc_sim_mcu bmpc_linux bmpc_core m)

install(TARGETS bmpc_core bmpc_linux bmpc_sim_mcu
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY core/include/bmpc platform/linux/include/bmpc DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

# ---- 시험 (devel/lib/bare_metal_mpc 에 생긴다) ----
add_executable(bmpc_test_core test/test_core.c)
target_compile_options(bmpc_test_core PRIVATE -std=c99 -Wall)
target_link_libraries(bmpc_test_core bmpc_core m)

add_executable(bmpc_test_app test/test_app.c)
target_compile_options(bmpc_test_app PRIVATE -std=c99 -Wall)
target_link_libraries(bmpc_test_app bmpc_core m)

# 원본 C++ 와 같은 값을 내는지: 원본 steering_delay.h 의 고정 사본(test/reference)을 include 한다.
# MPC + 안전 제어기 통합 계층 (integ/bmpc_safety + third_party/kuuve_safety)
add_executable(bmpc_test_integ test/test_integ.c integ/bmpc_safety.c third_party/kuuve_safety/safety_ctrl.c)
target_include_directories(bmpc_test_integ PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/integ ${CMAKE_CURRENT_SOURCE_DIR}/third_party/kuuve_safety)
target_compile_options(bmpc_test_integ PRIVATE -std=c99 -Wall)
target_compile_definitions(bmpc_test_integ PRIVATE BS_TEST_HOOK)
target_link_libraries(bmpc_test_integ bmpc_core m)

# 경계·거부 경로 추가 시험
add_executable(bmpc_test_core_more test/test_core_more.c)
target_compile_options(bmpc_test_core_more PRIVATE -std=c99 -Wall)
target_link_libraries(bmpc_test_core_more bmpc_core m)

add_executable(bmpc_test_qp_more test/test_qp_more.c)
target_compile_options(bmpc_test_qp_more PRIVATE -std=c99 -Wall)
target_link_libraries(bmpc_test_qp_more bmpc_core m)

# 같은 시험을 BMPC_PROFILE 로: 시계 대신 카운터를 넣어 기울기·Cholesky 구간 호출 횟수를 정확히 본다. bmpc_core 는 프로파일 없이 빌드되므로 소스를 다시 컴파일한다.
add_executable(bmpc_test_qp_more_prof test/test_qp_more.c
  core/src/bmpc_config.c core/src/bmpc_protocol.c core/src/bmpc_delay.c core/src/bmpc_weights.c core/src/bmpc_qp.c
  core/src/bmpc_mpc.c core/src/bmpc_post.c core/src/bmpc_controller.c core/src/bmpc_app.c)
target_include_directories(bmpc_test_qp_more_prof PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/core/include)
target_compile_options(bmpc_test_qp_more_prof PRIVATE -std=c99 -Wall)
target_compile_definitions(bmpc_test_qp_more_prof PRIVATE BMPC_PROFILE)
target_link_libraries(bmpc_test_qp_more_prof m)

add_executable(bmpc_test_integ_more test/test_integ_more.c integ/bmpc_safety.c third_party/kuuve_safety/safety_ctrl.c)
target_include_directories(bmpc_test_integ_more PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/integ ${CMAKE_CURRENT_SOURCE_DIR}/third_party/kuuve_safety)
target_compile_options(bmpc_test_integ_more PRIVATE -std=c99 -Wall)
target_compile_definitions(bmpc_test_integ_more PRIVATE BS_TEST_HOOK)
target_link_libraries(bmpc_test_integ_more bmpc_core m)

add_executable(bmpc_test_integ_plaus test/test_integ_more.c integ/bmpc_safety.c third_party/kuuve_safety/safety_ctrl.c)
target_include_directories(bmpc_test_integ_plaus PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/integ ${CMAKE_CURRENT_SOURCE_DIR}/third_party/kuuve_safety)
target_compile_options(bmpc_test_integ_plaus PRIVATE -std=c99 -Wall)
target_compile_definitions(bmpc_test_integ_plaus PRIVATE BS_TEST_HOOK BS_PLAUS)
target_link_libraries(bmpc_test_integ_plaus bmpc_core m)

add_executable(bmpc_test_vs_original test/test_vs_original.cpp)
target_compile_options(bmpc_test_vs_original PRIVATE -std=c++11 -Wall)
target_include_directories(bmpc_test_vs_original PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
target_link_libraries(bmpc_test_vs_original bmpc_core m)

# 원본 NMPC(CppAD + IPOPT)와 비교: 대회 워크스페이스의 IPOPT 빌드가 있을 때만 만든다.
# 이 패키지는 저장소 폴더(src/bare-metal-mpc/) 아래에 있으므로 워크스페이스 src 는 두 단계 위다.
set(BMPC_WS_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../.. CACHE PATH "CppAD, Ipopt_pkg 가 있는 워크스페이스 src")
set(BMPC_CPPAD_ROOT ${BMPC_WS_SRC}/CppAD)
set(BMPC_IPOPT_LIBDIR ${BMPC_WS_SRC}/Ipopt_pkg/Ipopt/build/src/.libs)
set(BMPC_HSL_LIBDIR ${BMPC_WS_SRC}/Ipopt_pkg/ThirdParty-HSL/.libs)
set(BMPC_MUMPS_LIBDIR ${BMPC_WS_SRC}/Ipopt_pkg/ThirdParty-Mumps/.libs)
set(BMPC_CPPAD_LIBDIR ${BMPC_CPPAD_ROOT}/build/cppad_lib)
find_library(BMPC_IPOPT_LIB NAMES ipopt PATHS ${BMPC_IPOPT_LIBDIR} /usr/local/lib NO_DEFAULT_PATH)
find_library(BMPC_CPPAD_LIB NAMES cppad_lib PATHS ${BMPC_CPPAD_LIBDIR} /usr/local/lib NO_DEFAULT_PATH)
find_library(BMPC_HSL_LIB NAMES coinhsl PATHS ${BMPC_HSL_LIBDIR} /usr/local/lib NO_DEFAULT_PATH)
find_library(BMPC_MUMPS_LIB NAMES coinmumps PATHS ${BMPC_MUMPS_LIBDIR} /usr/local/lib NO_DEFAULT_PATH)
find_library(BMPC_METIS_LIB NAMES metis libmetis.so.5 PATHS /usr/lib/x86_64-linux-gnu /usr/local/lib)
if(BMPC_IPOPT_LIB AND BMPC_CPPAD_LIB AND EXISTS ${BMPC_CPPAD_ROOT}/include/cppad/ipopt/solve.hpp)
  # CppAD 헤더가 Eigen 을 include 한다
  find_package(Eigen3 QUIET)
  if(NOT EIGEN3_INCLUDE_DIR)
    set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
  endif()
  foreach(t bmpc_test_vs_ipopt bmpc_test_track_vs_ipopt)
    string(REPLACE "bmpc_" "" src ${t})
    add_executable(${t} test/${src}.cpp)
    target_compile_options(${t} PRIVATE -std=c++11 -Wall -Wno-unused-variable)
    target_include_directories(${t} PRIVATE ${BMPC_CPPAD_ROOT}/include ${EIGEN3_INCLUDE_DIR} /usr/local/include)
    target_link_libraries(${t} bmpc_core ${BMPC_CPPAD_LIB} ${BMPC_IPOPT_LIB})
    if(BMPC_HSL_LIB)
      target_link_libraries(${t} ${BMPC_HSL_LIB})
    endif()
    if(BMPC_MUMPS_LIB)
      target_link_libraries(${t} ${BMPC_MUMPS_LIB})
    endif()
    if(BMPC_METIS_LIB)
      target_link_libraries(${t} ${BMPC_METIS_LIB})
    endif()
    target_link_libraries(${t} lapack blas gfortran quadmath pthread m)
    set_target_properties(${t} PROPERTIES LINK_FLAGS
      "-Wl,--disable-new-dtags -Wl,-rpath,${BMPC_MPC_LIBDIR}:${BMPC_IPOPT_LIBDIR}:${BMPC_HSL_LIBDIR}:${BMPC_MUMPS_LIBDIR}:${BMPC_CPPAD_LIBDIR} -Wl,-rpath-link,${BMPC_MPC_LIBDIR}:${BMPC_IPOPT_LIBDIR}:${BMPC_HSL_LIBDIR}:${BMPC_MUMPS_LIBDIR}:${BMPC_CPPAD_LIBDIR}")
  endforeach()
else()
  message(STATUS "bare_metal_mpc: IPOPT/CppAD not found, skipping bmpc_test_vs_ipopt")
endif()
