cmake_minimum_required(VERSION 3.10)

include(CheckSymbolExists)

project(lsl3dlib VERSION 1.0)


option(LSL_SIMD_SSE "Enable SSE4-based versions of CCL algorithms")
option(LSL_SIMD_AVX2 "Enable AVX2-based versions of CCL algorithms")
option(LSL_SIMD_AVX512 "Enable AVX512-based versions of CCL algorithms")
option(LSL_SIMD_NEON "Enable Neon-based versions of CCL algorithms")


list(APPEND lsl3d_target_list lsl3d-obj)
list(APPEND lsl3d_target_list lsl3d-slib)



macro(lsl3d_target_compile_definitions targets privacy dir)
  foreach(_target IN ITEMS ${targets})
    message("${_target} ${privacy} ${dir}")
    target_compile_definitions(${_target} ${privacy} ${dir})
  endforeach()
endmacro()



set(src_dir src)
set(inc_dir include)
set(lib_dir lib)

set(lsl3d_files
  ${src_dir}/papi_helper.cpp
  ${src_dir}/perf-helper.cpp
  ${src_dir}/utility.cpp
  ${src_dir}/timer.cpp
  ${src_dir}/rle/rle-sse.cpp
  ${src_dir}/rle/compress_lut.cpp
  ${src_dir}/lsl3d/relabeling.cpp
  ${src_dir}/lsl3d/unification_stats.cpp
  )


  
set(LIBRARY_OUTPUT_PATH ${lib_dir})

add_library(lsl3d-obj OBJECT ${lsl3d_files})
set_target_properties(lsl3d-obj PROPERTIES POSITION_INDEPENDENT_CODE ON) # set -fpic

add_library(lsl3d-slib STATIC $<TARGET_OBJECTS:lsl3d-obj>)
set_target_properties(lsl3d-slib PROPERTIES OUTPUT_NAME lsl3d POSITION_INDEPENDENT_CODE ON) # set -fpic

#list(APPEND lsl_none_target_list lsl3d-obj lsl3d-slib)
#list(APPEND lsl_target_list lsl3d-obj lsl3d-slib)
target_include_directories(lsl3d-obj PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${inc_dir}/>)
target_include_directories(lsl3d-slib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${inc_dir}/>)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/simdhelpers)

if(LSL3DLIB_USE_PERF)
  message("LSL3D: Enabling perf-utils")
  lsl3d_target_compile_definitions("${lsl3d_target_list}" PUBLIC LSL3DLIB_USE_PERF=1)

  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/perf-utils)
  
  target_link_libraries(lsl3d-obj PUBLIC perf-utils)
  target_link_libraries(lsl3d-slib PUBLIC perf-utils)
else()
  message("LSL3D: Disabling perf-utils")
  lsl3d_target_compile_definitions("${lsl3d_target_list}" PUBLIC LSL3DLIB_USE_PERF=0)
  
endif()


if (LSL_SIMD_SSE)
  lsl3d_target_compile_definitions("${lsl3d_target_list}" PUBLIC LSL_SIMD_SSE=1)
endif()

if (LSL_SIMD_AVX2)
  lsl3d_target_compile_definitions("${lsl3d_target_list}" PUBLIC LSL_SIMD_AVX2=1)
endif()

if (LSL_SIMD_AVX512)
  lsl3d_target_compile_definitions("${lsl3d_target_list}" PUBLIC LSL_SIMD_AVX512=1)
endif()

if (LSL_SIMD_NEON)
  lsl3d_target_compile_definitions("${lsl3d_target_list}" PUBLIC LSL_SIMD_NEON=1)
endif()


target_link_libraries(lsl3d-obj PUBLIC simdhelpers-slib)
target_link_libraries(lsl3d-slib PUBLIC simdhelpers-slib)

export(TARGETS lsl3d-slib NAMESPACE lsl3d:: FILE "${lib_dir}/cmake/lsl3d/${target-scalar-name}-config.cmake")
