cmake_minimum_required(VERSION 3.16)
project(c5div C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wno-unused-parameter)
add_compile_definitions(_GNU_SOURCE)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
  # Pi Zero 2W / Pi 3: Cortex-A53
  add_compile_options(-mcpu=cortex-a53)
endif()

find_package(Threads REQUIRED)
find_package(PkgConfig)
if(PkgConfig_FOUND)
  pkg_check_modules(DRM libdrm)
endif()
find_package(JPEG)

# Core pipeline: capture sources, decoder, diversity, frame publishing.
add_library(c5core STATIC
  src/ring.c
  src/video_std.c
  src/cvbs_synth.c
  src/decoder.c
  src/diversity.c
  src/fusion.c
  src/reclock.c
  src/pipeline.c
  src/frame.c
  src/jumpers.c
  src/source.c
  src/source_synth.c
  src/source_file.c
  src/source_smi.c
  src/source_smidma.c
)
target_include_directories(c5core PUBLIC src)
# DSP hot paths: let the compiler vectorise the filter dot products (NEON on the Pi).
set_source_files_properties(src/decoder.c src/frame.c src/fusion.c src/reclock.c PROPERTIES COMPILE_OPTIONS "-O3;-ffast-math")
target_link_libraries(c5core PUBLIC m Threads::Threads)

set(APP_SRCS src/main.c src/out_y4m.c src/probe.c)
set(APP_LIBS c5core)
set(APP_DEFS)

if(DRM_FOUND)
  list(APPEND APP_SRCS src/out_drm.c)
  list(APPEND APP_DEFS HAVE_DRM=1)
else()
  message(WARNING "libdrm not found: HDMI output disabled (apt install libdrm-dev)")
endif()

if(JPEG_FOUND)
  list(APPEND APP_SRCS src/out_http.c)
  list(APPEND APP_DEFS HAVE_JPEG=1)
else()
  message(WARNING "libjpeg not found: web output disabled (apt install libjpeg62-turbo-dev)")
endif()

add_executable(c5div ${APP_SRCS})
target_compile_definitions(c5div PRIVATE ${APP_DEFS})
target_link_libraries(c5div PRIVATE ${APP_LIBS})
if(DRM_FOUND)
  target_include_directories(c5div PRIVATE ${DRM_INCLUDE_DIRS})
  target_link_libraries(c5div PRIVATE ${DRM_LIBRARIES})
endif()
if(JPEG_FOUND)
  target_include_directories(c5div PRIVATE ${JPEG_INCLUDE_DIRS})
  target_link_libraries(c5div PRIVATE ${JPEG_LIBRARIES})
endif()

install(TARGETS c5div RUNTIME DESTINATION bin)

enable_testing()
add_executable(test_pipeline test/test_pipeline.c)
target_link_libraries(test_pipeline PRIVATE c5core)
add_test(NAME pipeline COMMAND test_pipeline)
