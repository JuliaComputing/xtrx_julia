cmake_minimum_required(VERSION 3.17)
project(SoapySDRXTRX CXX C)

option(GPU_SUPPORT "Enable GPU DMA Support" OFF)

########################################################################
## LitePCIe discovery
########################################################################

find_path(LITEPCIE_KERNEL_INCLUDE_DIR litepcie.h
          REQUIRED
          HINTS
            ${CMAKE_CURRENT_SOURCE_DIR}/../..
          PATH_SUFFIXES
            software/kernel)
find_path(LITEPCIE_USER_INCLUDE_DIR liblitepcie.h
          REQUIRED
          HINTS
            ${CMAKE_CURRENT_SOURCE_DIR}/../..
          PATH_SUFFIXES
            software/user/liblitepcie)
set(LITEPCIE_INCLUDE_DIR ${LITEPCIE_KERNEL_INCLUDE_DIR} ${LITEPCIE_USER_INCLUDE_DIR})
find_library(LITEPCIE_LIBRARY litepcie
             REQUIRED
             HINTS
               ${CMAKE_CURRENT_SOURCE_DIR}/../..
             PATH_SUFFIXES
               software/user/liblitepcie)

include_directories(${LITEPCIE_INCLUDE_DIR})


########################################################################
## LMS7002M discovery
########################################################################

include(ExternalProject)

set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)

ExternalProject_Add(LMS7002M-driver
    GIT_REPOSITORY https://github.com/JuliaComputing/LMS7002M-driver
    GIT_TAG tb/fclk_dly
    GIT_SHALLOW
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION}
)

include_directories(${EXTERNAL_INSTALL_LOCATION}/include)

link_directories(${EXTERNAL_INSTALL_LOCATION}/lib)

########################################################################
## Test executable
########################################################################

add_executable(main main.c)

add_dependencies(main LMS7002M-driver)

########################################################################
## CUDA discovery
########################################################################

if(GPU_SUPPORT)
  find_package(CUDAToolkit REQUIRED)

  include_directories(${CUDAToolkit_INCLUDE_DIRS})

  add_definitions(-DGPU)

  target_link_libraries(main ${LITEPCIE_LIBRARY} CUDA::cuda_driver LMS7002M m)

else()

  target_link_libraries(main ${LITEPCIE_LIBRARY} LMS7002M m)

endif()




########################################################################
## SoapySDR library
########################################################################

find_package(SoapySDR "0.2.1" REQUIRED)

if(GPU_SUPPORT)
  SOAPY_SDR_MODULE_UTIL(
      TARGET SoapyXTRX
      SOURCES XTRXDevice.cpp Streaming.cpp
      LIBRARIES ${LITEPCIE_LIBRARY} CUDA::cuda_driver LMS7002M m
  )
else()
  SOAPY_SDR_MODULE_UTIL(
      TARGET SoapyXTRX
      SOURCES XTRXDevice.cpp Streaming.cpp
      LIBRARIES ${LITEPCIE_LIBRARY} LMS7002M m
  )
endif()

add_dependencies(SoapyXTRX LMS7002M-driver)
