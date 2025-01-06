# CMake qpOASES module
#
# It defines the following variables:
# qpOASES_FOUND         - TRUE
# qpOASES_INCLUDE_DIR  - The qpOASES include directory
# qpOASES_LIBRARY_DIR  - The qpOASES library directory
# qpOASES_LIBRARY     - The qpOASES library
# qpOASES_VERSION       - The qpOASES version in the form <major>.<minor>




####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was qpOASESConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

####################################################################################

set(qpOASES_VERSION 3.2.2)


set_and_check(qpOASES_INCLUDE_DIR "${PACKAGE_PREFIX_DIR}/include")


set(qpOASES_INCLUDE_DIR "${qpOASES_INCLUDE_DIR}" CACHE STRING "Include path for qpOASES and its dependencies")
set_and_check(qpOASES_LIBRARY_DIR ${PACKAGE_PREFIX_DIR}/lib)
set(qpOASES_LIBRARY_DIR "${qpOASES_LIBRARY_DIR}" CACHE STRING "Library path for qpOASES and its dependencies")

find_library(qpOASES_LIBRARY NAMES qpOASES
    PATHS ${qpOASES_LIBRARY_DIR} NO_DEFAULT_PATH)


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(qpOASES DEFAULT_MSG qpOASES_INCLUDE_DIR qpOASES_LIBRARY_DIR qpOASES_LIBRARY qpOASES_VERSION)
