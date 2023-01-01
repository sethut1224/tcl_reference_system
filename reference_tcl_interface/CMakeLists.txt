cmake_minimum_required(VERSION 3.14)
project(reference_tcl_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(reference_tcl_interface SHARED
  include/reference_tcl_interface/reference_tcl_interface.hpp
  src/reference_tcl_interface.cpp
)


ament_auto_package()
