cmake_minimum_required(VERSION 3.12)

project(implot LANGUAGES CXX)

add_library(implot STATIC
  implot.cpp
  implot_items.cpp

)
