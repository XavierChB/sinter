cmake_minimum_required(VERSION 3.5)

aux_source_directory(. LIB_SRCS)
add_library(components ${LIB_SRCS})

target_link_libraries(components idf::esp32 idf::freertos)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)
