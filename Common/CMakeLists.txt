include_directories(./include)
aux_source_directory(./src COMMON_DIRS)
add_library(common SHARED ${COMMON_DIRS})
