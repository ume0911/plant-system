string(REPLACE "\\" "/" BUILD_DIR_UNIX "${CMAKE_BINARY_DIR}")
string(FIND "${BUILD_DIR_UNIX}" "/master/" IS_MASTER)

if(IS_MASTER GREATER -1)
    idf_component_register(
        SRCS "main.c"
        INCLUDE_DIRS "." "../slave"
    )
else()
    idf_component_register()
endif()