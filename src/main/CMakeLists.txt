string(REPLACE "\\" "/" BUILD_DIR_UNIX "${CMAKE_BINARY_DIR}")
string(FIND "${BUILD_DIR_UNIX}" "/master/" IS_MASTER)

if(IS_MASTER GREATER -1)
    idf_component_register(
        SRCS "../master/main.c"
        INCLUDE_DIRS "../master" "../slave"
    )
else()
    idf_component_register(
        SRCS "../slave/main.c"
             "../slave/config_store.c"
        INCLUDE_DIRS "../slave"
    )
endif()