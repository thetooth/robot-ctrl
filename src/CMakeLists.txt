set(sources_list
    main.cpp
)

# Getting all the .cpp files in any src/directory.
file(GLOB directories */)
foreach(dir ${directories})
    if(IS_DIRECTORY ${dir})
        string(FIND ${dir} "/" last_slash_pos REVERSE)
        math(EXPR string_start "${last_slash_pos}+1")
        string(SUBSTRING ${dir} ${string_start} -1 dir_stripped)
        file(GLOB_RECURSE sources ${dir_stripped}/*.cpp)
        list(APPEND sources_list ${sources})
    endif()
endforeach()

add_executable(${PROJECT_NAME} ${sources_list})

target_compile_features(
    ${PROJECT_NAME}
    PUBLIC
        c_std_11
        cxx_std_20
)

set_target_properties(
    ${PROJECT_NAME}
        PROPERTIES
            CXX_STANDARD_REQUIRED ON
            CXX_EXTENSIONS ON
)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CUSTOM_COMPILE_OPTIONS
        -Wall
    )
    if (UNIX)
        list(APPEND CUSTOM_COMPILE_OPTIONS
            -Werror
            -Wpedantic 
            -Wextra
            -Wno-gnu-anonymous-struct
            -Wno-gnu-zero-variadic-macro-arguments
            -Wno-nested-anon-types
        )
    endif()
    target_compile_options(
        ${PROJECT_NAME}
        PUBLIC
            ${CUSTOM_COMPILE_OPTIONS}
    )
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/CAN)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/FSM)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/IK)
