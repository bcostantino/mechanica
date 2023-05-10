cmake_minimum_required(VERSION 3.0)
project(mylib C)

add_subdirectory(pwm)

# Set up library target
add_library(mylib STATIC)

# Add source files to library target
target_sources(mylib
    PRIVATE
        mylib.c
)

# Add include directories for library target
target_include_directories(mylib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link library target to pwm module
target_link_libraries(mylib
    PRIVATE
        pwm
)

# Set compiler flags for library target
target_compile_options(mylib
    PRIVATE
        -Wall
        -Wextra
        -Werror
)


