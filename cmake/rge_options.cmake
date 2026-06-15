function(rge_set_target_options target_name)
    target_compile_features(${target_name} PUBLIC cxx_std_20)
    target_compile_options(${target_name} PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_link_options(${target_name} PRIVATE
            "-L/usr/lib/x86_64-linux-gnu"
            "-l:libstdc++.so.6"
        )
    endif()
endfunction()
