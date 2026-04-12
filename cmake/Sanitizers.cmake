# Optional: enable ASan/UBSan on a target when DS_ENABLE_SANITIZERS=ON.
# Usage: target_enable_sanitizers(my_target)
function(target_enable_sanitizers target)
    if(NOT DS_ENABLE_SANITIZERS)
        return()
    endif()
    if(MSVC)
        return()  # unsupported
    endif()
    target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(${target}    PRIVATE -fsanitize=address,undefined)
endfunction()
