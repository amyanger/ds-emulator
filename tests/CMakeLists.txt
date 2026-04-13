# ds_tests — one binary per test source file, each links ds_core only.
# No SDL dependency, runs in milliseconds.

function(add_ds_unit_test name)
    add_executable(${name} unit/${name}.cpp)
    target_link_libraries(${name} PRIVATE ds_core)
    target_include_directories(${name} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/tests/support
    )
    target_enable_warnings(${name})
    target_apple_silicon(${name})
    target_enable_sanitizers(${name})
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_ds_unit_test(fixed_test)
add_ds_unit_test(ring_buffer_test)
add_ds_unit_test(scheduler_test)
add_ds_unit_test(page_table_test)
add_ds_unit_test(arm9_bus_test)
add_ds_unit_test(arm7_bus_test)
add_ds_unit_test(wramcnt_test)
add_ds_unit_test(nds_integration_test)
add_ds_unit_test(arm7_register_file_test)
