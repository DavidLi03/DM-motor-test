cmake_minimum_required(VERSION 3.16)
project(dmcan_ws CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

foreach(tgt dm_probe dm_hold dm_spin dm_batch)
    add_executable(${tgt} src/${tgt}.cpp)

    target_include_directories(${tgt} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_link_libraries(${tgt} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdm_device.so
        Threads::Threads
    )

    # 让可执行文件在原地就能找到 libdm_device.so，无需设置 LD_LIBRARY_PATH
    set_target_properties(${tgt} PROPERTIES
        BUILD_RPATH ${CMAKE_CURRENT_SOURCE_DIR}/lib
        INSTALL_RPATH ${CMAKE_CURRENT_SOURCE_DIR}/lib
    )
endforeach()

# SocketCAN 路线（slcan 固件 + slcand，见 SOCKETCAN.md）——只依赖内核 API，不链厂商库
foreach(tgt dm_probe_sc dm_batch_sc dm_spin_sc dm_demo_sc dm_fric_sc dm_stir_sc dm_rated_sc dm_grip_sc dm_gripctl_sc dm_gripfric_sc)
    add_executable(${tgt} src/${tgt}.cpp)
    target_link_libraries(${tgt} PRIVATE Threads::Threads)
endforeach()
