 cmake_minimum_required(VERSION 3.16)
project(energyStorage)
find_package(Threads REQUIRED)

set(CMAKE_C_STANDARD 11)

 # ====================== 关键修改：开启 Debug 模式 + 调试信息 ======================
# # 默认构建类型
# if(NOT CMAKE_BUILD_TYPE)
#  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
# endif()
#
# set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
#         Debug
#         Release
#         RelWithDebInfo
#         MinSizeRel
# )
#
# # 通用调试友好选项
# add_compile_options(
#         -Wall
#         -Wextra
#         -fno-omit-frame-pointer
# )
#
# # Debug
# set(CMAKE_C_FLAGS_DEBUG "-g -O0")
# set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
#
# # RelWithDebInfo：推荐部署到工控机做问题复现/抓 core
# set(CMAKE_C_FLAGS_RELWITHDEBINFO "-g -O2")
# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g -O2")
#
# # Release
# set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")
# set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
 # ================================================================================

# 1. 基础配置：C++ 标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # 强制要求 C++20，缺失则报错

# 2. 交叉编译配置（RK3568 是 ARM64 架构）
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

#set(CMAKE_SYSTEM_PROCESSOR arm)
#set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
#set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)

# 3. 先创建目标（必须在 target_* 指令之前）

add_executable(energyStorage
        app/main.cpp
        app/app_manager.cpp
        app/app_manager.h
        app/state_machine/state_idle.cpp
        app/state_machine/state_run.cpp
        app/state_machine/state_error.cpp
        app/state_machine/state_base.h
        driver/can/can_driver.cpp
        driver/can/can_driver.h
        driver/can/can_thread.cpp
        driver/serial/rs485_driver.cpp
        driver/serial/rs485_driver.h
        driver/gpio/di.cpp
        driver/gpio/do.cpp
        driver/adc/ai.cpp
        driver/adc/ai.h
        driver/serial/serial_driver_base.cpp
        driver/serial/serial_driver_base.h
        services/mqtt/mqtt_client.cpp
        services/mqtt/mqtt_client.h
        services/mqtt/mqtt_thread.cpp
        services/protocol/protocol_base.h
        utils/logger/logger.cpp
        utils/logger/logger.h
        utils/queue/msg_queue.cpp
        utils/queue/msg_queue.h
        utils/thread/thread_base.cpp
        utils/thread/thread_base.h
        utils/timer/timer_thread.cpp
        utils/timer/timer_thread.h
        third_party/paho_mqtt.c
        driver/can/can_thread.h
        driver/serial/serial_thread.cpp
        driver/serial/serial_thread.h
        driver/common/serial_config.h
        driver/serial/serial_event.h
        UNUSED/board/board.h
        driver/driver_manager.cpp
        driver/driver_manager.h
        app/state_machine/state_error.h
        app/state_machine/state_run.h
        app/state_machine/state_idle.h
        app/event.h
        driver/timer/timer_thread.cpp
        driver/timer/timer_thread.h
        services/protocol/can/can_protocol_base.h
        services/protocol/can/can_dispatcher.cpp
        services/protocol/can/can_dispatcher.h
        services/protocol/can/proto_bms.cpp
        services/protocol/can/proto_bms.h
        services/protocol/can/proto_custom.cpp
        services/protocol/can/proto_custom.h
        services/protocol/rs485/modbusProtocol.h
        services/protocol/modbus/modbus_rtu.cpp
        services/protocol/modbus/modbus_rtu.h
        services/protocol/modbus/modbus_crc.cpp
        services/protocol/modbus/modbus_crc.h
        services/protocol/rs485/air_conditioner_proto.cpp
        services/protocol/rs485/air_conditioner_proto.h
        services/protocol/rs485/smoke_sensor_proto.cpp
        services/protocol/rs485/smoke_sensor_proto.h
        services/protocol/rs485/gas_detector_proto.cpp
        services/protocol/rs485/gas_detector_proto.h
        services/protocol/protocol_factory.cpp
        services/protocol/protocol_factory.h
        services/protocol/rs485/rs485_session.cpp
        services/protocol/rs485/rs485_session.h
        app/app_context.h
        driver/net/eth_driver.cpp
        driver/net/eth_driver.h
        driver/net/eth_thread.cpp
        driver/net/eth_thread.h
        services/output/uplink_eth.cpp
        services/output/uplink_eth.h
        services/device/device_base.h
        services/device/device_types.h
        app/scheduler/scheduler_timer.cpp
        app/scheduler/scheduler_timer.h
        app/scheduler/device_scheduler.cpp
        app/scheduler/device_scheduler.h
        services/display/display_data_map.cpp
        services/display/display_data_map.h
        services/output/display_rs485_session.cpp
        services/output/display_rs485_session.h
        services/aggregator/system_snapshot.h
        services/aggregator/data_aggregator.cpp
        services/aggregator/data_aggregator.h
        utils/logger/hex_dump.h
        services/parser/parser_message.h
        services/parser/protocol_parser_thread.cpp
        services/parser/protocol_parser_thread.h
        services/parser/frame_assembler.cpp
        services/parser/frame_assembler.h
        services/parser/crc_policy.cpp
        services/parser/crc_policy.h
        services/parser/protocol_router.cpp
        services/parser/protocol_router.h
        services/parser/232ups_ascii/ascii_frame_assembler.cpp
        services/protocol/rs232/ups_232_ascii_proto.cpp
        driver/serial/rs232_driver.cpp
        driver/serial/rs232_driver.h
        services/snapshot/snapshot_dispatcher.cpp
        services/snapshot/snapshot_dispatcher.h
        services/snapshot/snapshot_sink.h
        services/snapshot/uplink_sink.h
        services/snapshot/uplink_sink.cpp
        services/snapshot/file_sink.h
        services/snapshot/file_sink.cpp
        services/snapshot/display_sink.h
        services/snapshot/display_sink.cpp
        services/parser/request_context.h
        services/protocol/rs485/hmi/hmi_proto.cpp
        services/protocol/rs485/hmi/hmi_proto.h
        utils/config/config_loader.h
        utils/config/config_loader.cpp
        services/parser/modbus_rtu_req_assembler.cpp
        services/parser/modbus_rtu_req_assembler.h
        services/protocol/rs485/hmi/hmi_address_table.cpp
        services/protocol/rs485/hmi/hmi_address_table.h
        services/snapshot/display/hmi_display_map.cpp
        services/snapshot/display/hmi_display_map.h
        services/snapshot/display/hmi_snapshot_mapper.cpp
        services/snapshot/display/hmi_snapshot_mapper.h
        utils/time/getTime.h
        services/protocol/j1939/opensae_port.cpp
        services/protocol/j1939/opensae_port.h
        services/protocol/j1939/j1939_stack.cpp
        services/protocol/j1939/j1939_stack.h
        services/protocol/j1939/j1939_manager.cpp
        services/protocol/j1939/j1939_manager.h
        services/protocol/j1939/j1939_types.h
        services/protocol/j1939/opensae_adapter.c
        services/protocol/j1939/opensae_adapter.h
        services/protocol/can/pcu/proto_pcu.cpp
        services/protocol/can/pcu/proto_pcu.h
        services/aggregator/snapshot_device_types.h
        services/protocol/can/bms/proto_bms_table_runtime.cpp
        services/protocol/can/bms/proto_bms_table_runtime.h
        services/protocol/can/bms/bit_extract.h
        services/protocol/can/bms/bms_proto.cpp
        services/protocol/can/bms/bms_proto.h
        services/protocol/can/bms/bms_tx.cpp
        services/protocol/can/bms/bms_tx.h

        services/protocol/can/bms/generated/proto_bms_table.h
        services/protocol/can/bms/generated/proto_bms_table.cpp
        services/aggregator/bms/bms_snapshot.cpp
        services/aggregator/bms/bms_snapshot.h
        services/snapshot/bms_filesink/bms_snapshot_sink.h
        services/snapshot/bms_filesink/bms_file_sink.cpp
        services/snapshot/bms_filesink/bms_file_sink.h
        services/protocol/can/bms/bms_thread/bms_worker.cpp
        services/protocol/can/bms/bms_thread/bms_worker.h
        services/protocol/can/bms/bms_thread/bms_queue.cpp
        services/protocol/can/bms/bms_thread/bms_queue.h
        services/protocol/can/bms/bms_thread/bms_frame_router.cpp
        services/protocol/can/bms/bms_thread/bms_frame_router.h
        services/control/control_commands.h
        services/control/command_dispatcher.cpp
        services/control/command_dispatcher.h
        services/control/control_loop.cpp
        services/control/control_loop.h
        services/control/control_events.h
        services/control/fault/fault_ids.h
        services/control/fault/fault_addr_layout.h
        services/control/fault/fault_catalog.cpp
        services/control/fault/fault_catalog.h
        services/control/fault/fault_page_manager.cpp
        services/control/fault/fault_page_manager.h
        services/normal/normal_addr_layout.h
        services/normal/normal_hmi_writer.cpp
        services/normal/normal_hmi_writer.h
        services/protocol/can/bms/bms_name_map.cpp
        services/protocol/can/bms/bms_name_map.h
        services/control/bms/bms_logic_types.h
        services/control/bms/bms_logic_adapter.cpp
        services/control/bms/bms_logic_adapter.h
        services/protocol/can/bms/bms_command_types.h
        services/control/bms/bms_command_manager.cpp
        services/control/bms/bms_command_manager.h
        services/control/bms/bms_fault_mapper.cpp
        services/control/bms/bms_fault_mapper.h
        services/control/out/model_path_exporter.cpp
        services/control/out/model_path_exporter.h
        services/control/bms/bms_logic_adapter_oneFactor.h
        services/snapshot/sqlite/sqlite_snapshot_sink/sqlite_snapshot_sink.cpp
        services/snapshot/sqlite/sqlite_snapshot_sink/sqlite_snapshot_sink.h
        services/snapshot/sqlite/sqlite_bms_sink/sqlite_bms_sink.cpp
        services/snapshot/sqlite/sqlite_bms_sink/sqlite_bms_sink.h
        services/snapshot/sqlite/sqlite_snapshot_sink/sqlite_snapshot_flat_sink.cpp
        services/snapshot/sqlite/sqlite_snapshot_sink/sqlite_snapshot_flat_sink.h
        services/snapshot/sqlite/sqlite_bms_sink/sqlite_bms_flat_sink.cpp
        services/snapshot/sqlite/sqlite_bms_sink/sqlite_bms_flat_sink.h
        services/control/logic/logic_engine.cpp
        services/control/logic/logic_engine.h
        services/control/logic/logic_device_data.cpp
        services/control/logic/logic_snapshot.cpp
        services/control/logic/logic_hmi_input.cpp
        services/control/logic/logic_tick.cpp
        services/control/logic/logic_link.cpp
        services/control/logic/logic_io.cpp
        services/control/logic/logic_view_builder.cpp
        services/control/logic/logic_hmi_output.cpp
        services/control/logic/logic_helpers.cpp
        services/control/logic/logic_helpers.h
        services/control/logic/logic_context.h
        services/snapshot/sqlite/sqlite_fault_sink/sqlite_fault_sink.cpp
        services/snapshot/sqlite/sqlite_fault_sink/sqlite_fault_sink.h
        services/control/fault/fault_center.cpp
        services/control/fault/fault_center.h
        services/control/fault/fault_runtime_mapper.cpp
        services/control/fault/fault_runtime_mapper.h
        services/control/logic/logic_fault_refresh.cpp
        services/control/fault/fault_condition_state.cpp
        services/control/fault/fault_condition_state.h
        services/control/fault/fault_condition_engine.cpp
        services/control/fault/fault_condition_engine.h
        services/control/bms/bms_fault_evaluator.cpp
        services/control/bms/bms_fault_evaluator.h
        services/control/fault/fault_logic_evaluator.cpp
        services/control/fault/fault_logic_evaluator.h

)


find_package(Threads REQUIRED)

 add_subdirectory(third_party/openSAE_j1939)
 target_link_libraries(energyStorage PRIVATE openSAE_j1939)


target_include_directories(energyStorage PRIVATE ${GPIOD_INCLUDE_DIRS})
target_link_libraries(energyStorage
        PRIVATE
        sqlite3
        ${CMAKE_DL_LIBS}
        Threads::Threads
        ${GPIOD_LIBRARIES}
)

# 4. 配置头文件目录（PRIVATE 表示仅当前项目内部使用）
target_include_directories(${PROJECT_NAME}
        PRIVATE
        ${PROJECT_SOURCE_DIR}/app
        ${PROJECT_SOURCE_DIR}/app/state_machine
        UNUSED/board
        ${PROJECT_SOURCE_DIR}/driver
        ${PROJECT_SOURCE_DIR}/driver/adc
        ${PROJECT_SOURCE_DIR}/driver/can
        ${PROJECT_SOURCE_DIR}/driver/serial
        ${PROJECT_SOURCE_DIR}/driver/gpio
        ${PROJECT_SOURCE_DIR}/driver/common
        ${PROJECT_SOURCE_DIR}/services/mqtt
        ${PROJECT_SOURCE_DIR}/services/protocol
        ${PROJECT_SOURCE_DIR}/services/protocol/modbus
        ${PROJECT_SOURCE_DIR}/services/protocol/rs485
        ${PROJECT_SOURCE_DIR}/services/protocol/can
        ${PROJECT_SOURCE_DIR}/services/snapshot
        ${PROJECT_SOURCE_DIR}/utils/config
        ${PROJECT_SOURCE_DIR}/utils/logger
        ${PROJECT_SOURCE_DIR}/utils/queue
        ${PROJECT_SOURCE_DIR}/utils/thread
        ${PROJECT_SOURCE_DIR}/utils/timer
        ${PROJECT_SOURCE_DIR}/third_party
        ${PROJECT_SOURCE_DIR}/utils/time
        ${PROJECT_SOURCE_DIR}/driver/timer
        ${PROJECT_SOURCE_DIR}/services/aggregator/bms
        ${PROJECT_SOURCE_DIR}/services/protocol/can/bms/
        ${PROJECT_SOURCE_DIR}/services/protocol/can/bms/bms_thread
        ${PROJECT_SOURCE_DIR}/services/control/
        ${PROJECT_SOURCE_DIR}/services/control/logic/
        ${PROJECT_SOURCE_DIR}/services/fault/
        ${PROJECT_SOURCE_DIR}/services/snapshot/sqlite/sqlite_snapshot_sink/
        ${PROJECT_SOURCE_DIR}/services/snapshot/sqlite/sqlite_bms_sink/







)
 add_compile_definitions(LOG_LEVEL=LogLevel::Debug)
 # Release
 # add_compile_definitions(LOG_LEVEL=LogLevel::Info)


# （可选）若需链接 RK3568 相关库（如驱动库、SDK 库），添加以下配置
# link_directories(/path/to/rk3568_libs)  # 库文件在 WSL 中的路径
# target_link_libraries(${PROJECT_NAME} PRIVATE rk_gpio rk_hdmi)  # 需链接的库名

## 1. 为目标添加-pthread编译选项（启用线程相关编译特性）
#target_compile_options(energyStorage PRIVATE -pthread)
## 2. 链接pthread库（解决未定义引用问题）
#target_link_libraries(energyStorage PRIVATE pthread)