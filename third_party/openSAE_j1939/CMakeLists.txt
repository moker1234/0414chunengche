# third_party/openSAE_j1939/CMakeLists.txt
# 注意：这里不要写 project()，不要 set(CMAKE_C_COMPILER...)。
# 作为主工程的子目录库使用：add_subdirectory(third_party/openSAE_j1939)

# ============================
# Options
# ============================
option(OPENSAE_ENABLE_ISO11783 "Build ISO11783 (ISOBUS) application layer" OFF)

# OpenSAE Hardware.h 里：INTERNAL_CALLBACK = 6
set(OPENSAE_TARGET_PLATFORM_INTERNAL_CALLBACK 6)

# ============================
# Library
# ============================
add_library(openSAE_j1939 STATIC)

# include
target_include_directories(openSAE_j1939 PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/Open_SAE_J1939
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939
        ${CMAKE_CURRENT_LIST_DIR}/Hardware
        ${CMAKE_CURRENT_LIST_DIR}/ISO_11783
)

# OpenSAE 推荐 C99
set_target_properties(openSAE_j1939 PROPERTIES
        C_STANDARD 99
        C_EXTENSIONS OFF
)

# INTERNAL_CALLBACK：走你工程自己的 CAN 收发线程
target_compile_definitions(openSAE_j1939 PUBLIC
        OPENSAE_J1939_TARGET_PLATFORM=${OPENSAE_TARGET_PLATFORM_INTERNAL_CALLBACK}
)

# ============================
# Sources - Core
# ============================
target_sources(openSAE_j1939 PRIVATE
        # ---- Open_SAE_J1939 core ----
        ${CMAKE_CURRENT_LIST_DIR}/Open_SAE_J1939/Startup_ECU.c
        ${CMAKE_CURRENT_LIST_DIR}/Open_SAE_J1939/Listen_For_Messages.c
        ${CMAKE_CURRENT_LIST_DIR}/Open_SAE_J1939/Closedown_ECU.c

        # ---- SAE_J1939-21 Transport ----
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-21_Transport_Layer/Acknowledgement.c
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-21_Transport_Layer/Request.c
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-21_Transport_Layer/Transport_Protocol_Connection_Management.c
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-21_Transport_Layer/Transport_Protocol_Data_Transfer.c

        # ---- SAE_J1939-81 Network Management ----
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-81_Network_Management_Layer/Address_Claimed.c
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-81_Network_Management_Layer/Address_Delete.c
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-81_Network_Management_Layer/Address_Not_Claimed.c
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-81_Network_Management_Layer/Commanded_Address.c

        # ---- SAE_J1939-71 Application ----
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-71_Application_Layer/Application_Layer.c
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-71_Application_Layer/Request_Component_Identification.c
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-71_Application_Layer/Request_ECU_Identification.c
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-71_Application_Layer/Request_Proprietary.c
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-71_Application_Layer/Request_Proprietary_B.c
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-71_Application_Layer/Request_Software_Identification.c

        # ---- SAE_J1939-73 Diagnostics ----
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-73_Diagnostics_Layer/DM1.c
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-73_Diagnostics_Layer/DM2.c
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-73_Diagnostics_Layer/DM3.c
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-73_Diagnostics_Layer/DM14.c
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-73_Diagnostics_Layer/DM15.c
        ${CMAKE_CURRENT_LIST_DIR}/SAE_J1939/SAE_J1939-73_Diagnostics_Layer/DM16.c

        # ---- Hardware (必须) ----
#        ${CMAKE_CURRENT_LIST_DIR}/Hardware/CAN_Transmit_Receive.c
        ${CMAKE_CURRENT_LIST_DIR}/Hardware/Save_Load_Struct.c
        ${CMAKE_CURRENT_LIST_DIR}/Hardware/FLASH_EEPROM_RAM_Memory.c
)

# ============================
# ISO11783 (Optional / stub)
# ============================
if (OPENSAE_ENABLE_ISO11783)
    target_sources(openSAE_j1939 PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/ISO_11783/ISO_11783-7_Application_Layer/Auxiliary_Valve_Command.c
            ${CMAKE_CURRENT_LIST_DIR}/ISO_11783/ISO_11783-7_Application_Layer/Auxiliary_Valve_Estimated_Flow.c
            ${CMAKE_CURRENT_LIST_DIR}/ISO_11783/ISO_11783-7_Application_Layer/Auxiliary_Valve_Measured_Position.c
            ${CMAKE_CURRENT_LIST_DIR}/ISO_11783/ISO_11783-7_Application_Layer/General_Purpose_Valve_Command.c
            ${CMAKE_CURRENT_LIST_DIR}/ISO_11783/ISO_11783-7_Application_Layer/General_Purpose_Valve_Estimated_Flow.c
            ${CMAKE_CURRENT_LIST_DIR}/ISO_11783/ISO_11783-7_Application_Layer/General_Purpose_Valve_Measured_Position.c
    )
else()
    target_sources(openSAE_j1939 PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/iso11783_stub.c
    )
endif()
