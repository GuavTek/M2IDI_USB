cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(M2IDI_USB CXX C ASM)

set(PICO_BOARD board_m2idi_usb)
set(PICO_BOARD_HEADER_DIRS ${PROJECT_SOURCE_DIR}/src)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
file(GLOB src_list CONFIGURE_DEPENDS src/*.cpp src/*.c)
add_executable(M2IDI_USB
    ${src_list}
)

pico_generate_pio_header(M2IDI_USB ${CMAKE_CURRENT_LIST_DIR}/src/i2s.pio)

add_subdirectory(lib)

# Add library files
target_include_directories(M2IDI_USB PUBLIC src)

target_link_options(M2IDI_USB PUBLIC -Xlinker --print-memory-usage)
target_compile_options(M2IDI_USB PRIVATE -Wall -Wextra)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_RELEASE -O3)
set(CMAKE_CXX_FLAGS_DEBUG -g)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(M2IDI_USB pico_stdlib pico_unique_id hardware_pio hardware_timer hardware_spi hardware_dma midi_driver mcp2517_driver tinyusb_host usb_midi_host_app_driver tinyusb_device)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(M2IDI_USB)
