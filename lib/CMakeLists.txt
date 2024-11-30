
add_subdirectory(midi_host)
add_subdirectory(MCP2517_driver)
add_subdirectory(AM_MIDI2.0Lib)
target_include_directories(M2IDI_USB PRIVATE midi_host MCP2517_driver AM_MIDI2.0Lib)
