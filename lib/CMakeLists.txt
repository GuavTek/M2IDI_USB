
add_subdirectory(midi_host)
target_include_directories(M2IDI_USB PRIVATE midi_host)
target_include_directories(M2IDI_USB PUBLIC MCP2517_driver MIDI_driver)

