set(SOURCES
    "src/hardware_to_plane.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/hardware_to_plane.h"
)

include_directories(include)
add_library(hardware_to_plane MODULE ${SOURCES} ${HEADERS})
target_link_libraries(hardware_to_plane PRIVATE lmscore)
