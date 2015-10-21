set(SOURCES
    "src/plane_to_hardware.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/plane_to_hardware.h"
)

include_directories(include)
add_library(plane_to_hardware MODULE ${SOURCES} ${HEADERS})
target_link_libraries(plane_to_hardware PRIVATE lmscore)
