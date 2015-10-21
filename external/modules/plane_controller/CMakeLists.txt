set(SOURCES
    "src/plane_controller.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/plane_controller.h"
)

include_directories(include)
add_library(plane_controller MODULE ${SOURCES} ${HEADERS})
target_link_libraries(plane_controller PRIVATE lmscore)
