set(SOURCES
    "src/plane_visualizer.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/plane_visualizer.h"
)

include_directories(include)
add_library(plane_visualizer MODULE ${SOURCES} ${HEADERS})
target_link_libraries(plane_visualizer PRIVATE lmscore)
