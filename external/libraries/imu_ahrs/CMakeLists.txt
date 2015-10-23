set ( SOURCES
    "src/madgwick_AHRS.cpp"
    "src/mahony_AHRS.cpp"
)

set (HEADERS
    "include/imu_ahrs/madgwick_AHRS.h"
    "include/imu_ahrs/mahony_AHRS.h"
    "include/imu_ahrs/ahrs.h"
)

include_directories("include")

add_library(imu_ahrs_lib SHARED ${SOURCES} ${HEADERS})

