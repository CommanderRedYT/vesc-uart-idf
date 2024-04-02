set(headers
    src/buffer.h
    src/crc.h
    src/datatypes.h
    src/VescUart.h
)

set(sources
    src/buffer.cpp
    src/crc.cpp
    src/VescUart.cpp
)

set(dependencies
    driver
    espchrono
)

idf_component_register(
    INCLUDE_DIRS
        src
    SRCS
        ${headers}
        ${sources}
    REQUIRES
        ${dependencies}
)

# target_compile_options(${COMPONENT_TARGET}
#     PRIVATE
#         -fstack-reuse=all
# )
