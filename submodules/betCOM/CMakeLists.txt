#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libtmpTemplate-<target>.so
# and installs in the directory lib/orocos/tmpTemplate/
#

# Generate the Versions.proto file
set(VERSIONS_PROTOBUF Versions.proto)
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/${VERSIONS_PROTOBUF}
    COMMAND python find_versions.py > ${VERSIONS_PROTOBUF}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/find_versions.py
)

# Build the BetCOM library
INCLUDE(FindProtobuf)
FIND_PACKAGE(Protobuf REQUIRED)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIR})

FILE(GLOB PROTOFILES *.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADERS ${PROTOFILES} ${VERSIONS_PROTOBUF})

orocos_library(BetCOM SHARED ${PROTO_SRC})
TARGET_LINK_LIBRARIES(BetCOM ${PROTOBUF_LIBRARIES})

orocos_install_headers(${PROTO_HEADERS})
