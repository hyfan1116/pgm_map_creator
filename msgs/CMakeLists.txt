find_package(Protobuf REQUIRED)
# https://github.com/AlexandreBourrieau/collision_map_creator_plugin/commit/72578e3af45586f689c2ad51617207c905b9d2a2
set(PROTOBUF_IMPORT_DIRS ${GAZEBO_PROTO_PATH})

set (msgs
  collision_map_request.proto
  #${PROTOBUF_IMPORT_DIRS}/vector2d.proto
  #${PROTOBUF_IMPORT_DIRS}/header.proto
  #${PROTOBUF_IMPORT_DIRS}/time.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(collision_map_creator_msgs SHARED ${PROTO_SRCS})
target_include_directories(collision_map_creator_msgs PUBLIC ${PROTOBUF_INCLUDE_DIRS})
target_link_libraries(collision_map_creator_msgs ${PROTOBUF_LIBRARY})