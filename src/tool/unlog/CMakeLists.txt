cmake_minimum_required( VERSION 2.8.3 )

set( UNLOG_SRCS UnlogModule.cpp )

add_library(
  unlog
  STATIC
  ${UNLOG_SRCS}
)

target_link_libraries(
  unlog
  portals
  messages
  ${PROTOBUF_LIBRARY}
)