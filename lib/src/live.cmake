# Live video: feeds, FFmpeg push (RTMP/RTSP) and WHIP.
target_sources(docksim PRIVATE
    src/live/av.cpp
    src/live/feed.cpp
    src/live/push.cpp
    src/live/whip.cpp)
target_link_libraries(docksim_internal INTERFACE docksim::av LibDataChannel::LibDataChannelStatic)
target_compile_definitions(docksim_internal INTERFACE DOCKSIM_WITH_LIVE)
