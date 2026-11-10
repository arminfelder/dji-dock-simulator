#pragma once

#include <format>
#include <string>
#include <string_view>

// Topic builders. Each takes the serial number of the device that owns the topic.
namespace docksim::topic {

namespace detail {
inline std::string sys(std::string_view sn, std::string_view leaf) {
    return std::format("sys/product/{}/{}", sn, leaf);
}
inline std::string thing(std::string_view sn, std::string_view leaf) {
    return std::format("thing/product/{}/{}", sn, leaf);
}
}  // namespace detail

inline std::string status(std::string_view sn) {
    return detail::sys(sn, "status");
}
inline std::string status_reply(std::string_view sn) {
    return detail::sys(sn, "status_reply");
}
inline std::string network_probe(std::string_view sn) {
    return detail::sys(sn, "network/probe");
}
inline std::string osd(std::string_view sn) {
    return detail::thing(sn, "osd");
}
inline std::string state(std::string_view sn) {
    return detail::thing(sn, "state");
}
inline std::string state_reply(std::string_view sn) {
    return detail::thing(sn, "state_reply");
}
inline std::string services(std::string_view sn) {
    return detail::thing(sn, "services");
}
inline std::string services_reply(std::string_view sn) {
    return detail::thing(sn, "services_reply");
}
inline std::string events(std::string_view sn) {
    return detail::thing(sn, "events");
}
inline std::string events_reply(std::string_view sn) {
    return detail::thing(sn, "events_reply");
}
inline std::string requests(std::string_view sn) {
    return detail::thing(sn, "requests");
}
inline std::string requests_reply(std::string_view sn) {
    return detail::thing(sn, "requests_reply");
}
inline std::string property_set(std::string_view sn) {
    return detail::thing(sn, "property/set");
}
inline std::string property_set_reply(std::string_view sn) {
    return detail::thing(sn, "property/set_reply");
}
inline std::string drc_up(std::string_view sn) {
    return detail::thing(sn, "drc/up");
}
inline std::string drc_down(std::string_view sn) {
    return detail::thing(sn, "drc/down");
}

}  // namespace docksim::topic
