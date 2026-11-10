#include "services/peripherals.hpp"

#include "log/log.hpp"

#include <string_view>

namespace docksim::services {

void register_psdk(Registry& registry) {
    static constexpr std::string_view methods[] = {
        "speaker_play_volume_set",
        "speaker_play_mode_set",
        "speaker_play_stop",
        "speaker_replay",
        "speaker_tts_play_start",
        "speaker_audio_play_start",
        "psdk_input_box_text_set",
        "psdk_widget_value_set",
        "custom_data_transmission_to_psdk",
        "custom_data_transmission_to_esdk",
    };
    for (const auto method : methods) {
        registry.add(std::string(method), [method](const Json& data) -> asio::awaitable<Reply> {
            log::get("dock_sim.psdk")
                ->warn("{}: no PSDK or ESDK payload on the aircraft: {}", method, data.dump());
            co_return result(code::Result::not_supported);
        });
    }
}

void register_remote_unlocking(Registry& registry) {
    registry.add("unlock_license_list", [](const Json& data) -> asio::awaitable<Reply> {
        co_return ok({{"consistence", true},
                      {"device_model_domain", integer(data, "device_model_domain", 0)},
                      {"licenses", Json::array()}});
    });
    registry.add("unlock_license_switch", [](const Json& data) -> asio::awaitable<Reply> {
        std::ignore = flag(data, "enable", false);
        co_return ok({{"license_id", integer(data, "license_id", 0)}});
    });
    // The simulator does not download the license file.
    registry.add("unlock_license_update",
                 [](const Json&) -> asio::awaitable<Reply> { co_return ok(); });
}

}  // namespace docksim::services
