#pragma once

namespace relay_racer_integration {

inline int PlatformIdFromRacerId(const int racer_id, const int id_offset) {
  return racer_id + id_offset;
}

inline int RacerIdFromPlatformId(const int platform_id, const int id_offset) {
  return platform_id - id_offset;
}

}  // namespace relay_racer_integration
