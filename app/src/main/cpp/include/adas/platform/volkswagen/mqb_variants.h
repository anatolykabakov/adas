#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace volkswagen {
/** One MQB car: the same CAN layout and panda limits as every other, its own geometry and mass.
 *
 *  The numbers are the platform's published ones as openpilot's `values.py` carries them (kerb mass plus
 *  136 kg of driver and cargo, centre of mass at 45 % of the wheelbase); the Golf's are ours, measured
 *  and fitted on this car. Steering ratio and tyre stiffness are starting points — the learner refines
 *  both while driving — so a wrong entry costs convergence time, not safety. */
struct MqbVariant {
  const char* name;    ///< `vehicle.name` in the config.
  const char* label;   ///< What the parameter panel shows.
  double wheelbase_m;
  double steer_ratio;
  double mass_kg;
};

inline constexpr MqbVariant kMqbVariants[] = {
    {"vw_golf_7_mqb", "VW Golf 7 / GTI / e-Golf (MQB)", 2.636, 15.6, 1533.0},
    {"vw_jetta_7_mqb", "VW Jetta 7 (MQB)", 2.71, 15.3, 1464.0},
    {"vw_passat_b8_mqb", "VW Passat B8 (MQB)", 2.79, 15.4, 1687.0},
    {"vw_arteon_mqb", "VW Arteon (MQB)", 2.84, 15.4, 1940.0},
    {"vw_tiguan_2_mqb", "VW Tiguan 2 (MQB)", 2.74, 15.1, 1851.0},
    {"vw_touran_2_mqb", "VW Touran 2 (MQB)", 2.79, 15.6, 1652.0},
    {"vw_troc_mqb", "VW T-Roc (MQB)", 2.59, 15.1, 1549.0},
    {"vw_polo_6_mqb", "VW Polo 6 (MQB A0)", 2.55, 15.3, 1366.0},
    {"vw_atlas_mqb", "VW Atlas / Teramont (MQB)", 2.98, 15.6, 2147.0},
    {"skoda_octavia_3_mqb", "Skoda Octavia 3 (MQB)", 2.69, 15.6, 1524.0},
    {"skoda_superb_3_mqb", "Skoda Superb 3 (MQB)", 2.84, 15.1, 1641.0},
    {"skoda_kodiaq_mqb", "Skoda Kodiaq (MQB)", 2.79, 15.1, 1705.0},
    {"skoda_karoq_mqb", "Skoda Karoq (MQB)", 2.64, 15.1, 1414.0},
    {"seat_leon_3_mqb", "SEAT Leon 3 (MQB)", 2.64, 15.6, 1363.0},
    {"seat_ateca_mqb", "SEAT Ateca (MQB)", 2.64, 15.6, 1436.0},
    {"audi_a3_8v_mqb", "Audi A3 8V (MQB)", 2.64, 15.6, 1471.0},
    {"audi_q2_mqb", "Audi Q2 (MQB)", 2.60, 15.6, 1341.0},
};

/// \return The variant for a config name, or null. "volkswagen" is an alias for our own Golf.
inline const MqbVariant* findMqbVariant(std::string_view name)
{
  if (name == "volkswagen")
    name = "vw_golf_7_mqb";
  for (const auto& v : kMqbVariants)
    if (name == v.name)
      return &v;
  return nullptr;
}

/// \return Every accepted `vehicle.name`, aliases included.
inline std::vector<std::string> mqbVariantNames()
{
  std::vector<std::string> out;
  for (const auto& v : kMqbVariants)
    out.emplace_back(v.name);
  out.emplace_back("volkswagen");
  return out;
}

}  // namespace volkswagen
