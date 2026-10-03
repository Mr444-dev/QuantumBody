#pragma once

#include "quantumbody/world.hpp"

#include <cstdint>
#include <string_view>

namespace qb {

World makeScene(std::string_view name, std::uint32_t seed = 7, std::uint32_t objectCount = 240,
                WorldSettings settings = {});

} // namespace qb
