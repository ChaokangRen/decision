#include "ssc_map.h"
namespace jarvis {
namespace decision_lib {

void SscMap::print() {
    std::array<int, 3> map_size = {{10, 10, 8}};                   // s, d, t
    std::array<decimal_t, 3> map_resolution = {{0.25, 0.2, 0.1}};  // m, m, s
    std::array<std::string, 3> axis_name = {{"s", "d", "t"}};
    common::GridMapND<uint8_t, 3> grid_map_3d(map_size, map_resolution,
                                              axis_name);
}
}  // namespace decision_lib
}  // namespace jarvis