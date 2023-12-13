#ifndef _SSC_MAP_H_
#define _SSC_MAP_H_

#include <algorithm>
#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "semantics.h"
namespace jarvis {
namespace decision_lib {
using ObstacleMapType = uint8_t;
using SscMapDataType = uint8_t;

class SscMap {
public:
    using GridMap3D = GridMapND<ObstacleMapType, 3>;
    void print();
};

}  // namespace decision_lib
}  // namespace jarvis
#endif