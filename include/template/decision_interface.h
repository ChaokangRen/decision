#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <string>

namespace jarvis {
namespace decision_lib {

class DecisionInterface {
public:
    virtual ~DecisionInterface(){};

    static DecisionInterface *create_instance(std::string create_param);

    virtual std::string get_version() = 0;
    virtual bool init() = 0;
    virtual std::string execute() = 0;
};

}  // namespace decision_lib
}  // namespace jarvis