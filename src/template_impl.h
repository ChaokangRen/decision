#pragma once

#include <Eigen/Core>

#include "template/decision_interface.h"

namespace jarvis {
namespace decision_lib {

class DecisionImpl : public DecisionInterface {
public:
    DecisionImpl(std::string create_param);
    virtual ~DecisionImpl();

public:
    std::string get_version() override;

    bool init() override;
    std::string execute() override;

private:
    std::string priv_member_;
};

}  // namespace decision_lib
}  // namespace jarvis
