#include <sglog/sglog.h>
#include <sgtime/sgtime.h>

#include <sstream>

#include "ssc_map.h"
#include "template_impl.h"

namespace jarvis {
namespace decision_lib {

DecisionInterface *DecisionInterface::create_instance(
    std::string create_param) {
    return new DecisionImpl(create_param);
}

DecisionImpl::DecisionImpl(std::string create_param)
    : priv_member_(create_param) {
    SG_INFO("DecisionImpl construct");
}

DecisionImpl::~DecisionImpl() {
    SG_INFO("DecisionImpl destruct");
}

std::string DecisionImpl::get_version() {
#ifdef PKG_VERSION
    return PKG_VERSION;
#else
    return "UNKNOW";
#endif
}

bool DecisionImpl::init() {
    SG_INFO("DecisionImpl init");
    return true;
}

std::string DecisionImpl::execute() {
    std::stringstream ss;
    ss << "gigity execute ";
    SscMap ssc_map;
    ssc_map.print();

    return ss.str();
}
}  // namespace decision_lib
}  // namespace jarvis
