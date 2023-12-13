#ifndef _COMMON_BASICS_H_
#define _COMMON_BASICS_H_

#include <array>
#include <vector>
namespace jarvis {
namespace decision_lib {
enum ErrorType { kSuccess = 0, kWrongStatus, kIllegalInput, kUnknown };
using decimal_t = double;
}  // namespace decision_lib
}  // namespace jarvis
#endif