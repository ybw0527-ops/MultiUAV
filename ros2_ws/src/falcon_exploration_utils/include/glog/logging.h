#ifndef FALCON_EXPLORATION_UTILS_GLOG_COMPAT_LOGGING_H_
#define FALCON_EXPLORATION_UTILS_GLOG_COMPAT_LOGGING_H_

// FALCON vendors an older Kindr snapshot that only needs glog's CHECK macros.
// Keeping this small compatibility header makes the ROS 2 port self-contained.

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>

namespace falcon_glog_compat {

class CheckFailure {
public:
  CheckFailure(const char *expression, const char *file, int line) {
    stream_ << file << ':' << line << ": check failed: " << expression << ' ';
  }

  ~CheckFailure() {
    std::cerr << stream_.str() << std::endl;
    std::abort();
  }

  std::ostream &stream() { return stream_; }

private:
  std::ostringstream stream_;
};

class NullStream {
public:
  template <typename T>
  NullStream &operator<<(const T &) { return *this; }
};

}  // namespace falcon_glog_compat

#define CHECK(condition)                                                                \
  if (condition) {                                                                      \
  } else                                                                                \
    ::falcon_glog_compat::CheckFailure(#condition, __FILE__, __LINE__).stream()

#define CHECK_NOTNULL(value) CHECK((value) != nullptr)
#define CHECK_EQ(lhs, rhs) CHECK((lhs) == (rhs))
#define CHECK_NE(lhs, rhs) CHECK((lhs) != (rhs))
#define CHECK_LT(lhs, rhs) CHECK((lhs) < (rhs))
#define CHECK_LE(lhs, rhs) CHECK((lhs) <= (rhs))
#define CHECK_GT(lhs, rhs) CHECK((lhs) > (rhs))
#define CHECK_GE(lhs, rhs) CHECK((lhs) >= (rhs))
#define CHECK_NEAR(lhs, rhs, tolerance)                                                  \
  CHECK(std::abs((lhs) - (rhs)) <= (tolerance))
#define VLOG(level) ::falcon_glog_compat::NullStream()

#endif  // FALCON_EXPLORATION_UTILS_GLOG_COMPAT_LOGGING_H_
