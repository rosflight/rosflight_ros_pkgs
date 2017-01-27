/**
 * \file serial_exception.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_SERIAL_EXCEPTION_H
#define MAVROSFLIGHT_SERIAL_EXCEPTION_H

#include <exception>
#include <string>
#include <sstream>

#include <boost/system/system_error.hpp>

namespace mavrosflight
{
/**
 * \brief Describes an exception encountered while using the boost serial libraries
 */
class SerialException : public std::exception
{
public:
  explicit SerialException(const char *const description)
  {
    init(description);
  }

  explicit SerialException(const std::string &description)
  {
    init(description.c_str());
  }

  explicit SerialException(const boost::system::system_error &err)
  {
    init(err.what());
  }

  SerialException(const SerialException &other) : what_(other.what_)
  {
  }

  ~SerialException() throw()
  {
  }

  virtual const char *what() const throw()
  {
    return what_.c_str();
  }

private:
  std::string what_;

  void init(const char *const description)
  {
    std::ostringstream ss;
    ss << "Serial Error: " << description;
    what_ = ss.str();
  }
};

}  // namespace mavrosflight

#endif  // MAVROSFLIGHT_SERIAL_EXCEPTION_H
