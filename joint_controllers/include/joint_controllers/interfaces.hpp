#ifndef INTERFACES_HPP_
#define INTERFACES_HPP_

#include <string>

namespace joint_controllers
{

struct Interfaces {
  int _value;
  std::string _name;

  Interfaces(const int &value, const std::string &name)
    : _value(value)
    , _name(name)
    {}

  std::string _to_string() const {
    return _name;
  }

  operator int() const { return _value;}

  static const Interfaces position;
  static const Interfaces velocity;
  static const Interfaces effort;
} ;
}

#endif