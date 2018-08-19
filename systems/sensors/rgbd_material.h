#pragma once

#include <unordered_map>
#include <unordered_set>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {
namespace sensors {

class RgbdMaterial {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RgbdMaterial);

  RgbdMaterial() = default;

  /** @name   Adding new parameters

   These methods add previously undefined parameters to the material. Parameter
   names must be globally unique (regardless of parameter type). If a previous
   parameter has been provided with the given name, an exception is thrown.   */
  //@{
  void AddParameter(const std::string& name, double value);
  void AddParameter(const std::string& name, const Vector3<double>& value);
  void AddParameter(const std::string& name, const Vector4<double>& value);
  //@}

  /** @name   Set parameter values

   These methods can be used to set/overwrite parameters. If a parameter had
   been previously set, this will overwrite that parameter, if and only if the
   type is the same. If the type changes, an exception will be thrown.  */
  //@{
  void SetParameter(const std::string& name, double value);
  void SetParameter(const std::string& name, const Vector3<double>& value);
  void SetParameter(const std::string& name, const Vector4<double>& value);
  //@}

  /** @name  Retrieves parameter values

   Each parameter type has its own, unique getter. If the scalar type has no
   parameter of the given name, an exception is thrown.   */
  //@{
  const double& GetScalar(const std::string& name) const;
  const Vector3<double>& GetVector3(const std::string& name) const;
  const Vector4<double>& GetVector4(const std::string& name) const;
  //@}

  /** Clears the material parameters. */
  void Clear();

  /** @name  Test for parameter existence

   Reports if the given material has a parameter of the given name and, based
   on the method variant, if it is of a particular type.   */
  //@{
  bool has_parameter(const std::string& name) const;
  bool has_scalar(const std::string& name) const;
  bool has_vector3(const std::string& name) const;
  bool has_vector4(const std::string& name) const;
  //@}

 private:
  void IsUniqueOrThrow(const std::string& name) const;

  template <typename T>
  const T& GetOrThrow(const std::string& name,
                      const std::unordered_map<std::string, T>& map,
                      const char* type_name) const {
    auto iterator = map.find(name);
    if (iterator != map.end()) return iterator->second;
    throw std::runtime_error("Can't get parameter with name " + name +
                             " of type " + std::string(type_name));
  }

  std::unordered_set<std::string> parameter_names_;
  std::unordered_map<std::string, double> scalars_;
  std::unordered_map<std::string, Vector3<double>> vector3s_;
  std::unordered_map<std::string, Vector4<double>> vector4s_;
};
}  // namespace sensors
}  // namespace systems
}  // namespace drake
