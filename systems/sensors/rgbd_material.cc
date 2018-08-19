#include "drake/systems/sensors/rgbd_material.h"

namespace drake {
namespace systems {
namespace sensors {

void RgbdMaterial::AddParameter(const std::string& name, double value) {
  IsUniqueOrThrow(name);
  scalars_[name] = value;
}

void RgbdMaterial::AddParameter(const std::string& name,
                                const Vector3<double>& value) {
  IsUniqueOrThrow(name);
  vector3s_[name] = value;
}

void RgbdMaterial::AddParameter(const std::string& name,
                                const Vector4<double>& value) {
  IsUniqueOrThrow(name);
  vector4s_[name] = value;
}

void RgbdMaterial::SetParameter(const std::string& name, double value) {
  if (vector3s_.count(name) == 0) IsUniqueOrThrow(name);
  scalars_[name] = value;
}

void RgbdMaterial::SetParameter(const std::string& name,
                                const Vector3<double>& value) {
  if (vector3s_.count(name) == 0) IsUniqueOrThrow(name);
  vector3s_[name] = value;
}

void RgbdMaterial::SetParameter(const std::string& name,
                                const Vector4<double>& value) {
  if (vector4s_.count(name) == 0) IsUniqueOrThrow(name);
  vector4s_[name] = value;
}

const double& RgbdMaterial::GetScalar(const std::string& name) const {
  return GetOrThrow(name, scalars_, "double");
}

const Vector3<double>& RgbdMaterial::GetVector3(const std::string& name) const {
  return GetOrThrow(name, vector3s_, "Vector3");
}

const Vector4<double>& RgbdMaterial::GetVector4(const std::string& name) const {
  return GetOrThrow(name, vector4s_, "Vector4");
}

void RgbdMaterial::Clear() {
  parameter_names_.clear();
  scalars_.clear();
  vector3s_.clear();
  vector4s_.clear();
}

bool RgbdMaterial::has_parameter(const std::string& name) const {
  return parameter_names_.count(name) > 0;
}

bool RgbdMaterial::has_scalar(const std::string& name) const {
  return scalars_.count(name) > 0;
}

bool RgbdMaterial::has_vector3(const std::string& name) const {
  return vector3s_.count(name) > 0;
}

bool RgbdMaterial::has_vector4(const std::string& name) const {
  return vector4s_.count(name) > 0;
}

void RgbdMaterial::IsUniqueOrThrow(const std::string& name) const {
  if (parameter_names_.count(name) > 0) {
    throw std::runtime_error("The parameter name " + name + " is not unique");
  }
}
}  // namespace sensors
}  // namespace systems
}  // namespace drake
