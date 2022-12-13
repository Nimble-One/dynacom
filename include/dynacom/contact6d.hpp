/**
 * @file
 * @authors Nahuel Villa, Maximilien Naveau
 * @copyright Copyright (c) 2022 LAAS-CNRS, TOWARD, PAL_ROBOTICS
 *            License BSD-2
 *
 * @brief Class to perform inverse geometry on a biped robot.
 */

#ifndef DYNACOM_CONTACT_6D
#define DYNACOM_CONTACT_6D

// clang-format off
#include <Eigen/Dense>
#include "pinocchio/spatial/se3.hpp"
#include <memory>
// clang-format on

namespace dynacom {

class ContactBase {
 protected:
  pinocchio::SE3 oMs_, cMo_;

  Eigen::VectorXd contactForce_;
  size_t frameID_;
  // matrices
  Eigen::MatrixXd unilaterality_A_;
  Eigen::VectorXd unilaterality_b_;
  Eigen::MatrixXd friction_A_;
  Eigen::VectorXd friction_b_;
  Eigen::VectorXd regularization_A_;
  Eigen::VectorXd regularization_b_;
  Eigen::Matrix<double, 6, -1> newton_euler_A_;
  
 public:
  ContactBase() {}
  // virtual ~Contact() {}

  // setters
  void setPose(pinocchio::SE3 &pose) { oMs_ = pose; }
  void setFrameID(const size_t frameID) { frameID_ = frameID; }
  void applyForce(const Eigen::MatrixXd &force) {
    contactForce_ << force;
  }
  void deactivate() { contactForce_.setZero(); }

  virtual void updateNewtonEuler(const Eigen::Vector3d &CoM, 
                                 const pinocchio::SE3 &oMf) = 0;
    
  // getters
  const pinocchio::SE3 &getPose() const { return oMs_; }
  size_t getFrameID() const { return frameID_; }
  size_t uni_rows() const { return unilaterality_A_.rows(); }
  size_t fri_rows() const { return friction_A_.rows(); }
  size_t cols() const { return newton_euler_A_.cols(); }
  const Eigen::VectorXd &appliedForce() { return contactForce_; }

  const Eigen::MatrixXd &uni_A() { return unilaterality_A_; }
  const Eigen::VectorXd &uni_b() { return unilaterality_b_; }
  const Eigen::MatrixXd &fri_A() { return friction_A_; }
  const Eigen::VectorXd &fri_b() { return friction_b_; }
  const Eigen::VectorXd &reg_A() { return regularization_A_; }
  const Eigen::VectorXd &reg_b() { return regularization_b_; }
  const Eigen::Matrix<double, 6, -1> &NE_A() { return newton_euler_A_; }

  const Eigen::Matrix<double, 6, 6> toWorldForces() {
    return oMs_.toActionMatrixInverse().transpose();
  }
  const Eigen::Matrix<double, 6, 6> toCoMForces() {
    return cMo_.act(oMs_).toActionMatrixInverse().transpose();
  }

  virtual std::string &getFrameName() = 0;
};

struct Contact6DSettings {
 public:
  double mu, gu;
  double half_length, half_width;
  Eigen::Matrix<double, 6, 1> weights;
  std::string frame_name;

  std::string to_string() {
    std::ostringstream out;
    out << "Contact6D "
        << ":\n";
    out << "    mu: " << this->mu << "\n";
    out << "    gu: " << this->gu << "\n";
    out << "    weights: " << this->weights.transpose() << "\n";
    out << "    Surface half_length: " << this->half_length << "\n";
    out << "    Surface half_width: " << this->half_width << std::endl;

    return out.str();
  }

  std::ostream &operator<<(std::ostream &out) {
    out << this->to_string();
    return out;
  }

  bool operator!=(const Contact6DSettings &rhs) { return !(*this == rhs); }

  bool operator==(const Contact6DSettings &rhs) {
    bool test = true;
    test &= this->frame_name == rhs.frame_name;
    test &= this->mu == rhs.mu;
    test &= this->gu == rhs.gu;
    test &= this->half_length == rhs.half_length;
    test &= this->half_width == rhs.half_width;
    test &= this->weights == rhs.weights;
    return test;
  }
};

class Contact6D : public ContactBase{
 private:
  Contact6DSettings settings_;

 public:
  Contact6D();
  Contact6D(const Contact6DSettings &settings);
  void initialize(const Contact6DSettings &settings);

  // setters
  void setMu(const double &mu);
  void setGu(const double &gu);
  void setForceWeights(const Eigen::Vector3d &force_weights);
  void setTorqueWeights(const Eigen::Vector3d &torque_weights);
  void setSurfaceHalfWidth(const double &half_width);
  void setSurfaceHalfLength(const double &half_length);
  void updateNewtonEuler(const Eigen::Vector3d &CoM, const pinocchio::SE3 &oMf);

  // getters
  const Contact6DSettings &getSettings() { return settings_; }
  std::string &getFrameName() { return settings_.frame_name; }
};

struct ContactPointSettings {
 public:
  double mu;
  Eigen::Matrix<double, 3, 1> weights;
  std::string frame_name;

  std::string to_string() {
    std::ostringstream out;
    out << "Contact6D "
        << ":\n";
    out << "    mu: " << this->mu << "\n";
    out << "    weights: " << this->weights.transpose() << std::endl;

    return out.str();
  }

  std::ostream &operator<<(std::ostream &out) {
    out << this->to_string();
    return out;
  }

  bool operator==(const ContactPointSettings &rhs) {
    bool test = true;
    test &= this->frame_name == rhs.frame_name;
    test &= this->mu == rhs.mu;
    test &= this->weights == rhs.weights;
    return test;
  }

  bool operator!=(const ContactPointSettings &rhs) { return !(*this == rhs); }
};

class ContactPoint : public ContactBase {
 private:
  ContactPointSettings settings_;

 public:
  ContactPoint();
  ContactPoint(const ContactPointSettings &settings);
  void initialize(const ContactPointSettings &settings);

  // setters
  void setMu(const double &mu);
  void setForceWeights(const Eigen::Vector3d &force_weights);
  void updateNewtonEuler(const Eigen::Vector3d &CoM, const pinocchio::SE3 &oMf);

  // getters
  const ContactPointSettings &getSettings() { return settings_; }
  std::string &getFrameName() { return settings_.frame_name; }
};

}  // namespace dynacom

#endif  // DYNACOM_CONTACT_6D
