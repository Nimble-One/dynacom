#include <boost/python/module.hpp>
#include <eigenpy/eigenpy.hpp>
// #include <Eigen/Dense>

#include "dynacom/python.hpp"

typedef Eigen::Matrix<double, 6, 1, 0, 6, 1> eMatrix61;

BOOST_PYTHON_MODULE(dynacom) {
  // Enabling eigenpy support, i.e. numpy/eigen compatibility.
  eigenpy::enableEigenPy();
  ENABLE_SPECIFIC_MATRIX_TYPE(eMatrix61);
  ENABLE_SPECIFIC_MATRIX_TYPE(Eigen::MatrixXd);
  ENABLE_SPECIFIC_MATRIX_TYPE(Eigen::VectorXd);
  // ENABLE_SPECIFIC_MATRIX_TYPE(Eigen::Isometry3d);
  dynacom::python::exposeContact6D();
  dynacom::python::exposeDynaCoM();
}
