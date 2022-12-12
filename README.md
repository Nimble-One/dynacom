`build token` `unit test token`

# DYNACOM

[![Pipeline status](https://gitlab.laas.fr/gepetto/dynacom/badges/master/pipeline.svg)](https://gitlab.laas.fr/gepetto/dynacom/commits/master)
[![Coverage report](https://gitlab.laas.fr/gepetto/dynacom/badges/master/coverage.svg?job=doc-coverage)](https://gepettoweb.laas.fr/doc/gepetto/dynacom/master/coverage/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/gepetto/dynacom/master.svg)](https://results.pre-commit.ci/latest/github/gepetto/dynacom)

Computation of centroidal forces required for given centroidal motions, and optimal distribution of the 
forces among all the specified contacts between the robot and the environment.

DYNAmics of the Center of Mass:

- Given certain centroidal motion, it computes the centroidal wrench required to perform it.

- Given a centroidal wrench, it computes the optimal distribution of such wrench among all the specified contacts of the robot.

## 1/ :penguin: Installation

### 1.1/ Standard dependencies

This code relies essentially on the C++ std and on the Rigid Body Dynamics
implemented in
[Pinocchio](https://stack-of-tasks.github.io/pinocchio/).

For the unit-test it relies on the
[example-robot-data](https://github.com/Gepetto/example-robot-data)
package as well as the boost unit test framework.

In order to install those :package: from Debian / Ubuntu packages, you can use
with [robotpkg](http://robotpkg.openrobots.org)

1. If you have never added robotpkg's software repository,
   [do it now](http://robotpkg.openrobots.org/debian.html):

   ```bash
   sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
   deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg
   EOF

   curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
   sudo apt update
   ```

2. installation of pinocchio and example-robot-data and their python utils:
   ```bash
   sudo apt install robotpkg-py3\*-example-robot-data robotpkg-py3\*-pinocchio
   ```

### 1.2/ Build and install this package:

1. :turtle: With ROS

    Just clone it (with `--recursive`) into a catkin or a colcon workspace.

2. :file_folder: From source

    Clone it (with `--recursive`), create a `build` directory inside, and:
    ```bash
    cmake .. && make && make install
    ```

## 2/ Usage

### 2.1/ UnitTests/Examples

In this packages the unit-testsare based on the Talos robot from PAL Robotics.
Please read the unit-test to see how to use the code.

### 2.2/ API documentation

*How to build the API html doc.*
*Where to find the last built doc on the internet.*

## 3/ License and Copyrights

License BSD-2-Clause
Copyright (c) 2022, CNRS, Gepetto
