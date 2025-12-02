# kuka_iiwa_driver_crospi Package

This package contains a driver for controlling the Kuka iiwa using crospi_core, based on template_driver_crospi package which uses ROS2 plugins.

## Installation

Replace the *external/kuka_fri_sdk* with your own KUKA FRI SDK. We cannot distribute this SDK since it has a closed license and need to be distributed by KUKA. 

Then compile the SDK using cmake/make as instructed by KUKA.

Then compile the package using *colcon*.

## Documentation

For documentation and tutorials, checkout our [crospi website](https://crospi-website-907f83.pages.gitlab.kuleuven.be/).

## Authors

- Federico Ulloa <federico.ulloarios@kuleuven.be>
- Santiago Iregui <santiago.iregui@kuleuven.be>