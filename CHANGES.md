# Cepton ROS Release Notes

## Version 1.13 2019-05-01
* Update sdk.

## Version 1.12 2019-03-18
* Update sdk.

## Version 1.11 2019-01-25
* Update sdk.

## Version 1.10.1 2018-12-19
* **MAJOR CHANGES**.
* Remove `driver_single.launch` and `driver_multi.launch`. Call `driver.launch` directly.
* All points output on single topic `cepton/points`.
* If `transforms_path` is provided during launch, point cloud tf frames are `cepton_<serial_number>`. Otherwise, they are `cepton_0`.
* Add `SubscriberNodelet` for sample subscriber code and debugging.
* Add more launch files for individual components. See `demo.launch`.
* Switch to using dynamic SDK library.

## Version 1.10 2018-12-10
* Update SDK.

## Version 1.9 2018-11-02
* Update sdk.
* Combine image and 3d point cloud outputs.

## Version 1.8 2018-10-08
* Updated SDK.

## Version 1.7 2018-10-02
* Updated sdk.

## Version 1.6 2018-09-04
* Updated SDK.
* Updated tests.

## Version 1.5 2018-08-10
* Updated sdk.

## Version 1.4 (2018-08-02)
* Updated sdk.
* Changed point types.

## Version 1.2 (2018-06-09)
* Migrated to SDK v1.2.

## Version 1.0 (2018-04-02)
* Migrated to SDK v1.0.
* Using SDK C++ interface.
* Added `tests` folder for testing launch files.

### Version 0.1 (2017-03-24)
* Initial point cloud publisher.
* Contributors: Jonathan Allen
