#pragma once

#include <ros/ros.h>

#include <nodelet/nodelet.h>

namespace cepton_ros {

class DriverNodelet : public nodelet::Nodelet {
protected:
   void onInit() override;
};
}
