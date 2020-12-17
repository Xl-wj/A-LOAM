#pragma once

#include <ros/ros.h>

#include <queue>
#include <mutex>
#include <string>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

template <typename T>
class MsgBufRegister {
    static_assert(std::is_same<T, sensor_msgs::PointCloud2>::value ||
                  std::is_same<T, sensor_msgs::Imu>::value ||
                  std::is_same<T, nav_msgs::Odometry>::value,
                  "The Buf type must be sensor_msgs::PointCloud2,"
                          " sensor_msgs::Imu or nav_msgs::Odometry");

public:
    typedef std::shared_ptr<MsgBufRegister> Ptr;

public:
    MsgBufRegister(ros::NodeHandle &nh, const std::string &topic,
                   uint32_t queue_size, const std::string &buf_name = "") {
      subscriber_ = nh.subscribe<T>(topic, queue_size, &MsgBufRegister::messageHandler, this);
    }

    void messageHandler(const typename T::ConstPtr &msg) {
      std::lock_guard<std::mutex> lock(mBuf_);
      buf_.push(*msg);
    }

    void clear(double time = std::numeric_limits<double >::max()) {
      std::lock_guard<std::mutex> lock(mBuf_);
      while(!buf_.empty() && buf_.front().header.stamp.toSec() < (time - 1e-5)) {
        buf_.pop();
      }
    }

    double frontTime() {
      std::lock_guard<std::mutex> lock(mBuf_);
      if(buf_.empty()) return -1.0;
      return buf_.front().header.stamp.toSec();
    }

    double endTime() {
      std::lock_guard<std::mutex> lock(mBuf_);
      if(buf_.empty()) return -1.0;
      return buf_.back().header.stamp.toSec();
    }

    size_t size() {
      std::lock_guard<std::mutex> lock(mBuf_);
      return buf_.size();
    }

    bool empty() {
      std::lock_guard<std::mutex> lock(mBuf_);
      return buf_.empty();
    }

    T frontMsg() {
      std::lock_guard<std::mutex> lock(mBuf_);
      return buf_.front();
    }

    T backMsg() {
      std::lock_guard<std::mutex> lock(mBuf_);
      return buf_.front();
    }
private:
    std::string buf_name_;

    ros::Subscriber subscriber_;
    std::mutex mBuf_;
    std::queue<T> buf_;
};

