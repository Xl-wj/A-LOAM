#pragma once

#include <glog/logging.h>
#include <boost/geometry/geometry.hpp>
#include <string>
#include <vector>
#include <pcl/point_types.h>

struct XYZ {
    XYZ(double _x, double _y) :
            x(_x), y(_y) {}

    XYZ(double _x, double _y, double _z) :
            x(_x), y(_y), z(_z) {}

    double x;
    double y;
    double z;
};
typedef std::vector<XYZ> XYZs;

template <typename T>
class TaskFrameI {
public:
    using Type = T;
    using Ptr = std::shared_ptr<TaskFrameI>;
    using ConstPtr = std::shared_ptr<const TaskFrameI>;

    virtual ~TaskFrameI() = default;

    virtual void init(std::vector<T> inputs) = 0;

    void setBufferDistance(double distance) { buffer_distance_ = distance; };

    double getBufferDistance() const { return buffer_distance_; };

    virtual bool isInFrame(const T& t) const = 0;

    virtual bool isInBufferedFrame(const T& t) const = 0;

    const std::vector<T>& getInputs() const { return inputs; }

public:
    std::string task_area_id_ = "";

protected:
    double buffer_distance_ = 0.0;

    std::vector<T> inputs;
};

template <typename T>
class BoostTaskFrame : public TaskFrameI<T> {
public:
    void init(std::vector<T> inputs) override {
      polygon.outer().clear();
      for (const auto& t : inputs) polygon.outer().push_back(toPoint(t));
      boost::geometry::correct(polygon);
      this->inputs = std::move(inputs);
    };

    bool isInFrame(const T& t) const override {
      return boost::geometry::within(toPoint(t), polygon);
    };

    bool isInBufferedFrame(const T& t) const override {
      return isInFrame(t) ||
             boost::geometry::distance(toPoint(t), polygon) < this->buffer_distance_;
    };

protected:
    using Point =
    boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>;
    using Polygon = boost::geometry::model::polygon<Point>;

    virtual Point toPoint(const T& xy) const = 0;

    Polygon polygon;
};

class HDMapTaskFrame : public BoostTaskFrame<XYZ> {
public:
    using Ptr = std::shared_ptr<HDMapTaskFrame>;

protected:
    Point toPoint(const XYZ& xy) const override { return Point{xy.x, xy.y}; };
};


XYZs LoadHdMapRoadBoundary(const std::string &hdmap_polygen_file) {
  XYZs points;
  points.push_back(XYZ(68.02521514893, 45.56940460205));
  points.push_back(XYZ(1.461326904297E+2, 4.569599533081E+1));
  points.push_back(XYZ(1.772743988037E+2, 4.506303405762E+1));
  points.push_back(XYZ(1.860092773438E+2, 4.468325805664E+1));
  points.push_back(XYZ(1.909463806152E+2, 4.658214187622E+1));
  points.push_back(XYZ(2.113277435303E+2, 4.708851242065E+1));
  points.push_back(XYZ(2.175307769775E+2, 4.189822769165E+1));
  points.push_back(XYZ(2.160116577148E+2, 3.316335678101E+1));
  points.push_back(XYZ(2.124670715332E+2, 3.227721023560E+1));
  points.push_back(XYZ(1.363850860596E+2, 3.531542587280E+1));
  points.push_back(XYZ(6.941773223877E+1, 3.734090423584E+1));
  points.push_back(XYZ(6.802521514893E+1, 4.278437423706E+1));
//  points.push_back(XYZ(6.840499114990E+1, 4.569599533081E+1));
  return points;
}
