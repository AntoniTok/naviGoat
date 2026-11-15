// A Nav2 costmap layer that puts the people around the robot on the costmap, with social rules between them.
//
// People come from people_node.py on /people. At every costmap update, this layer draws three rules:
//   1. personCost(): each person is an obstacle, with a personal space around them that fades out, and reaches
//      further ahead of someone walking.
//   2. groupCost(): two people closer than group_distance are together, and the strip between them is expensive,
//      so the robot goes around them when it can.
//   3. predictionCost(): a strip along where each walker will be in the next prediction_time seconds.
//
// Only the people themselves are lethal (254). The other costs stay below 253: they make a path more expensive but
// never block it. That is how "avoid it if possible" is written in a costmap.
//
// The parameters are read again at every update, so they can be changed while running:
//   ros2 param set /global_costmap/global_costmap social_layer.group_cost 0
//
// To add a rule: write a function like groupCost() that gives the cost at one point, and paint() it in updateCosts().

// how a costmap works: it's a grid of cells (5 cm each for us), and each cell has a cost from 0 (free) to
// 254 (lethal = an obstacle). Nav2's planner looks for the cheapest path through it. the costmap is made of layers
// stacked on top of each other: the map, the obstacles, our people, then the inflation around everything.
// every layer gets two calls per update: updateBounds() ("which part of the map do you want to change?") and then
// updateCosts() ("ok, draw it").

#include <algorithm>  // std::min, std::max, std::clamp
#include <cmath>  // std::hypot (a length from x and y), std::atan2, std::cos, std::sin
#include <functional>  // std::function, so paint() can take any cost function
#include <mutex>  // std::mutex, because two threads touch latest_
#include <string>
#include <utility>  // std::pair
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"  // the special costs: LETHAL_OBSTACLE (254), NO_INFORMATION (255)
#include "nav2_costmap_2d/layer.hpp"  // the base class every costmap layer comes from
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "navigoat_msgs/msg/people.hpp"  // our own People message
#include "pluginlib/class_list_macros.hpp"  // for PLUGINLIB_EXPORT_CLASS at the bottom
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace social_layer
{

// one person. the global costmap is in the map frame and the local one in the odom frame, so we always convert the
// people into whichever frame the costmap uses (see currentPeople)
struct Person
{
  double x, y, vx, vy;  // in the costmap's frame
};

// A box in metres that grows to hold whatever is added to it.
// we use it to tell Nav2 which part of the costmap we're changing, so it only redraws that part
struct Box
{
  // it starts "inside out" (min bigger than max), which means empty until something gets added
  double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
  void add(double x0, double y0, double x1, double y1)
  {
    min_x = std::min(min_x, x0); min_y = std::min(min_y, y0);
    max_x = std::max(max_x, x1); max_y = std::max(max_y, y1);
  }
  bool empty() const {return min_x > max_x;}
};

// our layer. Nav2 creates it from the plugin name in the yaml config ("social_layer::SocialLayer")
class SocialLayer : public nav2_costmap_2d::Layer
{
public:
  // called once, when the costmap starts
  void onInitialize() override
  {
    // The parameters and their defaults. The costs are whole numbers, 0 to 252.
    // (in ROS they get the layer's name in front: e.g. "social_layer.group_cost" on the costmap node)
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("people_topic", rclcpp::ParameterValue(std::string("/people")));
    declareParameter("people_timeout", rclcpp::ParameterValue(1.0));   // s without /people before everyone is cleared
    declareParameter("ignore_radius", rclcpp::ParameterValue(0.6));    // m: people this close to the robot are left out
    declareParameter("person_radius", rclcpp::ParameterValue(0.3));    // m: the person, lethal
    declareParameter("personal_space", rclcpp::ParameterValue(1.0));   // m: where their cost has faded to 0
    declareParameter("personal_space_cost", rclcpp::ParameterValue(150));
    declareParameter("motion_stretch", rclcpp::ParameterValue(1.0));   // m more space ahead of a walker, per m/s
    declareParameter("min_speed", rclcpp::ParameterValue(0.25));       // m/s: anyone slower is standing
    declareParameter("group_distance", rclcpp::ParameterValue(2.2));   // m: closer than this, two people are together
    declareParameter("group_width", rclcpp::ParameterValue(0.6));      // m: width of the strip between them
    declareParameter("group_cost", rclcpp::ParameterValue(220));
    declareParameter("prediction_time", rclcpp::ParameterValue(2.0));  // s ahead a walker's path is predicted
    declareParameter("prediction_width", rclcpp::ParameterValue(1.0)); // m: width of that path
    declareParameter("prediction_cost", rclcpp::ParameterValue(200));
    readParameters();

    // subscribe to /people. the callback only keeps the latest message: the drawing happens later, in updateCosts
    auto node = node_.lock();  // node_ is the costmap's node. it's a "weak" pointer, lock() gives a usable one
    std::string topic = node->get_parameter(name_ + ".people_topic").as_string();
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;  // handled by the costmap's own thread
    sub_ = node->create_subscription<navigoat_msgs::msg::People>(
      topic, 10, [this](navigoat_msgs::msg::People::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);  // locked, so updateBounds can't read it at the same moment
        latest_ = msg;
      }, options);
    current_ = true;  // tells Nav2 this layer is ready
    RCLCPP_INFO(logger_, "%s: people from %s", name_.c_str(), topic.c_str());
  }

  // Say which part of the costmap to update: where people are now, and where they were last time, so the costs
  // they left behind are cleared.
  // (Nav2 calls this for every layer at every update: once a second for the global costmap, 5 times for the local)
  void updateBounds(
    double robot_x, double robot_y, double /*robot_yaw*/,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    readParameters();  // picks up any `ros2 param set` done since last time
    people_ = currentPeople(robot_x, robot_y);
    // find the groups: every pair of people closer than group_distance
    groups_.clear();
    for (size_t a = 0; a < people_.size(); ++a) {
      for (size_t b = a + 1; b < people_.size(); ++b) {  // b starts after a, so each pair is checked only once
        if (std::hypot(people_[a].x - people_[b].x, people_[a].y - people_[b].y) <= group_distance_) {
          groups_.push_back({people_[a], people_[b]});
        }
      }
    }

    // the box around everything we're going to draw: each person's space, and where the walkers are going.
    // (the strips between groups are inside the people's boxes already, so they don't need adding)
    Box now;
    for (const Person & p : people_) {
      double r = reach(p);
      now.add(p.x - r, p.y - r, p.x + r, p.y + r);
      Box ahead = predictionBox(p);
      if (!ahead.empty()) {
        now.add(ahead.min_x, ahead.min_y, ahead.max_x, ahead.max_y);
      }
    }
    // make Nav2's update area big enough for both boxes. min_x... are pointers so we can change Nav2's values
    for (const Box & box : {last_, now}) {
      if (!box.empty()) {
        *min_x = std::min(*min_x, box.min_x); *min_y = std::min(*min_y, box.min_y);
        *max_x = std::max(*max_x, box.max_x); *max_y = std::max(*max_y, box.max_y);
      }
    }
    last_ = now;  // remember it for the next update
  }

  // draw the three rules into the costmap (`grid`). min_i..max_j is the part being updated, counted in cells
  void updateCosts(nav2_costmap_2d::Costmap2D & grid, int min_i, int min_j, int max_i, int max_j) override
  {
    // rule 1, for every person. the [&](...) {...} bit is a tiny function given to paint(): the cost at (x, y)
    for (const Person & p : people_) {
      double r = reach(p);
      paint(grid, min_i, min_j, max_i, max_j, p.x - r, p.y - r, p.x + r, p.y + r,
        [&](double x, double y) {return personCost(p, x, y);});
    }
    // rule 2, for every group of two
    for (const auto & group : groups_) {
      const Person & a = group.first;
      const Person & b = group.second;
      double w = group_width_ / 2.0;
      paint(grid, min_i, min_j, max_i, max_j,
        std::min(a.x, b.x) - w, std::min(a.y, b.y) - w, std::max(a.x, b.x) + w, std::max(a.y, b.y) + w,
        [&](double x, double y) {return groupCost(a, b, x, y);});
    }
    // rule 3, for everyone walking (standing people have an empty box)
    for (const Person & p : people_) {
      Box ahead = predictionBox(p);
      if (!ahead.empty()) {
        paint(grid, min_i, min_j, max_i, max_j, ahead.min_x, ahead.min_y, ahead.max_x, ahead.max_y,
          [&](double x, double y) {return predictionCost(p, x, y);});
      }
    }
  }

  // Nav2 needs these two. we don't keep a grid of our own, so there's nothing to reset or clear
  void reset() override {}
  bool isClearable() override {return false;}

private:
  // Rule 1: the person and their personal space. Lethal inside person_radius, then fading to 0 at personal_space,
  // which reaches motion_stretch metres further per m/s in the direction someone walks.
  double personCost(const Person & p, double x, double y) const
  {
    double dx = x - p.x, dy = y - p.y;  // from the person to the cell
    double d = std::hypot(dx, dy);  // the distance between them
    if (d <= person_radius_) {
      return nav2_costmap_2d::LETHAL_OBSTACLE;  // inside the person: same as a wall
    }
    double space = personal_space_;
    double speed = std::hypot(p.vx, p.vy);
    if (speed >= min_speed_) {
      // someone walking: their space is stretched out in front of them
      double ahead = (dx * p.vx + dy * p.vy) / (d * speed);  // 1 straight ahead of them, 0 beside, < 0 behind
      space += motion_stretch_ * speed * std::max(0.0, ahead);
    }
    if (d >= space) {
      return 0.0;  // outside their space: free
    }
    // in between: f goes from 1 at the person's edge to 0 at the edge of their space. squaring it makes the cost
    // drop fast near the person and fade gently further out
    double f = 1.0 - (d - person_radius_) / (space - person_radius_);
    return personal_space_cost_ * f * f;
  }

  // Rule 2: don't pass between two people who are together. The strip of group_width between them costs group_cost.
  double groupCost(const Person & a, const Person & b, double x, double y) const
  {
    double sx = b.x - a.x, sy = b.y - a.y;  // the line from person a to person b
    double length2 = sx * sx + sy * sy;  // its length squared
    // t = how far along that line the cell is: 0 at a, 1 at b (clamped, so it stays between the two of them)
    double t = length2 > 0.0 ? std::clamp(((x - a.x) * sx + (y - a.y) * sy) / length2, 0.0, 1.0) : 0.0;
    double d = std::hypot(x - (a.x + t * sx), y - (a.y + t * sy));  // distance to the segment between them
    return d <= group_width_ / 2.0 ? group_cost_ : 0.0;  // inside the strip: expensive, outside: nothing
  }

  // Rule 3: where a walker is going. The strip of prediction_width along their velocity, as far as they walk in
  // prediction_time: prediction_cost right in front of them, fading to 0 where they will be at the end of it.
  double predictionCost(const Person & p, double x, double y) const
  {
    double speed = std::hypot(p.vx, p.vy);
    double ahead = ((x - p.x) * p.vx + (y - p.y) * p.vy) / speed;  // metres along their path
    double aside = std::abs((y - p.y) * p.vx - (x - p.x) * p.vy) / speed;  // metres beside it
    double length = speed * prediction_time_;  // how far they walk in prediction_time
    if (ahead < 0.0 || ahead > length || aside > prediction_width_ / 2.0) {
      return 0.0;  // behind them, past where they'll be, or too far to the side
    }
    return prediction_cost_ * (1.0 - ahead / length);  // highest right in front of them, 0 at the end
  }

  // The box around a walker's predicted path. Empty for someone standing, or with the prediction switched off.
  Box predictionBox(const Person & p) const
  {
    Box box;
    double speed = std::hypot(p.vx, p.vy);
    if (speed > 0.0 && speed >= min_speed_ && prediction_time_ > 0.0) {
      double w = prediction_width_ / 2.0;
      double end_x = p.x + p.vx * prediction_time_, end_y = p.y + p.vy * prediction_time_;  // where they'll be
      box.add(std::min(p.x, end_x) - w, std::min(p.y, end_y) - w, std::max(p.x, end_x) + w, std::max(p.y, end_y) + w);
    }
    return box;
  }

  // How far from a person their personal space can reach.
  double reach(const Person & p) const
  {
    double speed = std::hypot(p.vx, p.vy);
    return std::max(person_radius_, personal_space_ + (speed >= min_speed_ ? motion_stretch_ * speed : 0.0));
  }

  // Write cost(x, y) into each cell of the box [x0, x1] x [y0, y1] that is inside the part of the costmap being
  // updated, keeping whichever cost is higher. An unknown cell takes our cost, so a person standing somewhere not
  // mapped yet still counts.
  void paint(
    nav2_costmap_2d::Costmap2D & grid, int min_i, int min_j, int max_i, int max_j,
    double x0, double y0, double x1, double y1, std::function<double(double, double)> cost) const
  {
    // the box's corners, in cells ("EnforceBounds" = cut off at the edges of the costmap)
    int i0, j0, i1, j1;
    grid.worldToMapEnforceBounds(x0, y0, i0, j0);
    grid.worldToMapEnforceBounds(x1, y1, i1, j1);
    // every cell of the box that is also inside the area Nav2 is updating (max_i and max_j themselves are not in it)
    for (int j = std::max(j0, min_j); j <= std::min(j1, max_j - 1); ++j) {
      for (int i = std::max(i0, min_i); i <= std::min(i1, max_i - 1); ++i) {
        double x, y;
        grid.mapToWorld(i, j, x, y);  // the middle of the cell, in metres
        double c = std::min(cost(x, y), 254.0);  // 254 is the highest real cost there is (lethal)
        unsigned char old = grid.getCost(i, j);  // what the layers before us already put there
        // only write if our cost is higher, so we never erase a wall from the map
        if (c >= 1.0 && (old == nav2_costmap_2d::NO_INFORMATION || old < c)) {
          grid.setCost(i, j, static_cast<unsigned char>(c));
        }
      }
    }
  }

  // The latest people, in the costmap's frame. Nobody if the layer is off or /people stopped coming. People within
  // ignore_radius of the robot are left out: that is the person holding the handle, and a lethal cell on the robot
  // would stop the planner from finding any path.
  std::vector<Person> currentPeople(double robot_x, double robot_y)
  {
    std::vector<Person> people;
    // take the latest message while holding the lock, so the subscription can't swap it at the same moment
    navigoat_msgs::msg::People::ConstSharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg = latest_;
    }
    // layer switched off, no message yet, or the message is too old (people_node stopped): nobody
    if (!enabled_ || !msg || (clock_->now() - rclcpp::Time(msg->header.stamp, RCL_ROS_TIME)).seconds() > people_timeout_) {
      return people;
    }

    // the people are in the map frame, but the local costmap is in odom: ask TF for the transform between the two
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_->lookupTransform(layered_costmap_->getGlobalFrameID(), msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "%s: cannot place people: %s", name_.c_str(), e.what());
      return people;
    }
    // on flat ground the transform is just a rotation by yaw, then a shift by the translation
    const auto & q = t.transform.rotation;
    double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    double c = std::cos(yaw), s = std::sin(yaw);
    for (const auto & p : msg->people) {
      // the position gets rotated and shifted, the velocity only rotated (a speed doesn't move with the frame)
      Person out{
        c * p.position.x - s * p.position.y + t.transform.translation.x,
        s * p.position.x + c * p.position.y + t.transform.translation.y,
        c * p.velocity.x - s * p.velocity.y,
        s * p.velocity.x + c * p.velocity.y};
      if (std::hypot(out.x - robot_x, out.y - robot_y) > ignore_radius_) {
        people.push_back(out);
      }
    }
    return people;
  }

  // Read the parameters again, so a change made while running is used from the next update.
  void readParameters()
  {
    auto node = node_.lock();
    // a small helper, so we don't have to write node->get_parameter(name_ + "." + ...) every single time
    auto get = [&](const std::string & name) {return node->get_parameter(name_ + "." + name);};
    enabled_ = get("enabled").as_bool();
    people_timeout_ = get("people_timeout").as_double();
    ignore_radius_ = get("ignore_radius").as_double();
    person_radius_ = get("person_radius").as_double();
    personal_space_ = get("personal_space").as_double();
    // the costs are kept between 0 and 252 (clamp), so a typo can never turn them into walls
    personal_space_cost_ = std::clamp<int64_t>(get("personal_space_cost").as_int(), 0, 252);
    motion_stretch_ = get("motion_stretch").as_double();
    min_speed_ = get("min_speed").as_double();
    group_distance_ = get("group_distance").as_double();
    group_width_ = get("group_width").as_double();
    group_cost_ = std::clamp<int64_t>(get("group_cost").as_int(), 0, 252);
    prediction_time_ = get("prediction_time").as_double();
    prediction_width_ = get("prediction_width").as_double();
    prediction_cost_ = std::clamp<int64_t>(get("prediction_cost").as_int(), 0, 252);
  }

  // the parameters, filled in by readParameters() (the _ at the end is how we name the class's own variables)
  double people_timeout_, ignore_radius_, person_radius_, personal_space_, personal_space_cost_, motion_stretch_;
  double min_speed_, group_distance_, group_width_, group_cost_, prediction_time_, prediction_width_, prediction_cost_;

  rclcpp::Subscription<navigoat_msgs::msg::People>::SharedPtr sub_;  // the /people subscription
  std::mutex mutex_;  // latest_ is written by the subscription and read by the costmap updates
  navigoat_msgs::msg::People::ConstSharedPtr latest_;
  std::vector<Person> people_;  // found in updateBounds, drawn in updateCosts
  std::vector<std::pair<Person, Person>> groups_;
  Box last_;  // where the people were drawn last time
};

}  // namespace social_layer

// this line is what lets Nav2 load our class as a plugin
PLUGINLIB_EXPORT_CLASS(social_layer::SocialLayer, nav2_costmap_2d::Layer)
