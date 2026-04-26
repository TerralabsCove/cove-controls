#pragma once

#include <memory>

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/empty.hpp>

namespace cove_rviz_plugins
{

class PlanToTagPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit PlanToTagPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void captureTag();
  void planCaptured();
  void updateTopics();

private:
  void publishEmpty(const rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr & publisher);
  void setStatus(const QString & status);
  void createPublishers();

  QLabel * status_label_;
  QLineEdit * capture_topic_edit_;
  QLineEdit * plan_topic_edit_;
  QPushButton * capture_button_;
  QPushButton * plan_button_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr capture_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr plan_pub_;
};

}  // namespace cove_rviz_plugins
