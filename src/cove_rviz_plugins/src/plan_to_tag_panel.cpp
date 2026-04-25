#include "cove_rviz_plugins/plan_to_tag_panel.hpp"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace cove_rviz_plugins
{

PlanToTagPanel::PlanToTagPanel(QWidget * parent)
: rviz_common::Panel(parent),
  status_label_(new QLabel("Ready")),
  capture_topic_edit_(new QLineEdit("/apriltag/capture_tag")),
  plan_topic_edit_(new QLineEdit("/apriltag/plan_captured_tag")),
  execute_topic_edit_(new QLineEdit("/apriltag/execute_captured_tag")),
  capture_button_(new QPushButton("Capture Tag")),
  plan_button_(new QPushButton("Plan Captured")),
  execute_button_(new QPushButton("Execute Captured"))
{
  auto * topics_layout = new QFormLayout;
  topics_layout->addRow("Capture topic", capture_topic_edit_);
  topics_layout->addRow("Plan topic", plan_topic_edit_);
  topics_layout->addRow("Execute topic", execute_topic_edit_);

  auto * buttons_layout = new QHBoxLayout;
  buttons_layout->addWidget(capture_button_);
  buttons_layout->addWidget(plan_button_);
  buttons_layout->addWidget(execute_button_);

  auto * layout = new QVBoxLayout;
  layout->addLayout(topics_layout);
  layout->addLayout(buttons_layout);
  layout->addWidget(status_label_);
  setLayout(layout);

  connect(capture_button_, SIGNAL(clicked()), this, SLOT(captureTag()));
  connect(plan_button_, SIGNAL(clicked()), this, SLOT(planCaptured()));
  connect(execute_button_, SIGNAL(clicked()), this, SLOT(executeCaptured()));
  connect(capture_topic_edit_, SIGNAL(editingFinished()), this, SLOT(updateTopics()));
  connect(plan_topic_edit_, SIGNAL(editingFinished()), this, SLOT(updateTopics()));
  connect(execute_topic_edit_, SIGNAL(editingFinished()), this, SLOT(updateTopics()));
}

void PlanToTagPanel::onInitialize()
{
  node_ = rclcpp::Node::make_shared("plan_to_tag_rviz_panel");
  createPublishers();
}

void PlanToTagPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Capture Topic", capture_topic_edit_->text());
  config.mapSetValue("Plan Topic", plan_topic_edit_->text());
  config.mapSetValue("Execute Topic", execute_topic_edit_->text());
}

void PlanToTagPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  QString capture_topic;
  if (config.mapGetString("Capture Topic", &capture_topic)) {
    capture_topic_edit_->setText(capture_topic);
  }

  QString plan_topic;
  if (config.mapGetString("Plan Topic", &plan_topic)) {
    plan_topic_edit_->setText(plan_topic);
  }

  QString execute_topic;
  if (config.mapGetString("Execute Topic", &execute_topic)) {
    execute_topic_edit_->setText(execute_topic);
  }

  if (node_) {
    createPublishers();
  }
}

void PlanToTagPanel::captureTag()
{
  publishEmpty(capture_pub_);
  setStatus("Capture requested");
}

void PlanToTagPanel::planCaptured()
{
  publishEmpty(plan_pub_);
  setStatus("Plan requested");
}

void PlanToTagPanel::executeCaptured()
{
  publishEmpty(execute_pub_);
  setStatus("Execute requested");
}

void PlanToTagPanel::updateTopics()
{
  createPublishers();
  setStatus("Topics updated");
}

void PlanToTagPanel::publishEmpty(
  const rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr & publisher)
{
  if (!publisher) {
    setStatus("Publisher not ready");
    return;
  }

  publisher->publish(std_msgs::msg::Empty{});
}

void PlanToTagPanel::setStatus(const QString & status)
{
  status_label_->setText(status);
}

void PlanToTagPanel::createPublishers()
{
  if (!node_) {
    return;
  }

  capture_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
    capture_topic_edit_->text().toStdString(), 10);
  plan_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
    plan_topic_edit_->text().toStdString(), 10);
  execute_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
    execute_topic_edit_->text().toStdString(), 10);
}

}  // namespace cove_rviz_plugins

PLUGINLIB_EXPORT_CLASS(cove_rviz_plugins::PlanToTagPanel, rviz_common::Panel)
