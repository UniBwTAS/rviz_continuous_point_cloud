#ifndef LOW_LATENCY_MESSAGE_FILTER_DISPLAY_H
#define LOW_LATENCY_MESSAGE_FILTER_DISPLAY_H

#ifndef Q_MOC_RUN
#include "ros_transform_synchronizer.h"
#endif

#include <rviz/message_filter_display.h>

namespace rviz
{
/** @brief Helper superclass for LowLatencyMessageFilterDisplay, needed because
 * Qt's moc and c++ templates don't work nicely together.  Not
 * intended to be used directly. */
class RVIZ_EXPORT _LowLatencyMessageFilterDisplay : public _RosTopicDisplay
{
  Q_OBJECT
public:
  _LowLatencyMessageFilterDisplay()
  {
    wait_for_tf_property_ =
        new BoolProperty("Wait for transform", true,
                         "Whether to wait for the correct (interpolated) transform or use the latest "
                         "(possibly incorrect) transform. This can be useful when the should be "
                         "visualized as faster than TF is available",
                         this, SLOT(updateWaitForTf()), this);
  }

protected Q_SLOTS:
  virtual void updateWaitForTf() = 0;

protected:
  BoolProperty* wait_for_tf_property_;
};

/** @brief Display subclass using a tf2_ros::MessageFilter, templated on the ROS message type.
 *
 * This class brings together some common things used in many Display
 * types.  It has a tf2_ros::MessageFilter to filter incoming messages, and
 * it handles subscribing and unsubscribing when the display is
 * enabled or disabled.  It also has an Ogre::SceneNode which  */
template <class MessageType>
class LowLatencyMessageFilterDisplay : public _LowLatencyMessageFilterDisplay
{
  // No Q_OBJECT macro here, moc does not support Q_OBJECT in a templated class.
public:
  /** @brief Convenience typedef so subclasses don't have to use
   * the long templated class name to refer to their super class. */
  typedef LowLatencyMessageFilterDisplay<MessageType> LLMFDClass;

  LowLatencyMessageFilterDisplay() : messages_received_(0)
  {
    QString message_type = QString::fromStdString(ros::message_traits::datatype<MessageType>());
    topic_property_->setMessageType(message_type);
    topic_property_->setDescription(message_type + " topic to subscribe to.");
  }

  void onInitialize() override
  {
    // TODO: context_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
  }

  ~LowLatencyMessageFilterDisplay() override
  {
    LowLatencyMessageFilterDisplay::unsubscribe();
    LowLatencyMessageFilterDisplay::reset();
  }

  void reset() override
  {
    Display::reset();
    msg_tf_sync_.reset();
    messages_received_ = 0;
  }

  void setTopic(const QString& topic, const QString& /*datatype*/) override
  {
    topic_property_->setString(topic);
  }

protected:
  void updateTopic() override
  {
    unsubscribe();
    reset();
    subscribe();
    context_->queueRender();
  }

  void updateQueueSize() override
  {
    // TODO: tf_filter_->setQueueSize(static_cast<uint32_t>(queue_size_property_->getInt()));
    subscribe();
  }

  void updateWaitForTf() override
  {
    wait_for_tf_ = wait_for_tf_property_->getBool();
    if (msg_tf_sync_)
      msg_tf_sync_->waitForTransform(wait_for_tf_);
  }

  virtual void subscribe()
  {
    if (!isEnabled() || topic_property_->getStdString().empty())
    {
      return;
    }

    try
    {
      ros::TransportHints transport_hint = ros::TransportHints().reliable();
      // Determine UDP vs TCP transport for user selection.
      if (unreliable_property_->getBool())
      {
        transport_hint = ros::TransportHints().unreliable();
      }

      sub_ =
          update_nh_.subscribe(topic_property_->getStdString(),
                               static_cast<uint32_t>(queue_size_property_->getInt()),
                               &LowLatencyMessageFilterDisplay::incomingMessage, this, transport_hint);
      setStatus(StatusProperty::Ok, "Topic", "OK");
    }
    catch (ros::Exception& e)
    {
      setStatus(StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
    }
  }

  virtual void unsubscribe()
  {
    sub_.shutdown();
  }

  void onEnable() override
  {
    subscribe();
  }

  void onDisable() override
  {
    unsubscribe();
    reset();
  }

  void fixedFrameChanged() override
  {
    reset();
  }

  /** @brief Incoming message callback.  Checks if the message pointer
   * is valid, increments messages_received_, then calls
   * processMessage(). */
  void incomingMessage(const typename MessageType::ConstPtr& msg)
  {
    if (!msg)
    {
      return;
    }

    if (!msg_tf_sync_)
    {
      msg_tf_sync_ =
          std::make_shared<RosTransformSynchronizer<MessageType>>(context_->getTF2BufferPtr());
      msg_tf_sync_->reset(fixed_frame_.toStdString(), msg->header.frame_id);
      msg_tf_sync_->waitForTransform(wait_for_tf_);
      msg_tf_sync_->setCallback(&LowLatencyMessageFilterDisplay::incomingMessageWithTransform, this);
    }

    msg_tf_sync_->addMessage(msg->header.stamp, msg);
  }

  void incomingMessageWithTransform(const typename MessageType::ConstPtr& msg,
                                    const geometry_msgs::TransformStamped& transform)
  {
    // TODO: race conditions?
    transform_for_latest_message_ = transform;

    // process message synchronously in main GUI thread to avoid race conditions
    QMetaObject::invokeMethod(this, "processTypeErasedMessage", Qt::QueuedConnection,
                              Q_ARG(boost::shared_ptr<const void>,
                                    boost::static_pointer_cast<const void>(msg)));
  }

  void processTypeErasedMessage(boost::shared_ptr<const void> type_erased_msg) override
  {
    auto msg = boost::static_pointer_cast<const MessageType>(type_erased_msg);
    ++messages_received_;
    setStatus(StatusProperty::Ok, "Topic", QString::number(messages_received_) + " messages received");

    processMessage(msg);
  }

  /** @brief Implement this to process the contents of a message.
   *
   * This is called by incomingMessage(). */
  virtual void processMessage(const typename MessageType::ConstPtr& msg) = 0;

  ros::Subscriber sub_;
  uint32_t messages_received_;

  std::shared_ptr<RosTransformSynchronizer<MessageType>> msg_tf_sync_;
  geometry_msgs::TransformStamped transform_for_latest_message_;
  bool wait_for_tf_{true};
};

} // end namespace rviz

#endif // LOW_LATENCY_MESSAGE_FILTER_DISPLAY_H
