#ifndef OMNI_TELEOP_GUI__OMNI_TELEOP_HH_
#define OMNI_TELEOP_GUI__OMNI_TELEOP_HH_

#include <string>

#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>

/// Omnidirectional teleoperation panel for the Gazebo GUI.
/// Publishes gz.msgs.Twist (linear.x, linear.y, angular.z) on <topic>.
class OmniTeleop : public gz::gui::Plugin
{
  Q_OBJECT

  Q_PROPERTY(QString topic READ Topic WRITE SetTopic NOTIFY TopicChanged)
  Q_PROPERTY(double maxLinear READ MaxLinear NOTIFY LimitsChanged)
  Q_PROPERTY(double maxAngular READ MaxAngular NOTIFY LimitsChanged)

public:
  OmniTeleop();
  ~OmniTeleop() override = default;

  void LoadConfig(const tinyxml2::XMLElement * _pluginElem) override;

  QString Topic() const;
  double MaxLinear() const;
  double MaxAngular() const;

  /// Velocities in m/s and rad/s in the robot frame.
  Q_INVOKABLE void Publish(double _vx, double _vy, double _wz);

public slots:
  void SetTopic(const QString & _topic);

signals:
  void TopicChanged();
  void LimitsChanged();

private:
  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
  std::string topic{"/cmd_vel"};
  double maxLinear{0.4};
  double maxAngular{1.0};
};

#endif  // OMNI_TELEOP_GUI__OMNI_TELEOP_HH_
