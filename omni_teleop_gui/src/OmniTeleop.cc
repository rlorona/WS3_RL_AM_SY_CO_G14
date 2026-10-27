#include "OmniTeleop.hh"

#include <gz/common/Console.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>

OmniTeleop::OmniTeleop()
{
  this->SetTopic(QString::fromStdString(this->topic));
}

void OmniTeleop::LoadConfig(const tinyxml2::XMLElement * _pluginElem)
{
  if (this->title.empty()) {
    this->title = "Omnidirectional teleop";
  }
  if (!_pluginElem) {
    return;
  }
  if (auto elem = _pluginElem->FirstChildElement("topic"); elem && elem->GetText()) {
    this->SetTopic(QString::fromStdString(elem->GetText()));
  }
  if (auto elem = _pluginElem->FirstChildElement("max_linear")) {
    elem->QueryDoubleText(&this->maxLinear);
  }
  if (auto elem = _pluginElem->FirstChildElement("max_angular")) {
    elem->QueryDoubleText(&this->maxAngular);
  }
  emit this->LimitsChanged();
}

QString OmniTeleop::Topic() const
{
  return QString::fromStdString(this->topic);
}

double OmniTeleop::MaxLinear() const
{
  return this->maxLinear;
}

double OmniTeleop::MaxAngular() const
{
  return this->maxAngular;
}

void OmniTeleop::SetTopic(const QString & _topic)
{
  this->topic = _topic.toStdString();
  this->pub = this->node.Advertise<gz::msgs::Twist>(this->topic);
  if (!this->pub) {
    gzerr << "Could not advertise topic [" << this->topic << "]" << std::endl;
  }
  emit this->TopicChanged();
}

void OmniTeleop::Publish(double _vx, double _vy, double _wz)
{
  gz::msgs::Twist msg;
  msg.mutable_linear()->set_x(_vx);
  msg.mutable_linear()->set_y(_vy);
  msg.mutable_angular()->set_z(_wz);
  this->pub.Publish(msg);
}

GZ_ADD_PLUGIN(OmniTeleop, gz::gui::Plugin)
