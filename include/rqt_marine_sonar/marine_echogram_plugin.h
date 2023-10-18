#ifndef RQT_MARINE_SONAR_MARINE_ECHOGRAM_PLUGIN_H
#define RQT_MARINE_SONAR_MARINE_ECHOGRAM_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_marine_echogram_plugin.h>
#include <ros/ros.h>
#include <marine_acoustic_msgs/RawSonarImage.h>


namespace rqt_marine_sonar
{

class MarineEchogramPlugin: public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MarineEchogramPlugin();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  virtual void dataCallback(const marine_acoustic_msgs::RawSonarImage &message);


protected slots:
  virtual void updateTopicList();
  virtual void selectTopic(const QString& topic);  
  virtual void onTopicChanged(int index);
  void newPings();

  void on_minDbDoubleSpinBox_valueChanged(double value);
  void on_maxDbDoubleSpinBox_valueChanged(double value);
  void on_pingSpacingDoubleSpinBox_valueChanged(double value);
  void on_depthIntervalDoubleSpinBox_valueChanged(double value);

private:
  Ui::MarineEchogramWidget ui_;
  QWidget* widget_ = nullptr;

  QString arg_topic_;

  ros::Subscriber data_subscriber_;

  std::vector<marine_acoustic_msgs::RawSonarImage> new_pings_;
  std::mutex new_pings_mutex_;

};

} // namespace rqt_marine_sonar

#endif
