#include "rqt_marine_sonar/marine_echogram_plugin.h"
#include <pluginlib/class_list_macros.h>


namespace rqt_marine_sonar
{

MarineEchogramPlugin::MarineEchogramPlugin():rqt_gui_cpp::Plugin()
{
  setObjectName("MarineEchogram");
}

void MarineEchogramPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);
    
  widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");

  connect(ui_.minDbDoubleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MarineEchogramPlugin::on_minDbDoubleSpinBox_valueChanged);
  connect(ui_.maxDbDoubleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MarineEchogramPlugin::on_maxDbDoubleSpinBox_valueChanged);
  connect(ui_.pingSpacingDoubleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MarineEchogramPlugin::on_pingSpacingDoubleSpinBox_valueChanged);
  connect(ui_.depthIntervalDoubleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MarineEchogramPlugin::on_depthIntervalDoubleSpinBox_valueChanged);
    
  context.addWidget(widget_);
    
  updateTopicList();

  ui_.topicsComboBox->setCurrentIndex(ui_.topicsComboBox->findText(""));
  connect(ui_.topicsComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));

  ui_.refreshTopicsPushButton->setIcon(QIcon::fromTheme("view-refresh"));
  connect(ui_.refreshTopicsPushButton, SIGNAL(pressed()), this, SLOT(updateTopicList()));

  // set topic name if passed in as argument
  const QStringList& argv = context.argv();
  if (!argv.empty()) {
    arg_topic_ = argv[0];
    selectTopic(arg_topic_);
  }

}

void MarineEchogramPlugin::shutdownPlugin()
{
  data_subscriber_.shutdown();
}

void MarineEchogramPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  QString topic = ui_.topicsComboBox->currentText();
  instance_settings.setValue("topic", topic);

  instance_settings.setValue("minimum_db", ui_.echogramWidget->minimumDB());
  instance_settings.setValue("maximum_db", ui_.echogramWidget->maximumDB());
  instance_settings.setValue("ping_spacing", ui_.echogramWidget->pingSpacing());
  instance_settings.setValue("depth_interval", ui_.echogramWidget->depthInterval());
}

void MarineEchogramPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  QString topic = instance_settings.value("topic", "").toString();
  // don't overwrite topic name passed as command line argument
  if (!arg_topic_.isEmpty())
  {
    arg_topic_ = "";
  }
  else
  {
    selectTopic(topic);
  }

  ui_.echogramWidget->setMinimumDB(instance_settings.value("minimum_db", -100.0).toFloat());
  ui_.echogramWidget->setMaximumDB(instance_settings.value("maximum_db", 10.0).toFloat());
  ui_.echogramWidget->setPingSpacing(instance_settings.value("ping_spacing", 1.0).toFloat());
  ui_.echogramWidget->setDepthInterval(instance_settings.value("depth_interval", 100.0).toFloat());

  ui_.minDbDoubleSpinBox->setValue(ui_.echogramWidget->minimumDB());
  ui_.maxDbDoubleSpinBox->setValue(ui_.echogramWidget->maximumDB());
  ui_.pingSpacingDoubleSpinBox->setValue(ui_.echogramWidget->pingSpacing());
  ui_.depthIntervalDoubleSpinBox->setValue(ui_.echogramWidget->depthInterval());
}


void MarineEchogramPlugin::updateTopicList()
{
  QString selected = ui_.topicsComboBox->currentText();
    
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);
    
  QList<QString> topics;
  for(const auto t: topic_info)
    if (t.datatype == "marine_acoustic_msgs/RawSonarImage")
      topics.append(t.name.c_str());
        
  topics.append("");
  qSort(topics);
  ui_.topicsComboBox->clear();
  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
  {
    QString label(*it);
    label.replace(" ", "/");
    ui_.topicsComboBox->addItem(label, QVariant(*it));
  }

  // restore previous selection
  selectTopic(selected);
}

void MarineEchogramPlugin::selectTopic(const QString& topic)
{
  int index = ui_.topicsComboBox->findText(topic);
  if (index == -1)
  {
    // add topic name to list if not yet in
    QString label(topic);
    label.replace(" ", "/");
    ui_.topicsComboBox->addItem(label, QVariant(topic));
    index = ui_.topicsComboBox->findText(topic);
  }
  ui_.topicsComboBox->setCurrentIndex(index);
}

void MarineEchogramPlugin::onTopicChanged(int index)
{
  data_subscriber_.shutdown();
  ui_.echogramWidget->chart()->setTitle("");
  QString topic = ui_.topicsComboBox->itemData(index).toString();
  if(!topic.isEmpty())
  {
    data_subscriber_ = getNodeHandle().subscribe(topic.toStdString(), 10, &MarineEchogramPlugin::dataCallback, this);
    ui_.echogramWidget->chart()->setTitle(topic);
  }
}

void MarineEchogramPlugin::dataCallback(const marine_acoustic_msgs::RawSonarImage& message)
{
  {
    std::lock_guard<std::mutex> lock(new_pings_mutex_);
    new_pings_.push_back(message);
  }
  QMetaObject::invokeMethod(this,"newPings", Qt::QueuedConnection);
}

void MarineEchogramPlugin::newPings()
{
  std::lock_guard<std::mutex> lock(new_pings_mutex_);
  for(const auto& ping: new_pings_)
    ui_.echogramWidget->addPing(ping);
  new_pings_.clear();
}


void MarineEchogramPlugin::on_minDbDoubleSpinBox_valueChanged(double value)
{
  ui_.echogramWidget->setMinimumDB(value);
}

void MarineEchogramPlugin::on_maxDbDoubleSpinBox_valueChanged(double value)
{
  ui_.echogramWidget->setMaximumDB(value);
}

void MarineEchogramPlugin::on_pingSpacingDoubleSpinBox_valueChanged(double value)
{
  ui_.echogramWidget->setPingSpacing(value);
}

void MarineEchogramPlugin::on_depthIntervalDoubleSpinBox_valueChanged(double value)
{
  ui_.echogramWidget->setDepthInterval(value);
}


}

PLUGINLIB_EXPORT_CLASS(rqt_marine_sonar::MarineEchogramPlugin, rqt_gui_cpp::Plugin)
