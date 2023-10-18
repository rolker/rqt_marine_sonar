#ifndef RQT_MARINE_SONAR_ECHOGRAM_WIDGET_H
#define RQT_MARINE_SONAR_ECHOGRAM_WIDGET_H

#include <marine_acoustic_msgs/RawSonarImage.h>
#include <QChartView>
#include <QValueAxis>

namespace rqt_marine_sonar
{

class EchogramWidget: public QtCharts::QChartView
{
  Q_OBJECT
public:
  explicit EchogramWidget(QWidget *parent);

  float minimumDB() const;
  float maximumDB() const;
  float pingSpacing() const;
  float depthInterval() const;

signals:
  void mouseMoved(QPointF position);

public slots:
  void addPing(const marine_acoustic_msgs::RawSonarImage& ping);
  void setMinimumDB(float min_db);
  void setMaximumDB(float max_db);

  /// Set the distance in meters between pings for display
  void setPingSpacing(float spacing);

  /// Inteval between depth tick marks
  void setDepthInterval(float intrval);


protected:
  void resizeEvent(QResizeEvent* event) override;
  void wheelEvent(QWheelEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;

protected slots:
  void updateEchogram();
  void adjustPixmap();
  void adjustAxis();


private:
  static constexpr double min_zoom_scale_ = 1/10.0; // 10 m/pixel
  static constexpr double max_zoom_scale_ = 250; // 4 mm/pixel

  std::map<ros::Time, marine_acoustic_msgs::RawSonarImage> pings_;
  int maximum_ping_count_ = 2048;

  float min_db_ = -100.0;
  float max_db_ = 10.0;
  float ping_spacing_ = 1.0;

  float min_depth_ = 0.0;
  float max_depth_ = 0.0;
  float bin_size_ = 0.0;

  float depth_zoom_ = 1.0;
  float depth_offset_ = 0.0;

  bool translating_depth_ = false;
  float depth_translation_start_;
  float depth_offset_start_;

  QGraphicsPixmapItem* pixmap_item_ = nullptr;
  QtCharts::QValueAxis* depth_axis_ = nullptr;

  QImage echogram_;
};

}


#endif
