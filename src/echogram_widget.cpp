#include "rqt_marine_sonar/echogram_widget.h"
#include <QGuiApplication>
#include <QWheelEvent>
#include <cmath>
#include <QGraphicsPixmapItem>
#include "rqt_marine_sonar/ping.h"
#include <QAreaSeries>

namespace rqt_marine_sonar
{

const double EchogramWidget::min_zoom_scale_;
const double EchogramWidget::max_zoom_scale_;


EchogramWidget::EchogramWidget(QWidget *parent): QtCharts::QChartView(parent)
{
  QImage echogram(maximum_ping_count_, maximum_ping_count_, QImage::Format_Grayscale8);
  echogram.fill(Qt::lightGray);
  pixmap_item_ = scene()->addPixmap(QPixmap::fromImage(echogram));
  pixmap_item_->setTransformationMode(Qt::SmoothTransformation);

  setChart(new QtCharts::QChart());

  pixmap_item_->setParentItem(chart());

  depth_axis_ = new QtCharts::QValueAxis();
  depth_axis_->setTickType(QtCharts::QValueAxis::TicksDynamic);
  depth_axis_->setTickInterval(20.0);
  depth_axis_->setTickAnchor(0.0);
  chart()->addAxis(depth_axis_, Qt::AlignLeft);

  setRenderHints(QPainter::Antialiasing);
  //setRubberBand(RectangleRubberBand);
}

void EchogramWidget::addPing(const marine_acoustic_msgs::RawSonarImage& ping)
{
  pings_[ping.header.stamp] = ping;
  while(pings_.size() > maximum_ping_count_)
    pings_.erase(pings_.begin()->first);
  updateEchogram();
}

void EchogramWidget::resizeEvent(QResizeEvent* event)
{
  QChartView::resizeEvent(event);
  adjustPixmap();  
}

void EchogramWidget::wheelEvent(QWheelEvent *event)
{
  QChartView::wheelEvent(event);

  // wheel turn angles are encoded in 1/8 degree increments.
  auto angle_delta_degrees = event->angleDelta().y()/8.0;
  
  double zoom_level_per_degree = 0.01;
  
  // fine zoom if ctrl key is pressed
  if(QGuiApplication::keyboardModifiers().testFlag(Qt::ControlModifier))
    zoom_level_per_degree /= 3.0;

  double scale_change = pow(2,zoom_level_per_degree*angle_delta_degrees);

  depth_zoom_ *= scale_change;

  depth_zoom_ = std::max(depth_zoom_, 0.5f);

  adjustAxis();
}

void EchogramWidget::mouseMoveEvent(QMouseEvent *event)
{
  QChartView::mouseMoveEvent(event);
  if(translating_depth_)
  {
    float dy = event->y()-depth_translation_start_;
    auto area = chart()->plotArea();
    auto axis_range = depth_axis_->max() - depth_axis_->min();
    auto delta_depth = axis_range*dy/area.height();
    depth_offset_ = depth_offset_start_-delta_depth;
    adjustAxis();

  }
}

void EchogramWidget::mousePressEvent(QMouseEvent *event)
{
  QChartView::mousePressEvent(event);
  if (event->button() == Qt::LeftButton)
  {
    depth_offset_start_ = depth_offset_;
    depth_translation_start_ = event->y();
    translating_depth_ = true;
  }
}

void EchogramWidget::mouseReleaseEvent(QMouseEvent *event)
{
  QChartView::mouseReleaseEvent(event);
  if (event->button() == Qt::LeftButton)
  {
    translating_depth_ = false;
  }
}


void EchogramWidget::adjustAxis()
{
  auto range = (max_depth_-min_depth_)/depth_zoom_;
  auto max = -(min_depth_+depth_offset_);
  auto min = max - range;
  depth_axis_->setRange(min, max);
  //depth_axis_->applyNiceNumbers();

  adjustPixmap();

}

void EchogramWidget::adjustPixmap()
{
  auto area = chart()->plotArea();

  auto pixmap = QPixmap::fromImage(echogram_);

  if(area.width() < echogram_.width())
  {
    auto startx = echogram_.width()-area.width();
    pixmap = pixmap.copy(startx, 0, area.width(), echogram_.height());
  }
  pixmap_item_->setPixmap(pixmap);

  float xscale = ping_spacing_*area.width()/float(pixmap_item_->pixmap().width());

  auto xoffset = area.width()-ceil(area.width()*ping_spacing_); 

  auto min = depth_axis_->min();
  auto max = depth_axis_->max();

  auto axis_range = depth_axis_->max() - depth_axis_->min();
  auto axis_scale = (max_depth_-min_depth_)/axis_range;
  float yscale = axis_scale*area.height()/float(pixmap_item_->pixmap().height());
  pixmap_item_->setTransform(QTransform::fromScale(xscale, yscale));

  // flip from altitude to depth
  min = -max;

  auto scaled_offset = (min-min_depth_)*area.height()/axis_range;
  QPointF top_left = chart()->plotArea().topLeft();
  top_left.setY(top_left.y()-scaled_offset);
  top_left.setX(top_left.x()+xoffset);
  pixmap_item_->setPos(top_left);

}

void EchogramWidget::updateEchogram()
{
  if(pings_.empty())
  {
    echogram_ =  QImage(maximum_ping_count_, maximum_ping_count_, QImage::Format_Grayscale8);
    echogram_.fill(Qt::lightGray);
    pixmap_item_->setPixmap(QPixmap::fromImage(echogram_));
    return;
  }

  Ping first_ping(pings_.begin()->second);
  min_depth_ = first_ping.minimumDepth();
  max_depth_ = first_ping.maximumDepth();
  bin_size_ = first_ping.binSize();
  for(auto p: pings_)
  {
    Ping ping(p.second);
    min_depth_ = std::min(min_depth_, ping.minimumDepth());
    max_depth_ = std::min(max_depth_, ping.maximumDepth());
    bin_size_ = std::min(bin_size_, ping.binSize());
  }

  if(bin_size_ > 0.0)
  {
    int depth_sample_count = (max_depth_-min_depth_)/bin_size_;
    echogram_ =  QImage(maximum_ping_count_, depth_sample_count, QImage::Format_Grayscale8);
    echogram_.fill(Qt::black);

    uint32_t ping_count = pings_.size();
    uint32_t ping_number = maximum_ping_count_ - ping_count;
    for(const auto& ping_message: pings_)
    {
      Ping ping(ping_message.second);
      if(ping_number >= 0)
      {
        for(int sample_number = 0; sample_number < depth_sample_count; sample_number++)
        {
          float value = ping.sampleAt(min_depth_+sample_number*bin_size_);
          if(!isnan(value))
            echogram_.scanLine(sample_number)[ping_number] = std::max(0, std::min(254,int(255*((value-min_db_)/(max_db_-min_db_)))));
        }
      }
      ping_number++;
    }


    adjustAxis();
  }
}


void EchogramWidget::setMinimumDB(float min_db)
{
  if(min_db_ != min_db)
  {
    min_db_ = min_db;
    updateEchogram();
  }
}

void EchogramWidget::setMaximumDB(float max_db)
{
  if(max_db_ != max_db)
  {
    max_db_ = max_db;
    updateEchogram();
  }
}

void EchogramWidget::setPingSpacing(float spacing)
{
  if(ping_spacing_ != spacing)
  {
    ping_spacing_ = spacing;
    updateEchogram();
  }
}

void EchogramWidget::setDepthInterval(float interval)
{
  depth_axis_->setTickInterval(interval);
}


float EchogramWidget::minimumDB() const
{
  return min_db_;
}

float EchogramWidget::maximumDB() const
{
  return max_db_;
}

float EchogramWidget::pingSpacing() const
{
  return ping_spacing_;
}

float EchogramWidget::depthInterval() const
{
  return depth_axis_->tickInterval();
}

} // namespace rqt_marine_sonar
