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
  echogram_ = QImage(maximum_ping_count_, maximum_ping_count_, QImage::Format_Grayscale8);
  echogram_.fill(Qt::lightGray);
  pixmap_item_ = scene()->addPixmap(QPixmap::fromImage(echogram_));
  pixmap_item_->setTransformationMode(Qt::SmoothTransformation);

  setChart(new QtCharts::QChart());

  pixmap_item_->setParentItem(chart());

  depth_axis_ = new QtCharts::QValueAxis();
  depth_axis_->setTickType(QtCharts::QValueAxis::TicksDynamic);
  depth_axis_->setTickInterval(20.0);
  depth_axis_->setTickAnchor(0.0);
  chart()->addAxis(depth_axis_, Qt::AlignLeft);

  setRenderHints(QPainter::Antialiasing);
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

  // figure out mouse position to focus zoom
  auto area = chart()->plotArea();

  auto pixel_focus = event->y() - area.top();

  auto depth_range = depth_axis_->max() - depth_axis_->min();
  auto meters_per_pixel = depth_range/area.height();

  auto depth_focus = depth_axis_->max() - pixel_focus*meters_per_pixel;

  // apply zoom
  depth_zoom_ *= scale_change;

  // clamp zoom in
  depth_zoom_ = std::max(depth_zoom_, 0.5f);

  // adjust offset 

  auto new_depth_range = (max_depth_-min_depth_)/depth_zoom_;
  auto new_meters_per_pixel = new_depth_range/area.height();
  auto new_max = depth_focus+(pixel_focus*new_meters_per_pixel);
  depth_offset_ = -new_max - min_depth_;

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

  auto area = chart()->plotArea();
  auto meters_per_pixel = range/area.height();

  // 100 pixel tick interval
  auto min_tick_interval = 100*meters_per_pixel;
  double factor = 0.1;
  std::vector<double> tick_bases = {1.0, 2.0, 5.0};
  bool done = false;
  while(!done)
  {
    for(auto tick_base: tick_bases)
      if(tick_base*factor >= min_tick_interval)
      {
        depth_axis_->setTickInterval(tick_base*factor);
        done = true;
        break;
      }
    factor *= 10.0;
  }

  adjustPixmap();
}

void EchogramWidget::adjustPixmap()
{
  auto area = chart()->plotArea();

  // scale to match display area width with echogram width
  double area_to_echogram_scale = area.width()/double(echogram_.width());

  // make sure we display a ping as at least one pixel when not zoomed in
  double base_pixel_width = std::max(area_to_echogram_scale, 1.0);

  int visible_echogram_pixels = std::min(echogram_.width(), int(ceil(area.width()/(base_pixel_width*ping_spacing_))));
  int startx = echogram_.width()-visible_echogram_pixels;

  double xscale = area.width()/double(visible_echogram_pixels);

  auto axis_min = depth_axis_->min();
  auto axis_max = depth_axis_->max();

  auto axis_min_depth = -axis_max;
  auto axis_max_depth = -axis_min;

  auto axis_range = axis_max - axis_min;

  // meters per pixel
  auto axis_scale = axis_range/double(area.height());

  int starty = std::max(0, std::min(echogram_.height(), int((axis_min_depth - min_depth_)/bin_size_)));
  int endy = std::max(0, std::min(echogram_.height(), int((axis_max_depth - min_depth_)/bin_size_)));

  auto pixmap = QPixmap::fromImage(echogram_).copy(startx, starty, visible_echogram_pixels, endy-starty);

  pixmap_item_->setPixmap(pixmap);

  // image pixels to area pixels
  float yscale = bin_size_/axis_scale;

  pixmap_item_->setTransform(QTransform::fromScale(xscale, yscale));

  auto pixmap_min_depth = min_depth_+starty*bin_size_;
  auto yoffset =  (axis_min_depth-pixmap_min_depth)/axis_scale;
  QPointF top_left = area.topLeft();
  top_left.setY(top_left.y()-yoffset);
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
    max_depth_ = std::max(max_depth_, ping.maximumDepth());
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
