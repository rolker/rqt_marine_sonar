#include "rqt_marine_sonar/ping.h"

namespace rqt_marine_sonar
{

Ping::Ping(const marine_acoustic_msgs::RawSonarImage& message)
  :message_(message)
{

}

float Ping::minimumDepth() const
{
  return 0.5*message_.ping_info.sound_speed*message_.sample0/message_.sample_rate;
}

float Ping::maximumDepth() const
{
  return 0.5*message_.ping_info.sound_speed*(message_.sample0+message_.samples_per_beam)/message_.sample_rate;
}
  
float Ping::binSize() const
{
  return 0.5*message_.ping_info.sound_speed/message_.sample_rate;  
}

float Ping::sampleAt(float depth) const
{
  if(message_.image.dtype == marine_acoustic_msgs::SonarImageData::DTYPE_FLOAT32)
  {
    if(depth >= minimumDepth() && depth <= maximumDepth())
    {
      int index = (depth-minimumDepth())/binSize();
      return reinterpret_cast<const float*>(message_.image.data.data())[index];
    }

  }
  return std::nan("");
}


std::pair<float, float> depthRange(const marine_acoustic_msgs::RawSonarImage& ping)
{
  float min_depth = 0.5*ping.ping_info.sound_speed*ping.sample0/ping.sample_rate;
  float max_depth = 0.5*ping.ping_info.sound_speed*(ping.sample0+ping.samples_per_beam)/ping.sample_rate;
  return std::make_pair(min_depth, max_depth);
}


}
