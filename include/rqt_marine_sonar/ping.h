#ifndef RQT_MARINE_SONAR_PING_H
#define RQT_MARINE_SONAR_PING_H

#include <marine_acoustic_msgs/RawSonarImage.h>

namespace rqt_marine_sonar
{

class Ping
{
public:
  Ping(const marine_acoustic_msgs::RawSonarImage &message);

  float minimumDepth() const;
  float maximumDepth() const;
  float binSize() const;

  float sampleAt(float distance) const;

private:
  const marine_acoustic_msgs::RawSonarImage& message_;
};

}

#endif
