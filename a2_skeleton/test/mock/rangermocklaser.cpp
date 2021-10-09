#include "rangermocklaser.h"

RangerMockLaser::RangerMockLaser() {
}

RangerMockLaser::RangerMockLaser(unsigned int fov, unsigned int res, int offset, std::vector<double> mockData) {
  //Extra constructor for easy mocking
  mockData_ = mockData;

  Laser::setAngularResolution(res);
  Laser::setAngularOffset(offset);
  Laser::setFieldOfView(fov);
}


std::vector<double> RangerMockLaser::generateData() {
  return mockData_;
}
