#include "AdaptiveApplication.h"
#include "HardwareConfig.h"
void setup() {
  adaptive_hardware::verifyBuildConfiguration(
      static_cast<adaptive_hardware::BuildConfiguration *>(nullptr));
  adaptive_application::initialize();
}
void loop() {
  adaptive_application::poll();
}
