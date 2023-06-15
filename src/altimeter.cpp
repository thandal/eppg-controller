#include "sp140/altimeter.h"
#include "sp140/structs.h"

#include <Adafruit_BMP3XX.h>

Adafruit_BMP3XX bmp;
bool bmpPresent = false;
float groundAltitude = 0;

void setGroundAltitude(float altitude) {
  groundAltitude = altitude;
}

// TODO: store the initial sea pressure (from setupAltimeter),
// and then we can remove deviceData here...
float getAltitude(const STR_DEVICE_DATA_140_V1& deviceData) {
  if (!bmpPresent) return 0.0;
  if (!bmp.performReading()) return 0.0;
  return bmp.readAltitude(deviceData.sea_pressure) - groundAltitude;
}

// Start the bmp388 sensor
void setupAltimeter(const STR_DEVICE_DATA_140_V1& deviceData) {
  if (!bmp.begin_I2C()) return;
  bmp.setOutputDataRate(BMP3_ODR_25_HZ);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);
  getAltitude(deviceData);  // Discard first value
  bmpPresent = true;
}