#ifndef DATA_TYPES_H
#define DATA_TYPES_H

struct Vec3f {
  float x;
  float y;
  float z;
};

struct Vec3i {
  int x;
  int y;
  int z;
};

struct sensorData_t {
  struct Vec3f accel;
  struct Vec3f gyro;
};

struct sensorConfig_t {
  struct Vec3i offset_accel;
  struct Vec3i offset_gyro;
  float scale_accel;
  float scale_gyro;
};

#endif  // DATA_TYPES_H
