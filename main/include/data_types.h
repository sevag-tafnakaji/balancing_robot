#ifndef DATA_TYPES_H
#define DATA_TYPES_H

typedef struct {
  float x;
  float y;
  float z;
} Vec3f;

typedef struct {
  int x;
  int y;
  int z;
} Vec3i;

typedef struct {
  float roll;
  float pitch;
  float yaw;
} eulerAngles_t;

typedef struct {
  Vec3f accel;
  Vec3f gyro;
} sensorData_t;

typedef struct {
  Vec3i offset_accel;
  Vec3i offset_gyro;
  float scale_accel;
  float scale_gyro;
} sensorConfig_t;

#endif  // DATA_TYPES_H
