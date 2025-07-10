#ifndef CAMERA_ID_HPP__
#define CAMERA_ID_HPP__

#pragma once

#include <cstdint>

enum CameraId: uint8_t
{
  ONE = 1,
  TWO = 2,
  THREE = 3,
  FOUR = 4,
  FIVE = 5,

  LAST // Caution: LAST should not be used!!!
};

#endif // CAMERA_ID_HPP__