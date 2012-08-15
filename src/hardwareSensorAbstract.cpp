/**
 * \file hardwareSensorAbstract.cpp
 * \date 15/03/2011
 * \author croussil
 * \ingroup rtslam
 */


#include "rtslam/hardwareSensorAbstract.hpp"

namespace jafar {
namespace rtslam {
namespace hardware {

  //enum Quantity { qPos, qOriQuat, qOriEuler, qVel, qAbsVel, qAngVel, qAbsAngVel, qAcc, qAbsAcc, qBundleobs, qMag, qNQuantity };
  const int ReadingAbstract::QuantityDataSizes[qNQuantity] = { 3, 4, 3, 3, 3, 3, 3, 3, 3, 6, 3 };
  const int ReadingAbstract::QuantityObsSizes [qNQuantity] = { 3, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3 };


}}}
