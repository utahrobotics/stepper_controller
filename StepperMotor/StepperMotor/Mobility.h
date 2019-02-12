#ifndef _ROS_motion_control_Mobility_h
#define _ROS_motion_control_Mobility_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace motion_control
{

  class Mobility : public ros::Msg
  {
    public:
      typedef float _front_right_type;
      _front_right_type front_right;
      typedef float _front_left_type;
      _front_left_type front_left;
      typedef float _rear_right_type;
      _rear_right_type rear_right;
      typedef float _rear_left_type;
      _rear_left_type rear_left;

    Mobility():
      front_right(0),
      front_left(0),
      rear_right(0),
      rear_left(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_front_right;
      u_front_right.real = this->front_right;
      *(outbuffer + offset + 0) = (u_front_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_front_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_front_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_front_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->front_right);
      union {
        float real;
        uint32_t base;
      } u_front_left;
      u_front_left.real = this->front_left;
      *(outbuffer + offset + 0) = (u_front_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_front_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_front_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_front_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->front_left);
      union {
        float real;
        uint32_t base;
      } u_rear_right;
      u_rear_right.real = this->rear_right;
      *(outbuffer + offset + 0) = (u_rear_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rear_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rear_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rear_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rear_right);
      union {
        float real;
        uint32_t base;
      } u_rear_left;
      u_rear_left.real = this->rear_left;
      *(outbuffer + offset + 0) = (u_rear_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rear_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rear_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rear_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rear_left);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_front_right;
      u_front_right.base = 0;
      u_front_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_front_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_front_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_front_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->front_right = u_front_right.real;
      offset += sizeof(this->front_right);
      union {
        float real;
        uint32_t base;
      } u_front_left;
      u_front_left.base = 0;
      u_front_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_front_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_front_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_front_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->front_left = u_front_left.real;
      offset += sizeof(this->front_left);
      union {
        float real;
        uint32_t base;
      } u_rear_right;
      u_rear_right.base = 0;
      u_rear_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rear_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rear_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rear_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rear_right = u_rear_right.real;
      offset += sizeof(this->rear_right);
      union {
        float real;
        uint32_t base;
      } u_rear_left;
      u_rear_left.base = 0;
      u_rear_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rear_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rear_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rear_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rear_left = u_rear_left.real;
      offset += sizeof(this->rear_left);
     return offset;
    }

    const char * getType(){ return "motion_control/Mobility"; };
    const char * getMD5(){ return "3b96da8ae2e84349c2bc3b9a36b75b56"; };

  };

}
#endif
