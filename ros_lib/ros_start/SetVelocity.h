#ifndef _ROS_SERVICE_SetVelocity_h
#define _ROS_SERVICE_SetVelocity_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ros_start
{

static const char SETVELOCITY[] = "ros_start/SetVelocity";

  class SetVelocityRequest : public ros::Msg
  {
    public:
      typedef float _linear_velocity_type;
      _linear_velocity_type linear_velocity;
      typedef float _angular_velocity_type;
      _angular_velocity_type angular_velocity;

    SetVelocityRequest():
      linear_velocity(0),
      angular_velocity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->linear_velocity);
      offset += serializeAvrFloat64(outbuffer + offset, this->angular_velocity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->linear_velocity));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angular_velocity));
     return offset;
    }

    const char * getType(){ return SETVELOCITY; };
    const char * getMD5(){ return "e55b2cec3678035367208627e07de350"; };

  };

  class SetVelocityResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetVelocityResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return SETVELOCITY; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetVelocity {
    public:
    typedef SetVelocityRequest Request;
    typedef SetVelocityResponse Response;
  };

}
#endif
