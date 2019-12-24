#ifndef _ROS_SERVICE_SetMavFrame_h
#define _ROS_SERVICE_SetMavFrame_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mavros_msgs
{

static const char SETMAVFRAME[] = "mavros_msgs/SetMavFrame";

  class SetMavFrameRequest : public ros::Msg
  {
    public:
      typedef uint8_t _mav_frame_type;
      _mav_frame_type mav_frame;
      enum { FRAME_GLOBAL =  0                    };
      enum { FRAME_LOCAL_NED =  1                 };
      enum { FRAME_MISSION =  2                   };
      enum { FRAME_GLOBAL_RELATIVE_ALT =  3       };
      enum { FRAME_LOCAL_ENU =  4                 };
      enum { FRAME_GLOBAL_INT =  5                };
      enum { FRAME_GLOBAL_RELATIVE_ALT_INT =  6   };
      enum { FRAME_LOCAL_OFFSET_NED =  7          };
      enum { FRAME_BODY_NED =  8                  };
      enum { FRAME_BODY_OFFSET_NED =  9           };
      enum { FRAME_GLOBAL_TERRAIN_ALT =  10       };
      enum { FRAME_GLOBAL_TERRAIN_ALT_INT =  11   };

    SetMavFrameRequest():
      mav_frame(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->mav_frame >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mav_frame);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->mav_frame =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mav_frame);
     return offset;
    }

    const char * getType(){ return SETMAVFRAME; };
    const char * getMD5(){ return "4102fcf8d7971e4f06392711a40bc2cd"; };

  };

  class SetMavFrameResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetMavFrameResponse():
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

    const char * getType(){ return SETMAVFRAME; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetMavFrame {
    public:
    typedef SetMavFrameRequest Request;
    typedef SetMavFrameResponse Response;
  };

}
#endif
