/* Auto-generated by genmsg_cpp for file /home/djifos/Desktop/pandora_svn/ros/gui_communications/msg/targetPosition.msg */
#ifndef GUI_COMMUNICATIONS_MESSAGE_TARGETPOSITION_H
#define GUI_COMMUNICATIONS_MESSAGE_TARGETPOSITION_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"


namespace gui_communications
{
template <class ContainerAllocator>
struct targetPosition_ : public ros::Message
{
  typedef targetPosition_<ContainerAllocator> Type;

  targetPosition_()
  : x(0.0)
  , y(0.0)
  {
  }

  targetPosition_(const ContainerAllocator& _alloc)
  : x(0.0)
  , y(0.0)
  {
  }

  typedef float _x_type;
  float x;

  typedef float _y_type;
  float y;


private:
  static const char* __s_getDataType_() { return "gui_communications/targetPosition"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "ff8d7d66dd3e4b731ef14a45d38888b6"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "float32 x\n\
float32 y\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, x);
    ros::serialization::serialize(stream, y);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, x);
    ros::serialization::deserialize(stream, y);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(x);
    size += ros::serialization::serializationLength(y);
    return size;
  }

  typedef boost::shared_ptr< ::gui_communications::targetPosition_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gui_communications::targetPosition_<ContainerAllocator>  const> ConstPtr;
}; // struct targetPosition
typedef  ::gui_communications::targetPosition_<std::allocator<void> > targetPosition;

typedef boost::shared_ptr< ::gui_communications::targetPosition> targetPositionPtr;
typedef boost::shared_ptr< ::gui_communications::targetPosition const> targetPositionConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::gui_communications::targetPosition_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::gui_communications::targetPosition_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace gui_communications

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::gui_communications::targetPosition_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ff8d7d66dd3e4b731ef14a45d38888b6";
  }

  static const char* value(const  ::gui_communications::targetPosition_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xff8d7d66dd3e4b73ULL;
  static const uint64_t static_value2 = 0x1ef14a45d38888b6ULL;
};

template<class ContainerAllocator>
struct DataType< ::gui_communications::targetPosition_<ContainerAllocator> > {
  static const char* value() 
  {
    return "gui_communications/targetPosition";
  }

  static const char* value(const  ::gui_communications::targetPosition_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::gui_communications::targetPosition_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 x\n\
float32 y\n\
\n\
";
  }

  static const char* value(const  ::gui_communications::targetPosition_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::gui_communications::targetPosition_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::gui_communications::targetPosition_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.x);
    stream.next(m.y);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct targetPosition_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gui_communications::targetPosition_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::gui_communications::targetPosition_<ContainerAllocator> & v) 
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
  }
};


} // namespace message_operations
} // namespace ros

#endif // GUI_COMMUNICATIONS_MESSAGE_TARGETPOSITION_H
