// Generated by gencpp from file ros_yolo/yoloFeedback.msg
// DO NOT EDIT!


#ifndef ROS_YOLO_MESSAGE_YOLOFEEDBACK_H
#define ROS_YOLO_MESSAGE_YOLOFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ros_yolo
{
template <class ContainerAllocator>
struct yoloFeedback_
{
  typedef yoloFeedback_<ContainerAllocator> Type;

  yoloFeedback_()
    : feedback_vector()  {
    }
  yoloFeedback_(const ContainerAllocator& _alloc)
    : feedback_vector(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _feedback_vector_type;
  _feedback_vector_type feedback_vector;





  typedef boost::shared_ptr< ::ros_yolo::yoloFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ros_yolo::yoloFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct yoloFeedback_

typedef ::ros_yolo::yoloFeedback_<std::allocator<void> > yoloFeedback;

typedef boost::shared_ptr< ::ros_yolo::yoloFeedback > yoloFeedbackPtr;
typedef boost::shared_ptr< ::ros_yolo::yoloFeedback const> yoloFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ros_yolo::yoloFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ros_yolo::yoloFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ros_yolo::yoloFeedback_<ContainerAllocator1> & lhs, const ::ros_yolo::yoloFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.feedback_vector == rhs.feedback_vector;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ros_yolo::yoloFeedback_<ContainerAllocator1> & lhs, const ::ros_yolo::yoloFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ros_yolo

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ros_yolo::yoloFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_yolo::yoloFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_yolo::yoloFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_yolo::yoloFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_yolo::yoloFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_yolo::yoloFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ros_yolo::yoloFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2ee6f212dbc91cfc6dfefacdb2b93a33";
  }

  static const char* value(const ::ros_yolo::yoloFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2ee6f212dbc91cfcULL;
  static const uint64_t static_value2 = 0x6dfefacdb2b93a33ULL;
};

template<class ContainerAllocator>
struct DataType< ::ros_yolo::yoloFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ros_yolo/yoloFeedback";
  }

  static const char* value(const ::ros_yolo::yoloFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ros_yolo::yoloFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#feedback\n"
"int32[] feedback_vector\n"
"\n"
;
  }

  static const char* value(const ::ros_yolo::yoloFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ros_yolo::yoloFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.feedback_vector);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct yoloFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ros_yolo::yoloFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ros_yolo::yoloFeedback_<ContainerAllocator>& v)
  {
    s << indent << "feedback_vector[]" << std::endl;
    for (size_t i = 0; i < v.feedback_vector.size(); ++i)
    {
      s << indent << "  feedback_vector[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.feedback_vector[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROS_YOLO_MESSAGE_YOLOFEEDBACK_H