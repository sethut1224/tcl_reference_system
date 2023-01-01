#ifndef REFERENCE_TCL_INTERFACE_HPP_
#define REFERENCE_TCL_INTERFACE_HPP_

#include <type_traits>
#include "tcl_std_msgs/msg/timing_header.hpp"
#include "rclcpp/tcl_node_interfaces/node_timing_interface.hpp"

#include "tcl_reference_system_msg/msg/tcl_dynamic_message.hpp"

namespace reference_tcl_interface
{
    template<typename M, typename = void>
    struct hasTimingHeader : public std::false_type {};

    template<typename M>
    struct hasTimingHeader<M, decltype((void) M::timing_header)>: std::true_type {};

    template<typename MT1, typename MT2, typename Enable = void>
    struct MessageConversion
    {
      static void convert(MT1& mt1, MT2& mt2, rclcpp::tcl_node_interfaces::NodeTimingInterface::SharedPtr node_timing)
      {
          (void)mt1;
          (void)mt2;
      }

      static void convert(const MT1& mt1, MT2& mt2, rclcpp::tcl_node_interfaces::NodeTimingInterface::SharedPtr node_timing)
      {
          (void)mt1;
          (void)mt2;
      }

      static void convert(MT1& mt1, const MT2& mt2, rclcpp::tcl_node_interfaces::NodeTimingInterface::SharedPtr node_timing)
      {
          (void)mt1;
          (void)mt2;
      }

      static void convert(const MT1& mt1, const MT2& mt2, rclcpp::tcl_node_interfaces::NodeTimingInterface::SharedPtr node_timing)
      {
          (void)mt1;
          (void)mt2;
      }
    };

    template<typename MT1, typename MT2>
    struct MessageConversion<MT1, MT2, typename std::enable_if<hasTimingHeader<MT1>::value>::type>
    {
      static void convert(MT1& mt1, MT2& mt2, rclcpp::tcl_node_interfaces::NodeTimingInterface::SharedPtr node_timing)
      {
        node_timing->receive_timing_header(mt1.timing_header);
        mt2 = mt1.data;
      }

      static void convert(const MT1& mt1, MT2& mt2, rclcpp::tcl_node_interfaces::NodeTimingInterface::SharedPtr node_timing)
      {
        node_timing->receive_timing_header(mt1.timing_header);
        mt2 = mt1.data;
      }
    };

    template<typename MT1, typename MT2>
    struct MessageConversion<MT1, MT2, typename std::enable_if<hasTimingHeader<MT2>::value>::type>
    {
      static void convert(MT1& mt1, MT2& mt2, rclcpp::tcl_node_interfaces::NodeTimingInterface::SharedPtr node_timing)
      {
        auto timing_header = node_timing->get_timing_header();
        if(timing_header)
        {
          mt2.data = mt1;
          mt2.timing_header = *timing_header;
        }
      }

      static void convert(MT1& mt1, const MT2& mt2, rclcpp::tcl_node_interfaces::NodeTimingInterface::SharedPtr node_timing)
      {
        auto timing_header = node_timing->get_timing_header();
        if(timing_header)
        {
          mt2.data = mt1;
          mt2.timing_header = *timing_header;
        }
      }
    };


    template <class MT1, class MT2>
    void messageInterface(MT1& msg1, MT2& msg2, rclcpp::tcl_node_interfaces::NodeTimingInterface::SharedPtr node_timing)
    {   
        MessageConversion<MT1, MT2>::convert(msg1, msg2, node_timing);
    }

    template <class MT1, class MT2>
    void messageInterface(const MT1& msg1, MT2& msg2, rclcpp::tcl_node_interfaces::NodeTimingInterface::SharedPtr node_timing)
    {
        MessageConversion<MT1, MT2>::convert(msg1, msg2, node_timing);
    }

    template <class MT1, class MT2>
    void messageInterface(MT1& msg1, const MT2& msg2, rclcpp::tcl_node_interfaces::NodeTimingInterface::SharedPtr node_timing)
    {
        MessageConversion<MT1, MT2>::convert(msg1, msg2, node_timing);
    }
}
#endif