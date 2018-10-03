#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <thread>

#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/xpressive/xpressive.hpp>
#include <boost/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>

namespace ba = boost::asio;
namespace qi = boost::spirit::qi;
namespace bp = boost::phoenix;

class Ssv102{

public:
    Ssv102(ba::io_service &io):
        nh("~"),
        io(),
        port(io),
        delim("\r")
    {
        nh.param<std::string>("device", dev, "/dev/ttyUSB0");
        nh.param("baud", baud, 19200);
        port.open(dev);
        port.set_option(ba::serial_port_base::baud_rate(static_cast<unsigned int>(baud)));
        port.set_option(ba::serial_port_base::flow_control(ba::serial_port_base::flow_control::none));
        port.set_option(ba::serial_port_base::parity(ba::serial_port_base::parity::none));
        port.set_option(ba::serial_port_base::stop_bits(ba::serial_port_base::stop_bits::one));
        port.set_option(ba::serial_port_base::character_size(8));
        ba::async_read_until(port, buffer, delim, boost::bind(&Ssv102::rx_callback, this, ba::placeholders::error, ba::placeholders::bytes_transferred));

        pub_navsat = nh.advertise<sensor_msgs::NavSatFix>("gps", 20);
        pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 20);
    }

    ~Ssv102()
    {
        if(port.is_open()){
            port.close();
        }
    }
    
    void close()
    {
        if(port.is_open()){
            port.close();
        }
    }

private:
    void rx_callback(const boost::system::error_code& e, std::size_t size){
        //std::cout << "call backed!!!" << std::endl;
        if(!e){
            std::istream ins(&buffer);
            std::string line;
            std::getline(ins, line);
            std::cout << line << std::endl;

            std::string talker;
            std::string message;
            std::string words;
            int checksum;
            if (qi::parse(
                line.cbegin(),
                line.cend(),
                (
                qi::lit("$") >>
                qi::as_string[qi::char_ >> qi::char_][bp::ref(talker) = qi::_1] >>
                qi::as_string[+qi::alnum][bp::ref(message) = qi::_1] >>
                qi::lit(",") >>
                qi::as_string[*(qi::char_ - '*')][bp::ref(words) = qi::_1] >>
                qi::lit("*") >>
                qi::hex[bp::ref(checksum) = qi::_1]
                )
                )) {
                std::cout << "Talker   : " << talker << std::endl;
                std::cout << "Message  : " << message << std::endl;
                std::cout << "Words    : " << words << std::endl;
                std::cout << "Checksum : " << checksum << std::endl;
            }else{
                std::cout << "Parse failed : " << line << std::endl;
            }

            ba::async_read_until(port, buffer, delim, boost::bind(&Ssv102::rx_callback, this, ba::placeholders::error, ba::placeholders::bytes_transferred));
        }
    }

    void config_device(){

    }
    
    ros::NodeHandle nh;
    ros::Publisher pub_navsat;
    ros::Publisher pub_pose;
    ba::io_service io;
    ba::serial_port port;
    std::string dev;
    std::string delim;
    int baud;
    ba::streambuf buffer;
    std::thread io_thread;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ssv102_ros");
    ROS_INFO("Start ssV102 driver node");
    ba::io_service io_service;
	Ssv102 gps(io_service);
    std::thread t( [&io_service](){ io_service.run(); });

	ros::spin();

    gps.close();
    t.join();

	return 0;
}