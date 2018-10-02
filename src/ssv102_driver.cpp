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
        std::string str = "Hello World\r\n";
        port.write_some(ba::buffer(str, str.length()));
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

            sensor_msgs::NavSatFix navsat;
            geometry_msgs::PoseStamped pose;

            pub_navsat.publish(navsat);
            pub_pose.publish(pose);

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
    std::thread t([&io_service](){ io_service.run(); });

	ros::spin();

    gps.close();
    t.join();

	return 0;
}