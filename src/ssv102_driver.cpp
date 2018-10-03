#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <thread>
#include <cmath>

#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/xpressive/xpressive.hpp>
#include <boost/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/lambda/lambda.hpp>

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
        nh.param<std::string>("framed_id", frame_id, "ssV102");

        pub_navsat = nh.advertise<sensor_msgs::NavSatFix>("gps", 20);
        pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 20);

        navsat_msg.header.frame_id = frame_id;
        navsat_msg.header.seq = 0;
        navsat_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }

    bool init()
    {
        try{
            port.open(dev);
            port.set_option(ba::serial_port_base::baud_rate(static_cast<unsigned int>(baud)));
            port.set_option(ba::serial_port_base::flow_control(ba::serial_port_base::flow_control::none));
            port.set_option(ba::serial_port_base::parity(ba::serial_port_base::parity::none));
            port.set_option(ba::serial_port_base::stop_bits(ba::serial_port_base::stop_bits::one));
            port.set_option(ba::serial_port_base::character_size(8));
            ba::async_read_until(port, buffer, delim, boost::bind(&Ssv102::rx_callback, this, ba::placeholders::error, ba::placeholders::bytes_transferred));
        }
        catch(boost::system::system_error e){
            ROS_ERROR("Faild to open a device. %s", e.what());
            return false;
        }
        return true;
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
            ros::Time now = ros::Time::now();
            std::istream ins(&buffer);
            std::string line;
            std::getline(ins, line);
            //std::cout << line << std::endl;

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
                //std::cout << "Talker   : " << talker << std::endl;
                //std::cout << "Message  : " << message << std::endl;
                //std::cout << "Words    : " << words << std::endl;
                //std::cout << "Checksum : " << checksum << std::endl;
                int checksum_calced = 0;
                for(auto it = line.cbegin() + 1; it != line.cend() - 4; ++it){
                    checksum_calced ^= static_cast<uint8_t>(*it);
                }

                if(checksum == checksum_calced){
                    if(message == "GGA"){
                        //std::cout << "GGA Sentense : " << words << std::endl;

                        double time;
                        double lat;
                        double lon;
                        double alt;
                        double alt_geo;
                        int mode;
                        int sat_num;
                        double hdop;
                        
                        if(qi::parse(
                          words.cbegin(),
                          words.cend(),
                          (
                              *qi::double_[bp::ref(time) = qi::_1] >> ',' >>
                              *qi::double_[([&lat](double dm){lat = Ssv102::dm2dd(dm);})] >> ',' >>
                              *(qi::lit('N') | qi::lit('S')[([&lat](){lat = -lat;})]) >> ',' >>
                              *qi::double_[([&lon](double dm){lon = Ssv102::dm2dd(dm);})] >> ',' >>
                              *(qi::lit('E') | qi::lit('W')[([&lon](){lon = -lon;})]) >> ',' >>
                              *qi::int_[bp::ref(mode) = qi::_1] >> ',' >>
                              *qi::int_[bp::ref(sat_num) = qi::_1] >> ',' >>
                              *qi::double_[bp::ref(hdop) = qi::_1] >> ',' >>
                              *qi::double_[bp::ref(alt) = qi::_1] >> ',' >>
                              *qi::lit('M') >> ',' >>
                              *qi::double_[bp::ref(alt_geo) = qi::_1] >> ',' >>
                              *qi::lit('M') >> ',' >>
                              *qi::double_ >> ',' >>
                              *qi::int_
                          )  
                        )){
                            if(mode > 0){
                                /*
                                std::cout << "Latitude : " << lat << std::endl;
                                std::cout << "Longitude : " << lon << std::endl;
                                std::cout << "Altitude : " << alt - alt_geo << std::endl;
                                std::cout << "HDOP : " << hdop << std::endl;           
                                */                     

                                navsat_msg.header.stamp = now;
                                navsat_msg.latitude = lat;
                                navsat_msg.longitude = lon;
                                navsat_msg.altitude = alt - alt_geo;
                                navsat_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
                                navsat_msg.status.status = mode - 1;

                                if(navsat_msg.position_covariance_type == sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN || 
                                navsat_msg.position_covariance_type == sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED){
                                    double co = std::pow(2.5 * hdop, 2.0);
                                    navsat_msg.position_covariance = {  co, 0.0, 0.0,
                                                                        0.0, co, 0.0,
                                                                        0.0, 0.0, co };
                                    navsat_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
                                }

                                pub_navsat.publish(navsat_msg);
                            }
                        }
                    }else if(message == "GST"){
                        std::cout << "GST Sentense : " << words << std::endl;

                    }
                }else{
                    ROS_INFO("GPS Checksum failed");
                }           
            }else{
                //std::cout << "Parse failed : " << line << std::endl;
            }

            ba::async_read_until(port, buffer, delim, boost::bind(&Ssv102::rx_callback, this, ba::placeholders::error, ba::placeholders::bytes_transferred));
        }else{
            ROS_ERROR("GPS Serialport receive error. Stop receiving.");
        }
    }

    void config_device(){

    }

    static double dm2dd(const double dm)
    {
        double dd = 0.0;
        dd = static_cast<int>(dm / 100.0);
        dd += (dm - dd * 100.0) / 60.0;
        return dd;
    };
    
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
    sensor_msgs::NavSatFix navsat_msg;
    int pose_seq;
    std::string frame_id;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ssv102_ros");
    ROS_INFO("Start ssV102 driver node");
    ba::io_service io_service;
	Ssv102 gps(io_service);

    if(gps.init()){
        std::thread t( [&io_service](){ io_service.run(); });

        ros::spin();

        gps.close();
        t.join();
    }

	return 0;
}