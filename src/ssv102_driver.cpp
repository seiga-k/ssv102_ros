#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>
#include <thread>
#include <cmath>
#include <sys/ioctl.h>
#include <errno.h>  
#include <ctime>

#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/xpressive/xpressive.hpp>
#include <boost/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace ba = boost::asio;
namespace qi = boost::spirit::qi;
namespace bp = boost::phoenix;

class Ssv102{

public:
    Ssv102(ba::io_service &io) :
        nh("~"),
        io_service(io),
        port(io),
        delim("\r\n")
    {
        nh.param<std::string>("device", dev, "/dev/ttyUSB0");
        nh.param<int>("baud", baud, 115200);
        nh.param<std::string>("framed_id", frame_id, "ssv102");

        pub_navsat = nh.advertise<sensor_msgs::NavSatFix>("fix", 20);
        pub_time = nh.advertise<sensor_msgs::TimeReference>("time", 20);
        pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 20);
        pub_twist = nh.advertise<geometry_msgs::TwistStamped>("twist", 20);

        navsat_msg.header.frame_id = frame_id;
        navsat_msg.header.seq = 0;
        navsat_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        pose.header.frame_id = frame_id;
        pose.header.seq = 0;
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;
        twist.header.frame_id = frame_id;
        twist.header.seq = 0;

        time_msg.header = navsat_msg.header;
        time_msg.source = "GNSS";
    }
    
    bool init()
    {
        try{
            port.open(dev);
            port.set_option(ba::serial_port_base::baud_rate(baud));
            port.set_option(ba::serial_port_base::flow_control(ba::serial_port_base::flow_control::none));
            port.set_option(ba::serial_port_base::parity(ba::serial_port_base::parity::none));
            port.set_option(ba::serial_port_base::stop_bits(ba::serial_port_base::stop_bits::one));
            port.set_option(ba::serial_port_base::character_size(8));

            ba::async_read_until(port, buffer, delim, boost::bind(&Ssv102::rx_callback, this, ba::placeholders::error, ba::placeholders::bytes_transferred));
            t = std::thread( [this](){ io_service.run(); });

            config_device();
            ROS_INFO("ssV102 configure completed. Start to receiving.");

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
            port.cancel();
            port.close();
        }
    }
    
    void close()
    {
        if(port.is_open()){
            port.cancel();
            port.close();
            t.join();
        }
    }

private:
    void rx_callback(const boost::system::error_code& e, std::size_t size){
        //std::cout << "call backed!!!" << std::endl;
        if(e){            
            ROS_ERROR("GPS Serialport receive error. Stop receiving.");
            return;
        }

        ros::Time now = ros::Time::now();
        static ros::Time stamp_vtg(now);
        static ros::Time stamp_rot(now);
        const double vtgrot_to(1.0 / 10.0 / 2.0);
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
                    parse_gga(words, now);
                }else if(message == "GST"){
                    //std::cout << "GST Sentense : " << words << std::endl;
                    parse_gst(words, now);
                }else if(talker == "PS" && message == "AT"){
                    if(words.find("HPR") != std::string::npos){
                        parse_hpr(words, now);
                    }
                }else if(message == "ZDA"){
                    parse_zda(words, now);
                }else if(message == "VTG"){
                    if(parse_vtg(words, now)){
                        if((now - stamp_rot).toSec() < vtgrot_to){
                            twist.header.stamp = now;
                            pub_twist.publish(twist);
                        }
                        stamp_vtg = now;
                    }
                }else if(message == "ROT"){
                    if(parse_rot(words, now)){
                        if((now - stamp_vtg).toSec() < vtgrot_to){
                            twist.header.stamp = now;
                            pub_twist.publish(twist);
                        }
                        stamp_rot = now;
                    }
                }
            }else{
                ROS_WARN("SSV102 checksum failed.");
            }           
        }else{
            ROS_WARN("SSV102 NMEA parse failed. Recieved line : %s", line.c_str());
        }

        ba::async_read_until(port, buffer, delim, boost::bind(&Ssv102::rx_callback, this, ba::placeholders::error, ba::placeholders::bytes_transferred));
    }

    bool parse_gga(const std::string& words, const ros::Time& stamp){
        double utc;
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
                *qi::double_[bp::ref(utc) = qi::_1] >> ',' >>
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
                std::cout << "UTC : " << utc << std::endl;
                */

                navsat_msg.header.stamp = stamp;
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
                navsat_msg.header.seq++;
                return true;
            }
        }
        return false;
    }

    bool parse_gst(const std::string& words, const ros::Time& stamp){
        double co_lon = -1.0;
        double co_lat = -1.0;
        double co_alt = -1.0;

        if(qi::parse(
            words.cbegin(),
            words.cend(),
            (
                *qi::double_ >> ',' >>    // UTC
                *qi::double_ >> ',' >>    // Total covariance
                *qi::double_ >> ',' >>    // Covariance in long axis
                *qi::double_ >> ',' >>    // Covariance in short axis
                *qi::double_ >> ',' >>    // Heading angle of long axis
                *qi::double_[bp::ref(co_lat) = qi::_1] >> ',' >>
                *qi::double_[bp::ref(co_lon) = qi::_1] >> ',' >>
                *qi::double_[bp::ref(co_alt) = qi::_1]
            )  
        )){
            if(co_lon > 0.0 && co_lat > 0.0 && co_alt > 0.0){
                navsat_msg.position_covariance = {  co_lon * co_lon, 0.0, 0.0,
                                                    0.0, co_lat * co_lat, 0.0,
                                                    0.0, 0.0, co_alt * co_alt };
                navsat_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
                return true;
            }
        }
        return false;
    }

    bool parse_hpr(const std::string& words, const ros::Time& stamp){
        //std::cout << "HPR Sentense : " << words << std::endl;
        double heading = 1000;
        double pitch;
        double roll;
        std::string mode;
        if(qi::parse(
            words.cbegin(),
            words.cend(),
            (
                qi::lit("HPR") >> ',' >>    // Header string
                *qi::double_ >> ',' >>      // UTC
                *qi::double_[bp::ref(heading) = -qi::_1] >> ',' >>
                *qi::double_[bp::ref(pitch) = -qi::_1] >> ',' >>
                *qi::double_[bp::ref(roll) = qi::_1] >> ',' >>
                *qi::alnum[bp::ref(mode) = qi::_1]
            )
        )){
            if(heading != 1000){
                /*
                std::cout << "HPR" << std::endl;
                std::cout << "Heading : " << heading << std::endl;
                std::cout << "Pitch   : " << pitch << std::endl;
                std::cout << "Roll    : " << roll << std::endl; 
                */
                pose.header.stamp = stamp;

                // SSV102の出力するheadingは真北が0度でCW方向が正の座標系で出力されている．
                // これをROSで一般的な東がX，北がYの座標系に沿うように，東が0度，CCW方向が正となるように変換して出力する．
                tf2::Quaternion q;
                q.setRPY(roll * M_PI / 180.0, pitch * M_PI / 180.0, (heading + 90.0) * M_PI / 180.0);
                pose.pose.orientation = tf2::toMsg(q);

                pub_pose.publish(pose);
                pose.header.seq++;
                return true;
            }
        }
        return false;
    }

    bool parse_zda(const std::string& words, const ros::Time& stamp){
        double hhmmss;
        int dd, MM, yyyy, xx, yy;
        if(qi::parse(
            words.cbegin(),
            words.cend(),
            (
                *qi::double_[bp::ref(hhmmss) = qi::_1] >> ',' >>    // hours minutes seconds . miliseconds
                *qi::int_[bp::ref(dd) = qi::_1] >> ',' >>           // day
                *qi::int_[bp::ref(MM) = qi::_1] >> ',' >>           // month
                *qi::int_[bp::ref(yyyy) = qi::_1] >> ',' >>         // year
                *qi::int_[bp::ref(xx) = qi::_1] >> ',' >>           // local zone hours
                *qi::int_[bp::ref(yy) = qi::_1]                     // local zone minutes
            )  
        )){
            int hh, mm, ss, ns;
            hh = hhmmss / 10000.0;
            mm = hhmmss / 100.0 - hh * 100;
            ss = hhmmss - hh * 10000 - mm * 100;
            ns = (hhmmss - hh * 10000.0 - mm * 100.0 - ss) * 10000000.0;
            /*
            std::cout << "ZDA" << std::endl;
            std::cout << "Input  : " << hhmmss << std::endl;
            std::cout << "Output : " << hh << mm << ss << "." << ns << std::endl;
            */
            try{
                boost::gregorian::date date(yyyy, MM, dd);
                boost::posix_time::ptime boost_time(boost::gregorian::date(yyyy, MM, dd), boost::posix_time::time_duration(hh, mm, ss, ns));
                time_msg.header.stamp = stamp;
                time_msg.time_ref = ros::Time::fromBoost(boost_time);
                pub_time.publish(time_msg);
                time_msg.header.seq++;
                return true;
            }
            catch(std::out_of_range ex){
                ROS_WARN("GPZDA sentense parse failed.");
            }
        }
        return false;
    }

    bool parse_vtg(const std::string& words, const ros::Time& stamp){
        double vel, heading;
        if(qi::parse(
            words.cbegin(),
            words.cend(),
            (
                *qi::double_[bp::ref(heading) = qi::_1] >> ',' >>   // heading angle for north
                'T' >> ',' >>
                *qi::double_ >> ',' >>                              // heading angle for magnetic north
                'M' >> ',' >>
                *qi::double_ >> ',' >>                              // velocity in knot
                'N' >> ',' >>
                *qi::double_[bp::ref(vel) = qi::_1] >> ',' >>       // velocity in km/h
                'K' >> ',' >>
                *qi::int_                                           // mode
            )  
        )){
            vel /= 3.6; // km/h to m/s
            heading = (-heading + 90.0) * M_PI / 180.0;
            //std::printf("VTG : %+3.3f %+3.3f\r\n", vel, heading * 180.0 / M_PI);
            twist.twist.linear.x = vel * std::cos(heading);
            twist.twist.linear.y = vel * std::sin(heading);
            twist.twist.linear.z = 0.0;
            return true;
        }
        return false;
    }

    bool parse_rot(const std::string& words, const ros::Time& stamp){
        double rotation;
        if(qi::parse(
            words.cbegin(),
            words.cend(),
            (
                *qi::double_[bp::ref(rotation) = -qi::_1] >> ',' >>   // rotation velocity in deg/sec
                'A'
            )  
        )){
            //std::printf("ROT : %+3.3f\r\n", rotation / 60.0);
            twist.twist.angular.x = 0.0;
            twist.twist.angular.y = 0.0;
            twist.twist.angular.z = rotation * M_PI / 180.0 / 60.0; // deg / min to rad / sec
            return true;
        }
        return false;
    }

    void config_device(){
        std::vector<std::string> init_commands = {
            "$JASC,GPHPR,10",
            "$JASC,GPGST,1",
            "$JASC,GPGGA,10",
            "$JASC,GPVTG,10",
            "$JASC,GPROT,10",
            "$JASC,GPZDA,1",
            "$JNMEA,GGAALLGNSS,YES",
            "$JATT,COGTAU,0.1",
            "$JATT,HRTAU,0.1",
            "$JATT,HTAU,0.1",
            "$JATT,PTAU,0.1",
            "$JATT,SPDTAU,0.1"
        };
        for(auto&& cmd : init_commands){
            cmd += "\r\n";
            port.write_some(boost::asio::buffer(cmd, cmd.length()));
        }
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
    ros::Publisher pub_time;
    ros::Publisher pub_pose;
    ros::Publisher pub_twist;
    ba::serial_port port;
    ba::io_service& io_service;
    std::thread t;
    std::string dev;
    std::string delim;
    int baud;
    ba::streambuf buffer;
    std::thread io_thread;
    sensor_msgs::NavSatFix navsat_msg;  
    geometry_msgs::PoseStamped pose;
    sensor_msgs::TimeReference time_msg;
    std::string frame_id;
    geometry_msgs::TwistStamped twist;
    ros::Time stamp_vtg;
    ros::Time stamp_rot;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ssv102_ros");
    ROS_INFO("Start ssV102 driver node");
    
    ba::io_service io;
	Ssv102 gps(io);
    
    if(!gps.init()){
        return -1;
    }

    ros::spin();
    gps.close();

	return 0;
}