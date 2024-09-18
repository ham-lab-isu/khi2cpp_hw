/*
Written by KHI, Adapted for ROS 2 by Jakob D. Hamilton

KHI_KRNX_DRIVER_H is the header file for KHI_KRNX_DRIVER. It initializes the methods for communicating with KRNX.h, the API for the Kawasaki controller

*/

#ifndef KHI_KRNX_DRIVER_H
#define KHI_KRNX_DRIVER_H

#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "krnx.h"
#include "khi_robot_driver.h"

namespace khi_robot_control
{
#define KRNX_MSGSIZE 1024
#define KRNX_STDAXES 6
#define KRNX_MOTION_BUF 10
#define KRNX_PRINT_TH 1000

struct KhiRobotKrnxRtcData
{
    int sw;
    int seq_no;
    float comp[KRNX_MAX_ROBOT][KRNX_MAXAXES];
    float old_comp[KRNX_MAX_ROBOT][KRNX_MAXAXES];
    int status[KRNX_MAX_ROBOT][KRNX_MAXAXES];
};

class KhiRobotKrnxDriver : public KhiRobotDriver {
public:
    KhiRobotKrnxDriver();
    ~KhiRobotKrnxDriver();
    bool setState( const int& cont_no, const int& state );

    bool initialize( const int& cont_no, const double& period, KhiRobotData& data, const bool in_simulation = false ) override;
    bool open( const int& cont_no, const std::string& ip_address, KhiRobotData& data ) override;
    bool close( const int& cont_no ) override;
    bool activate( const int& cont_no, KhiRobotData& data ) override;
    bool hold( const int& cont_no, const KhiRobotData& data ) override;
    bool deactivate( const int& cont_no, const KhiRobotData& data ) override;
    bool readData( const int& cont_no, KhiRobotData& data ) override;
    bool writeData( const int& cont_no, const KhiRobotData& data ) override;
    bool updateState( const int& cont_no, const KhiRobotData& data ) override;
    bool getPeriodDiff( const int& cont_no, double& diff ) override;
    //bool commandHandler( khi_robot_msgs::srv::KhiRobotCmd::Request& req, khi_robot_msgs::srv::KhiRobotCmd::Response& res ) override;

private:
    /* general */
    char cmd_buf[KRNX_MSGSIZE];
    char msg_buf[KRNX_MSGSIZE];
    int sw_dblrefflt[KRNX_MAX_CONTROLLER];
    std::mutex mutex_state[KRNX_MAX_CONTROLLER];

    /* RTC */
    KhiRobotKrnxRtcData rtc_data[KRNX_MAX_CONTROLLER];

    bool getCurMotionData( const int& cont_no, const int& robot_no, TKrnxCurMotionData* p_motion_data );
    int execAsMonCmd( const int& cont_no, const char* cmd, char* buffer, int buffer_sz, int* as_err_code );
    bool retKrnxRes( const int& cont_no, const std::string& name, const int& ret, const bool error = true );
    bool conditionCheck( const int& cont_no, const KhiRobotData& data );
    bool setRobotDataHome( const int& cont_no, KhiRobotData& data );
    std::vector<std::string> splitString( const std::string& str, const char& del );
    bool loadDriverParam( const int& cont_no, KhiRobotData& data );
    bool loadRtcProg( const int& cont_no, const std::string& name );
    bool syncRtcPos( const int& cont_no, KhiRobotData& data );
};

} // namespace

#endif // KHI_ROBOT_KRNX_DRIVER_H