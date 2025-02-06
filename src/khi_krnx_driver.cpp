/*
Written by KHI, Adapted for ROS 2 by Jakob D. Hamilton

KHI_KRNX_DRIVER source file defines the methods for communicating with KRNX.h, the API for the Kawasaki controller

*/

#include "geometry_msgs/msg/pose.hpp"
#include "khi_krnx_driver.h"

// JDH I don't know what this does but 
// it has to be here or std_msg goes haywire
using std::placeholders::_1;

// This is the service initialization format
//khi_robot_msgs::srv::KhiRobotCmd::Request req;

namespace khi_robot_control
{
#define KHI_ROBOT_CX165L "CX165L"
#define KHI_ROBOT_CX110L "CX110L"
#define KHI_KRNX_BUFFER_SIZE 4
#define KHI_KRNX_ACTIVATE_TH 0.02
#define KHI_KRNX_M2MM 1000

KhiRobotKrnxDriver::KhiRobotKrnxDriver() : KhiRobotDriver()
{
    driver_name = __func__;
    for ( int cno = 0; cno < KRNX_MAX_CONTROLLER; cno++ )
    {
        rtc_data[cno].seq_no = 0;
        sw_dblrefflt[cno] = 0;
    }
}

KhiRobotKrnxDriver::~KhiRobotKrnxDriver()
{
    int state;

    for ( int cno = 0; cno < KRNX_MAX_CONTROLLER; cno++ )
    {
        state = getState( cno );
        if ( ( state != INIT ) && ( state != DISCONNECTED ) )
        {
            infoPrint("destructor");
            close( cno );
        }
    }
}
bool KhiRobotKrnxDriver::setState( const int& cont_no, const int& state )
{
    bool is_state_set;
    std::lock_guard<std::mutex> lock( mutex_state[cont_no] );

    is_state_set = KhiRobotDriver::setState( cont_no, state );
    return is_state_set;
}

int KhiRobotKrnxDriver::execAsMonCmd( const int& cont_no, const char* cmd, char* buffer, int buffer_sz, int* as_err_code )
{
    std::lock_guard<std::mutex> lock( mutex_state[cont_no] );

    return_code = krnx_ExecMon( cont_no, cmd, buffer, buffer_sz, as_err_code );
    if ( *as_err_code != 0 )
    {
        warnPrint( "AS returned %d by %s", *as_err_code, cmd );
    }
    else
    {
        retKrnxRes( cont_no, "krnx_ExecMon()", return_code );
    }

    return return_code;
}

bool KhiRobotKrnxDriver::retKrnxRes( const int& cont_no, const std::string& name, const int& ret, const bool error )
{
    if ( ret != KRNX_NOERROR )
    {
        RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX ERROR: %s returned -0x%X -------------", name.c_str(), -ret);
        errorPrint( "%s returned -0x%X", name.c_str(), -ret );
        if ( error ) { setState( cont_no, ERROR ); }
        return false;
    }
    else
    {
        return true;
    }
}

/* This function needs some communication time. Don't use this in control loop */
bool KhiRobotKrnxDriver::conditionCheck( const int& cont_no, const KhiRobotData& data )
{
    TKrnxPanelInfo panel_info;
    bool ret = true;

    if ( getState( cont_no ) == ERROR )
    {
        return false;
    }

    if ( in_simulation ) { return true; }

    for ( int ano = 0; ano < data.arm_num; ano++ )
    {
        /* Condition Check */
        return_code = krnx_GetPanelInfo( cont_no, ano, &panel_info );
        if ( !retKrnxRes( cont_no, "krnx_GetPanelInfo", return_code ) )
        {
            ret = false;
        }

        if ( panel_info.repeat_lamp != -1 )
        {
            errorPrint( "Please change Robot Controller's TEACH/REPEAT to REPEAT" );
            ret = false;
        }
        if ( panel_info.teach_lock_lamp != 0 )
        {
            errorPrint( "Please change Robot Controller's TEACH LOCK to OFF" );
            ret = false;
        }
        else if ( panel_info.run_lamp != -1 )
        {
            errorPrint( "Please change Robot Controller's RUN/HOLD to RUN" );
            ret = false;
        }
        else if ( panel_info.emergency != 0 )
        {
            errorPrint( "Please change Robot Controller's EMERGENCY to OFF" );
            ret = false;
        }
    }

    if ( !ret )
    {
        setState( cont_no, ERROR );
    }

    return ret;
}

bool KhiRobotKrnxDriver::initialize( const int& cont_no, const double& period, KhiRobotData& data, const bool in_simulation )
{
    char msg[256] = { 0 };

    // robot info
    //cont_info[cont_no].period = period; JDH
    cont_info[cont_no].period = 4;


    return_code = krnx_GetKrnxVersion( msg, sizeof(msg) );
    infoPrint( msg );

    RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER IS INITIALIZED, PERIOD IS %d -------------", cont_info[cont_no].period);


    this->in_simulation = in_simulation;

    return true;
}

bool KhiRobotKrnxDriver::open( const int& cont_no, const std::string& ip_address, KhiRobotData& data )
{
    char c_ip_address[64] = { 0 };

    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { return false; }

    if ( getState( cont_no ) != INIT )
    {
        warnPrint( "Cannot open cont_no:%d because it is already opend...", cont_no );
        return false;
    }

    if ( in_simulation )
    {
        setState( cont_no, CONNECTING );
        setState( cont_no, INACTIVE );
        RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER IS OPENING IN SIMULATION -------------");
        return true;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER IS OPENING THE REAL-ROBOT INTERFACE -------------");
    }


    setState( cont_no, CONNECTING );
    strncpy( c_ip_address, ip_address.c_str(), sizeof(c_ip_address) );
    infoPrint( "Connecting to real controller: %s", c_ip_address );

    // OPEN THE KRNX API
    RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER OPENING CONTROLLER %d AT %s ------------", cont_no, c_ip_address);
    return_code = krnx_Open( cont_no, c_ip_address );
    RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER HAS BEEN OPENED, RETURN_CODE %d ------------", return_code);
    
    // added for debugging
    //return_code = cont_no;

    if ( return_code == cont_no )
    {
        cont_info[cont_no].ip_address = ip_address;
        if ( !loadDriverParam( cont_no, data ) ) { 
            RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- FAILED TO LOAD KRNX DRIVER PARAMS ------------");
            return false; };

        setState( cont_no, INACTIVE );
        RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER SUCCESSFULLY OPENED REAL ROBOT -------------");
        return true;
    }
    else
    {
        retKrnxRes( cont_no, "krnx_Open", return_code, false );
        setState( cont_no, INIT );
        RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER FAILED TO OPEN REAL ROBOT -------------");
        return false;
    }
}

bool KhiRobotKrnxDriver::close( const int& cont_no )
{
    char msg[1024] = { 0 };

    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { return false; }

    if ( in_simulation )
    {
        setState( cont_no, DISCONNECTING );
        setState( cont_no, DISCONNECTED );
        return true;
    }

    /* Switch Reversion */
    if ( sw_dblrefflt[cont_no] == -1 )
    {
        snprintf( msg, sizeof(msg), "SW ZDBLREFFLT_MODSTABLE=ON" );
        return_code = execAsMonCmd( cont_no, msg, msg_buf, sizeof(msg_buf), &error_code );
    }

    setState( cont_no, DISCONNECTING );
    return_code = krnx_Close( cont_no );
    if ( return_code == KRNX_NOERROR )
    {
        setState( cont_no, DISCONNECTED );
    }
    
    RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER SUCCESSFULLY CLOSED -------------");
    
    return retKrnxRes( cont_no, "krnx_Close", return_code, false );
}

bool KhiRobotKrnxDriver::activate( const int& cont_no, KhiRobotData& data )
{
    const int to_home_vel = 20; /* speed 20 */
    const double timeout_sec_th = 5.0; /* 5 sec */
    bool is_ready;
    TKrnxCurMotionData motion_data = { 0 };
    double timeout_sec_cnt = 0.0;
    int conv = 1;
    float diff = 0;
    int error_lamp = 0;
    TKrnxPanelInfo panel_info;
    TKrnxProgramInfo program_info;
    int arm_num = data.arm_num;

    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { 
        RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER COULD NOT BE ACTIVATED (Breakpoint 1) -------------");
        return false; }

    setState( cont_no, ACTIVATING );
    if ( !conditionCheck( cont_no, data ) ) { 
        RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER COULD NOT BE ACTIVATED (Breakpoint 2) -------------");
        return false; }

    if ( in_simulation )
    {
        setRobotDataHome( cont_no, data );
        setState( cont_no, ACTIVE );
        RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER ACTIVE (IN SIMULATION) -------------");
        return true;
    }

    /* Preparation */
    bool is_cycle = true;
    for ( int ano = 0; ano < arm_num; ano++ )
    {
        return_code = krnx_GetCurErrorLamp( cont_no, ano, &error_lamp );
        if ( retKrnxRes( cont_no, "krnx_GetCurErrorLamp", return_code ) && ( error_lamp != 0 ) )
        {
            /* Error Reset */
            return_code = krnx_Ereset( cont_no, ano, &error_code );
        }
        return_code = krnx_GetPanelInfo( cont_no, ano, &panel_info );
        if ( retKrnxRes( cont_no, "krnx_GetPanelInfo", return_code ) )
        {
            if ( panel_info.cycle_lamp != -1 )
            {
                is_cycle = false;
            }
            if ( panel_info.motor_lamp != -1 )
            {
                /* Motor Power ON */
                return_code = execAsMonCmd( cont_no, "ZPOW ON", msg_buf, sizeof(msg_buf), &error_code );
                /* Error Reset */
                return_code = krnx_Ereset( cont_no, ano, &error_code );
            }
        }
       /* Clear RTC Comp Data */
        return_code = krnx_OldCompClear( cont_no, ano );
    }
    if ( is_cycle )
    {
        for ( int ano = 0; ano < arm_num; ano++ )
        {
            return_code = krnx_GetProgramInfo( cont_no, ano, &program_info );
            if ( retKrnxRes( cont_no, "krnx_GetProgramInfo", return_code ) )
            {
                std::stringstream program;
                program << "rb_rtc" << ano + 1;
                if ( strcmp( program_info.robot[ano].program_name, program.str().c_str() ) == 0 )
                {
                    /* Already ROS Control */
                    setState( cont_no, ACTIVE );
                    return true;
                }
                else
                {
                    /* Not ROS Control */
                    return_code = krnx_Hold( cont_no, ano, &error_code );
                    rclcpp::sleep_for(std::chrono::milliseconds(200));
                }
            }
        }
    }

    /* Sync RTC Position */ // THIS IS WHAT ESTABLISHES DATA.ARM[ANO].HOME[JT]
    if ( !syncRtcPos( cont_no, data ) )
    {
        errorPrint( "Failed to sync position" );
        setState( cont_no, ERROR );
        RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER COULD NOT BE ACTIVATED (Breakpoint 3, RTC Sync failure) -------------");
        return false;
    }

    /* Set HOME position */
    for ( int ano = 0; ano < arm_num; ano++ )
    {
        /* Speed SLOW */
        return_code = krnx_SetMonSpeed( cont_no, ano, to_home_vel, &error_code );
        /* Executing base program */
        std::stringstream program;
        program << "rb_rtc" << ano + 1;
        return_code = krnx_Execute( cont_no, ano, program.str().c_str(), 1, 0, &error_code );

        while ( 1 )
        {
            int sleep_time = static_cast<int>(cont_info[cont_no].period/1e+9);
            rclcpp::sleep_for(std::chrono::seconds(sleep_time));
            timeout_sec_cnt += cont_info[cont_no].period/1e+9;
            if ( timeout_sec_cnt > timeout_sec_th )
            {
                errorPrint( "Failed to activate: timeout" );
                setState( cont_no, ERROR );
                RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER COULD NOT BE ACTIVATED (Breakpoint 4, Timeout failure) -------------");
                return false;
            }

            return_code = krnx_GetRtcSwitch( cont_no, ano, &rtc_data[cont_no].sw );
            if ( ( return_code != KRNX_NOERROR ) || ( rtc_data[cont_no].sw == 0 ) ) { continue; }

            return_code = krnx_GetCurMotionData( cont_no, ano, &motion_data );
            if ( return_code != KRNX_NOERROR ) { 
                RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER GETCURMOTION ERROR (Breakpoint 4.5) -------------");
                continue; }

            is_ready = true;
            for ( int jt = 0; jt < data.arm[ano].jt_num; jt++ )
            {
                data.arm[ano].cmd[jt] = data.arm[ano].pos[jt] = data.arm[ano].home[jt]; // JDH 2024-10-22
                diff = data.arm[ano].home[jt]*conv - motion_data.ang[jt];
                if ( fabs(diff) > KHI_KRNX_ACTIVATE_TH )
                {
                    RCLCPP_WARN(rclcpp::get_logger("KRNX Driver"), "----------- KRNX ROBOT POSITION DIFFERENCE OVER THRESHOLD VALUE (Breakpoint 4.6) ------------");
                    is_ready = false;
                    break;
                }
            }

            // JDH 2024-10-23 debugging rtc comp data errors 
            float comp_limit;
            return_code = krnx_GetRtcCompLimit(cont_no, ano, &comp_limit);
            RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX Arm[%d] RTC Comp Limit: %f -------------", ano, comp_limit);
            // end JDH

            if ( is_ready )
            {
                /* Speed 100 */
                return_code = krnx_SetMonSpeed( cont_no, ano, 100, &error_code );
                break;
            }
        }
    }

    if ( !conditionCheck( cont_no, data ) ) { 
        RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER COULD NOT BE ACTIVATED (Breakpoint 5) -------------");
        return false; }

    setState( cont_no, ACTIVE );
    RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER ACTIVE (REAL ROBOT) -------------");
    return true;
}

bool KhiRobotKrnxDriver::hold( const int& cont_no, const KhiRobotData& data )
{
    int state;
    bool ret = true;

    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { 
        RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER FAILED IN THE 'HOLD' COMMAND -------------");
        return false; }

    state = getState( cont_no );
    if ( state == ACTIVE )
    {
        ret = setState( cont_no, HOLDED );
    }

    return ret;
}

bool KhiRobotKrnxDriver::deactivate( const int& cont_no, const KhiRobotData& data )
{
    char msg[1024] = { 0 };
    int error_lamp = 0;

    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { 
        RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DRIVER FAILED DURING DEACTIVATION -------------");
        return false; }

    if ( in_simulation )
    {
        setState( cont_no, DEACTIVATING );
        setState( cont_no, INACTIVE );
        return true;
    }

    setState( cont_no, DEACTIVATING );

    for ( int ano = 0; ano < data.arm_num; ano++ )
    {
        /* Hold Program */
        return_code = krnx_Hold( cont_no, ano, &error_code );
        rclcpp::sleep_for(std::chrono::milliseconds(200));
        /* Kill Program */
        return_code = krnx_Kill( cont_no, ano, &error_code );
        /* Motor Power OFF */
        return_code = execAsMonCmd( cont_no, "ZPOW OFF", msg_buf, sizeof(msg_buf), &error_code );
        return_code = krnx_GetCurErrorLamp( cont_no, ano, &error_lamp );
        if ( retKrnxRes( cont_no, "krnx_GetCurErrorLamp", return_code ) && ( error_lamp != 0 ) )
        {
            /* Error Reset */
            return_code = krnx_Ereset( cont_no, ano, &error_code );
        }
    }

    setState( cont_no, INACTIVE );

    return true;
}

bool KhiRobotKrnxDriver::loadDriverParam( const int& cont_no, KhiRobotData& data )
{
    RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX PARAM LOAD ------------");
    char robot_name[64] = { 0 };
    char msg[256] = { 0 };
    TKrnxPanelInfo panel_info;
    int jt_num = 0;

    int arm_num = data.arm_num;

    if ( arm_num <= 0 )
    {
        errorPrint( "Invalid robot size" );
        return false;
    }

    for ( int ano = 0; ano < arm_num; ano++ )
    {
        /* Robot Name */
        RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX PARAM LOAD: ano %d; cont_no %d; robot_name %s ------------", ano, cont_no, robot_name);
        return_code = krnx_GetRobotName( cont_no, ano, robot_name );
        
        if ( strncmp( robot_name, data.robot_name.c_str(), 6 ) != 0 )
        {
            RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX ROBOT NAME ERROR %s != %s------------", data.robot_name.c_str(), robot_name);
            errorPrint( "ROS Robot:%s does not match AS:%s", data.robot_name.c_str(), robot_name );
            return false;
        }
        //RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX ROBOT NAME ASSIGNMENT COMPLETE ------------");

        /* AS Switch */
        return_code = execAsMonCmd( cont_no, "TYPE SWITCH(ZDBLREFFLT_MODSTABLE)", msg_buf, sizeof(msg_buf), &error_code );
        if ( retKrnxRes( cont_no, msg, return_code ) )
        {
            sw_dblrefflt[cont_no] = atoi(msg_buf);
            if ( sw_dblrefflt[cont_no] == -1 )
            {
                return_code = execAsMonCmd( cont_no, "SW ZDBLREFFLT_MODSTABLE=OFF", msg_buf, sizeof(msg_buf), &error_code );
            }
        }

        /* Joint num */
        snprintf( msg, sizeof(msg), "TYPE SYSDATA(ZROB.NOWAXIS,%d)", ano+1 );
        return_code = execAsMonCmd( cont_no, msg, msg_buf, sizeof(msg_buf), &error_code );
        if ( retKrnxRes( cont_no, msg, return_code ) )
        {
            jt_num = atoi(msg_buf);
            if ( data.arm[ano].jt_num != jt_num )
            {
                RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"),"ROS JT:%d does not match AS:%d", data.arm[ano].jt_num, jt_num );
            }
        }

        return_code = krnx_GetPanelInfo( cont_no, ano, &panel_info );
        if ( retKrnxRes( cont_no, "krnx_GetPanelInfo", return_code ) && ( panel_info.cycle_lamp != 0 ) )
        {
            // DEBUG
            RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX ERROR AT panel_info.cycle_lamp ------------");
            
            /* Hold Program */
            return_code = krnx_Hold( cont_no, ano, &error_code );
            if ( !retKrnxRes( cont_no, "krnx_Hold", return_code ) ) { return false; }
        }

        /* KRNX */
        TKrnxRtcInfo rtcont_info;
        // JDH rtcont_info.cyc = (int)(cont_info[cont_no].period/1e+6);
        rtcont_info.cyc = (int)(2);
        rtcont_info.buf = KHI_KRNX_BUFFER_SIZE;
        rtcont_info.interpolation = 1;
        RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX rtcont_info: cyc %d, buf %d, int %d ------------", rtcont_info.cyc, rtcont_info.buf, rtcont_info.interpolation);
        return_code = krnx_SetRtcInfo( cont_no, &rtcont_info );
        retKrnxRes( cont_no, "krnx_SetRtcInfo", return_code );
        krnx_SetRtcCompMask( cont_no, ano, pow( 2, data.arm[ano].jt_num ) - 1 );

        /* Kill Program */
        return_code = krnx_Kill( cont_no, ano, &error_code );
        if ( !retKrnxRes( cont_no, "krnx_Kill", return_code ) ) { 
            RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX ERROR: KRNX CONTROLLER KILLING PROGRAM ------------");
            return false; }

        /* Load Program */
        if ( !loadRtcProg( cont_no, data.robot_name.c_str() ) )
        {
            RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "Failed to load RTC program");
            return false;
        }
    }

    return true;
}

/**
 * public read function
 */
bool KhiRobotKrnxDriver::readData( const int& cont_no, KhiRobotData& data )
{
    static int sim_cnt[KHI_MAX_CONTROLLER] = { 0 };
    static KhiRobotData prev_data = data;
    int arm_num = data.arm_num;

    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { return false; }

    if ( in_simulation )
    {
        for ( int ano = 0; ano < arm_num; ano++ )
        {
            memcpy( data.arm[ano].pos, data.arm[ano].cmd, sizeof(data.arm[ano].pos) );
            for ( int jt = 0; jt < data.arm[ano].jt_num; jt++ )
            {
                data.arm[ano].vel[jt] =  data.arm[ano].pos[jt] - prev_data.arm[ano].pos[jt];
            }
        }
        prev_data = data;
        if ( ( sim_cnt[cont_no] - 1 ) % KRNX_PRINT_TH == 0 )
        {
            jointPrint( std::string("read"), data );
        }
        sim_cnt[cont_no]++;
        return true;
    }

    static std::vector<TKrnxCurMotionData> motion_data[KRNX_MAX_CONTROLLER][KRNX_MAX_ROBOT];
    TKrnxCurMotionData motion_cur[KRNX_MAX_ROBOT];
    float ang[KRNX_MAX_ROBOT][KRNX_MAXAXES] = {{ 0 }};
    float vel[KRNX_MAX_ROBOT][KRNX_MAXAXES] = {{ 0 }};

    for ( int ano = 0; ano < arm_num; ano++ )
    {
        if ( !getCurMotionData( cont_no, ano, &motion_cur[ano] ) ) {
            RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX DID NOT READ CURRENT MOTION DATA ------------");
            return false; }

        if ( motion_data[cont_no][ano].size() >= KRNX_MOTION_BUF )
        {
            motion_data[cont_no][ano].erase( motion_data[cont_no][ano].begin() );
        }
        motion_data[cont_no][ano].push_back( motion_cur[ano] );

        // ang
        memcpy( ang[ano], &motion_cur[ano].ang, sizeof(motion_cur[ano].ang) );
        // vel
        if ( motion_data[cont_no][ano].size() > 1 )
        {
            std::vector<TKrnxCurMotionData>::iterator it = motion_data[cont_no][ano].end();
            it--;
            for ( int jt=0; jt < KHI_MAX_JOINT; jt++ )
            {
                vel[ano][jt] = motion_data[cont_no][ano].back().ang[jt] - it->ang[jt];
            }
        }

        for ( int jt = 0; jt < data.arm[ano].jt_num; jt++ )
        {
            data.arm[ano].pos[jt] = (double)ang[ano][jt];
            data.arm[ano].vel[jt] = (double)vel[ano][jt];
            data.arm[ano].eff[jt] = (double)0; // tmp
        }
    }

    return true;
}

bool KhiRobotKrnxDriver::getCurMotionData( const int& cont_no, const int& robot_no, TKrnxCurMotionData* p_motion_data )
{
    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { return false; }

    return_code = krnx_GetCurMotionData( cont_no, robot_no, p_motion_data );

    return retKrnxRes( cont_no, "krnx_GetCurMotionData", return_code );
}

bool KhiRobotKrnxDriver::setRobotDataHome( const int& cont_no, KhiRobotData& data )
{
    KhiRobotData base;
    int arm_num = data.arm_num;

    if ( data.robot_name == "null robot" )
    {
        data.arm[0].home[0] = -45.0f*M_PI/180.0;
        data.arm[0].home[1] = 45.0f*M_PI/180.0;
        data.arm[1].home[0] = 45.0f*M_PI/180.0;
        data.arm[1].home[1] = -45.0f*M_PI/180.0;
        data.arm[0].home[2] = data.arm[1].home[2] = 90.0f/KHI_KRNX_M2MM;
        data.arm[0].home[3] = data.arm[1].home[3] = 0.0f;
    }
    else
    {
        for ( int ano = 0; ano < arm_num; ano++ )
        {
            for ( int jt = 0; jt < data.arm[ano].jt_num; jt++ )
            {
                data.arm[ano].home[jt] = 0.0f;
            }
        }
    }

    for ( int ano = 0; ano < arm_num; ano++ )
    {
        for ( int jt = 0; jt < data.arm[ano].jt_num; jt++ )
        {
            data.arm[ano].cmd[jt] = data.arm[ano].pos[jt] = data.arm[ano].home[jt];
        }
    }

    return true;
}

/**
 * public write function
 */
bool KhiRobotKrnxDriver::writeData( const int& cont_no, const KhiRobotData& data )
{
    static int sim_cnt[KHI_MAX_CONTROLLER] = { 0 };
    int idx, ano, jt;
    char msg[1024] = { 0 };
    char status[128] = { 0 };
    TKrnxCurMotionData motion_data;
    float jt_pos = 0.0F;
    float jt_vel = 0.0F;
    bool is_primed = true;
    int arm_num = data.arm_num;
    KhiRobotKrnxRtcData* p_rtc_data = &rtc_data[cont_no];

    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { return false; }

    if ( getState( cont_no ) != ACTIVE ) { return true; }

    if ( in_simulation )
    {
        if ( ( sim_cnt[cont_no] - 1 ) % KRNX_PRINT_TH == 0 )
        {
            jointPrint( std::string("write"), data );
        }
        //RCLCPP_DEBUG(rclcpp::get_logger("KRNX Driver"), "----------- KRNX RTC: %f -------------", data.arm[0].pos[0]);
        sim_cnt[cont_no]++;
        return true;
    }

    /* convert */
    for ( int ano = 0; ano < arm_num; ano++ )
    {
        for ( int jt = 0; jt < data.arm[ano].jt_num; jt++ )
        {
            // JDH this appears to be the critical line - using the pointer p_rtc_data to assign new Krnx data from the object KhiRobotData(KhiRobotArmData)
            p_rtc_data->comp[ano][jt] = (float)((data.arm[ano].cmd[jt] - data.arm[ano].home[jt]));

            // adding some comp limits just to get it to run
            //if (p_rtc_data->comp[ano][jt] > 0.004) {p_rtc_data->comp[ano][jt] = 0.004;};
            //if (p_rtc_data->comp[ano][jt] < -0.004) {p_rtc_data->comp[ano][jt] = -0.004;};
            //

            //RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX data.arm[%d].cmd[%d] = %f, comp[%d][%d] = %f -------------", ano, jt, data.arm[ano].cmd[jt], ano, jt, p_rtc_data->comp[ano][jt]);
        }
    }

    for ( int ano = 0; ano < arm_num; ano++ )
    {
        //RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX PrimeRtcCompData Args: %d, %d, %p, %p", cont_no, ano, &p_rtc_data->comp[ano][0], &p_rtc_data->status[ano][0] );
        return_code = krnx_PrimeRtcCompData( cont_no, ano, &p_rtc_data->comp[ano][0], &p_rtc_data->status[ano][0] );
        //RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX PrimeRtcCompData Status: %i --------------", status[ano]);
        if ( !retKrnxRes( cont_no, "krnx_PrimeRtcCompData", return_code ) ) { is_primed = false; }
    }
    if ( !is_primed )
    {
        /* ERROR log */
        for ( int ano = 0; ano < arm_num; ano++ )
        {
            snprintf( msg, sizeof(msg), "[krnx_PrimeRtcCompData] ano:%d [jt]pos:vel:status ", ano+1 );
            krnx_GetRtcCompData( cont_no, ano, &p_rtc_data->old_comp[ano][0] );
            getCurMotionData( cont_no, ano, &motion_data );
            for ( int jt = 0; jt < data.arm[ano].jt_num; jt++ )
            {
                jt_pos = motion_data.ang_ref[jt];
                jt_vel = ( p_rtc_data->comp[ano][jt] - p_rtc_data->old_comp[ano][jt] )*(1e+9/cont_info[cont_no].period);

                snprintf( status, sizeof(status), "[%d]%.4f:%.4f:%d ", jt+1, jt_pos, jt_vel, p_rtc_data->status[ano][jt] );
                strcat( msg, status );
                //RCLCPP_WARN(rclcpp::get_logger("krnx_logger"), "JT%d:%f,%f,%f,%f,%f,%f", jt+1, data.arm[ano].cmd[jt], data.arm[ano].home[jt]+p_rtc_data->comp[ano][jt],p_rtc_data->old_comp[ano][jt], p_rtc_data->comp[ano][jt], data.arm[ano].home[jt], motion_data.ang_ref[jt]);
                //RCLCPP_WARN(rclcpp::get_logger("krnx_logger"), "JT%d:%f,%f,%f,%f,%f,%f", jt+1, data.arm[ano].cmd[jt]*180.0/M_PI, (data.arm[ano].home[jt]+p_rtc_data->comp[ano][jt])*180.0/M_PI, p_rtc_data->old_comp[ano][jt]*180.0/M_PI, p_rtc_data->comp[ano][jt]*180.0/M_PI, data.arm[ano].home[jt]*180.0/M_PI, motion_data.ang_ref[jt]*180.0/M_PI);
            }
            errorPrint( msg );
        }
        return false;
    }

    // JDH Is this what sends the RTC movement data to the hardware (i.e. controller)? YES, see KRNX API manual p77
    //infoPrint("JDH SENDING RTC COMP DATA");
    return_code = krnx_SendRtcCompData( cont_no, p_rtc_data->seq_no );
    p_rtc_data->seq_no++;

    return retKrnxRes( cont_no, "krnx_SendRtcCompData", return_code );
}

bool KhiRobotKrnxDriver::updateState( const int& cont_no, const KhiRobotData& data )
{
    int error_lamp = 0;
    int error_code = 0;
    int state;
    TKrnxPanelInfo panel_info;
    int arm_num = data.arm_num;

    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { return false; }

    state = getState( cont_no );
    if ( state == ERROR )
    {
        for ( int ano = 0; ano < arm_num; ano++ )
        {
            return_code = krnx_GetPanelInfo( cont_no, ano, &panel_info );
            if ( retKrnxRes( cont_no, "krnx_GetPanelInfo", return_code ) && ( panel_info.cycle_lamp != 0 ) )
            {
                /* Hold Program */
                return_code = krnx_Hold( cont_no, ano, &error_code );
                if ( !retKrnxRes( cont_no, "krnx_Hold", return_code ) ) { return false; }
            }
        }
    }

    if ( in_simulation ) { return true; }

    for ( int ano = 0; ano < arm_num; ano++ )
    {
        return_code = krnx_GetCurErrorLamp( cont_no, ano, &error_lamp );
        if ( ( state != ERROR ) && ( error_lamp != 0 ) )
        {
            return_code = krnx_GetCurErrorInfo( cont_no, ano, &error_code );
            errorPrint( "AS ERROR %d: ano:%d code:%d", cont_no, ano+1, error_code );
            setState( cont_no, ERROR );
            return true;
        }
        return_code = krnx_GetRtcSwitch( cont_no, ano, &rtc_data[cont_no].sw );
        if ( ( state != INACTIVE ) && ( rtc_data[cont_no].sw == 0 ) )
        {
            errorPrint( "RTC SWITCH turned OFF %d: ano:%d", cont_no, ano+1 );
            deactivate( cont_no, data );
            return true;
        }
    }

    return true;
}

bool KhiRobotKrnxDriver::getPeriodDiff( const int& cont_no, double& diff )
{
    if ( !contLimitCheck( cont_no, KRNX_MAX_CONTROLLER ) ) { return false; }
    static bool buffer_enabled = false;
    int ano = 0;
    int buffer_length = 0;

    if ( getState( cont_no ) != ACTIVE )
    {
        diff = 0;
        return true;
    }

    if ( in_simulation )
    {
        diff = 0;
        return true;
    }

    buffer_length = krnx_GetRtcBufferLength( cont_no, ano );
    // check if KRNX can get buffer length from KHI controller
    if ( buffer_length > 0 )
    {
        buffer_enabled = true;
    }

    if ( buffer_enabled )
    {
        diff = ( buffer_length - KHI_KRNX_BUFFER_SIZE ) * cont_info[cont_no].period;
    }
    else
    {
        diff = 0;
    }

    return true;
}

std::vector<std::string> KhiRobotKrnxDriver::splitString( const std::string& str, const char& del )
{
    int first = 0;
    int last = str.find_first_of( del );
    std::vector<std::string> list;

    if ( first < str.size() )
    {
        std::string sub_str1( str, first, last - first );
        list.push_back( sub_str1 );
        first = last + 1;
        std::string sub_str2( str, first, std::string::npos );
        list.push_back( sub_str2 );
    }

    return list;
}

bool KhiRobotKrnxDriver::loadRtcProg( const int& cont_no, const std::string& name )
{
    FILE* fp;
    int fd;
    char tmplt[] = "/tmp/khi_robot-rtc_param-XXXXXX";
    char fd_path[128] = { 0 };
    char file_path[128] = { 0 };
    ssize_t rsize;

    fd = mkstemp(tmplt);

    if ( ( fp = fdopen( fd, "w" ) ) != NULL)
    {
        /* retrieve path */
        snprintf( fd_path, sizeof(fd_path), "/proc/%d/fd/%d", getpid(), fd );
        rsize = readlink( fd_path, file_path, sizeof(file_path) );
        if ( rsize < 0 ) { return false; }

        /* RTC program */
        if ( name == "null robot" )
        {
            fprintf( fp, ".PROGRAM rb_rtc1()\n" );
            fprintf( fp, "  HERE #rtchome1\n" );
            fprintf( fp, "  FOR .i = 1 TO 8\n" );
            fprintf( fp, "    .acc[.i] = 0\n" );
            fprintf( fp, "  END\n" );
            fprintf( fp, "  L3ACCURACY .acc[1] ALWAYS\n" );
            fprintf( fp, "  RTC_SW 1: ON\n" );
            fprintf( fp, "1 JMOVE #rtchome1\n" );
            fprintf( fp, "  GOTO 1\n" );
            fprintf( fp, "  RTC_SW 1: OFF\n" );
            fprintf( fp, ".END\n" );
            fprintf( fp, ".PROGRAM rb_rtc2()\n" );
            fprintf( fp, "  HERE #rtchome2\n" );
            fprintf( fp, "  FOR .i = 1 TO 8\n" );
            fprintf( fp, "    .acc[.i] = 0\n" );
            fprintf( fp, "  END\n" );
            fprintf( fp, "  L3ACCURACY .acc[1] ALWAYS\n" );
            fprintf( fp, "  RTC_SW 2: ON\n" );
            fprintf( fp, "1 JMOVE #rtchome2\n" );
            fprintf( fp, "  GOTO 1\n" );
            fprintf( fp, "  RTC_SW 2: OFF\n" );
            fprintf( fp, ".END\n" );
        }
        else
        {
            fprintf( fp, ".PROGRAM rb_rtc1()\n" );
            fprintf( fp, "  HERE #rtchome1\n" );;
            fprintf( fp, "  ACCURACY 0 ALWAYS\n" );
            fprintf( fp, "  RTC_SW 1: ON\n" );
            fprintf( fp, "1 JMOVE #rtchome1\n" );
            fprintf( fp, "  GOTO 1\n" );
            fprintf( fp, "  RTC_SW 1: OFF\n" );
            fprintf( fp, ".END\n" );
        }
        fclose( fp );
    }
    else
    {
        return false;
    }

    return_code = krnx_Load( cont_no, file_path );
    unlink( file_path );
    if ( !retKrnxRes( cont_no, "krnx_Load", return_code ) ) { return false; }

    return true;
}

bool KhiRobotKrnxDriver::syncRtcPos( const int& cont_no, KhiRobotData& data )
{
    TKrnxCurMotionData motion_data = { 0 };

    for ( int ano = 0; ano < data.arm_num; ano++ )
    {
        /* Driver */
        if ( !getCurMotionData( cont_no, ano, &motion_data ) ) { return false; }
        for ( int jt = 0; jt < data.arm[ano].jt_num; jt++ )
        {
            data.arm[ano].home[jt] = (double)motion_data.ang[jt];
            //RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX HOME ASSIGNMENT for JT=%d is %f", jt, data.arm[ano].home[jt]);
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("KRNX Driver"), "----------- KRNX ASSIGNED MOTION DATA TO KhiRobotData OBJECT HOME --------------");
    return true;
}

// I removed the source code for the commandHandler

} // end of khi_robot_control namespace
