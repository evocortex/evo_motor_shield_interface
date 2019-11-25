//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file MotorShieldTestNode.cpp
 * @author MBA (info@evocortex.com)
 *
 * @brief Motor Shield Test Node
 *
 * @version 1.0
 * @date 2019-10-30
 *
 * @copyright Copyright (c) 2019 Evocortex GmbH
 *
 */

/* Includes ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include <evo_mbed/tools/com/ComServer.h>
#include <evo_motor_shield_interface/MotorShield.h>
#include <evo_mbed/tools/Logging.h>

#include <csignal>

using namespace evo_mbed;
/*--------------------------------------------------------------------------------*/

/**
 * @brief Motor configuration parameters
 */
class ROSMotor
{
 public:
   void init(const unsigned int abs_motor_id, ros::NodeHandle& nh,
             std::shared_ptr<Motor>& motor)
   {
      _motor = motor;

      const std::string prefix = "Motor" + std::to_string(abs_motor_id) + "/";

      _pub_operation_status = nh.advertise<std_msgs::String>(prefix + "OpStatus", 1);
      _pub_position = nh.advertise<std_msgs::Float32>(prefix + "Position", 1);
      _pub_speed    = nh.advertise<std_msgs::Float32>(prefix + "Speed", 1);

      _sub_tgt_status = nh.subscribe<std_msgs::UInt8>(
          prefix + "SetOpStatus", 1, &ROSMotor::subOperationStatus, this);
      _sub_tgt_pwm      = nh.subscribe<std_msgs::Float32>(prefix + "SetTgtPWM", 1,
                                                     &ROSMotor::subTgtPWM, this);
      _sub_tgt_speed    = nh.subscribe<std_msgs::Float32>(prefix + "SetTgtSpeed", 1,
                                                       &ROSMotor::subTgtSpeed, this);
      _sub_tgt_position = nh.subscribe<std_msgs::Float32>(
          prefix + "SetTgtPosition", 1, &ROSMotor::subTgtPosition, this);
   }

   void update(void)
   {
      std_msgs::String msg_str;
      switch(_motor->getOperationStatus())
      {
      case MOTOR_STS_DISABLED: msg_str.data = "Disabled"; break;
      case MOTOR_STS_ENABLED: msg_str.data = "Enabled"; break;
      }
      _pub_operation_status.publish(msg_str);

      std_msgs::Float32 msg_float;

      // Check type
      switch(_motor->getType())
      {
      case MOTOR_TYPE_LIFT: { msg_float.data = _motor->getPositionMM();
      }
      break;

      case MOTOR_TYPE_DRIVE:
      {
         msg_float.data = _motor->getRevolutions();

         if(msg_float.data > 10.0f)
         {
            _motor->resetRevs(0.0f);
         }
      }
      break;

      default: { msg_float.data = 0.0f;
      }
      break;
      }

      _pub_position.publish(msg_float);

      msg_float.data = _motor->getSpeedRPM();
      _pub_speed.publish(msg_float);
   }

   void subOperationStatus(const std_msgs::UInt8::ConstPtr& msg)
   {
      MotorStatus status = MOTOR_STS_DISABLED;

      switch(msg->data)
      {
      case 0: status = MOTOR_STS_DISABLED; break;
      case 1: status = MOTOR_STS_ENABLED; break;
      default: status = MOTOR_STS_DISABLED; break;
      }

      _motor->setOperationStatus(status);
   }

   void subTgtPWM(const std_msgs::Float32::ConstPtr& msg)
   {
      _motor->setTargetPWM(msg->data);
   }

   void subTgtSpeed(const std_msgs::Float32::ConstPtr& msg)
   {
      _motor->setTargetSpeed(msg->data);
   }

   void subTgtPosition(const std_msgs::Float32::ConstPtr& msg)
   {
      _motor->setTargetPosition(msg->data);
   }

   ros::Publisher _pub_operation_status; //!< Operation status publisher
   ros::Publisher _pub_position;         //!< Position publisher
   ros::Publisher _pub_speed;            //!< Speed publisher

   ros::Subscriber _sub_tgt_status;   //!< Target operation status subscriber
   ros::Subscriber _sub_tgt_pwm;      //!< Target pwm subscriber
   ros::Subscriber _sub_tgt_speed;    //!< Target speed subscriber
   ros::Subscriber _sub_tgt_position; //!< Target position subscriber

   int _type                        = 0;    //!< Type of the drive
   int _control_mode                = 0;    //!< Control mode of the drive
   float _pwm_max                   = 0.0f; //!< Maximum allowed PWM value
   float _gear_ratio                = 0.0f; //!< Gear ratio of the drive
   int _encoder_resolution          = 0;    //!< Resolution of the encoder
   float _mm_per_rev                = 0.0f; //!< MM per wheel revolution
   float _lift_conv_fac_mm_per_tick = 0.0f; //!< Conversion factor mm per ADC tick
   float _lift_offs_mm              = 0.0f; //!< Offset of the lift drive in mm
   float _position_kp               = 0.0f; //!< Position controller Kp value
   float _position_ki               = 0.0f; //!< Position controller Ki value
   float _position_kd               = 0.0f; //!< Position controller Kd value
   float _speed_kp                  = 0.0f; //!< Speed controller Kp value
   float _speed_ki                  = 0.0f; //!< Speed controller Ki value
   float _speed_kd                  = 0.0f; //!< Speed controller Kd value

   std::shared_ptr<Motor> _motor; //!< Pointer to motor instance
};

/**
 * @brief Application Class
 */
class App
{
 public:
   /** \brief Default constructor */
   App(ros::NodeHandle& nh);

   /** \brief Default destructor */
   ~App(void);

   /**
    * @brief Initializes the app
    *        Initializes com server, reads config and initialize motor shields
    *
    * @return true Success
    * @return false Error
    */
   const bool init(void);

   /**
    * @brief Runs the app (blocking mode)
    *
    * @return true App exits without error
    * @return false App exits due to an error
    */
   const bool run(void);

 private:
   /** \brief Reads general config from ros param server */
   void readGeneralConfig(void);

   /** \brief Read motor shield config from ros param server */
   void readMotorShieldConfig(void);

   /** \brief Initializes the motorshields */
   const bool initMotorShields(void);

   ros::NodeHandle& _nh; //!< Node handle

   /** \brief Communication server */
   std::shared_ptr<ComServer> _com_server =
       std::shared_ptr<ComServer>(new ComServer());

   /** \brief List containing all config data for every motor */
   std::vector<ROSMotor> _motor_list;

   /** \brief Vector containing all motor shields */
   std::vector<std::shared_ptr<MotorShield>> _motor_shield_list;

   /* General config */
   std::string _can_if_name = "slcan0"; //!< CAN interface name
   int _com_first_id        = 1u;       //!< First communication ID
   int _num_shields         = 1u;       //!< Num motor shields attached
   int _com_timeout_ms = 10u; //!< Communication timeout threshold in milliseconds
};

App::App(ros::NodeHandle& nh) : _nh(nh) {}

App::~App(void)
{
   for(auto& shield : _motor_shield_list)
   {
      shield->release();
   }

   _com_server->release();
}

const bool App::init(void)
{
   readGeneralConfig();

   if(RES_OK != _com_server->init(_can_if_name, 200))
   {
      return false;
   }

   readMotorShieldConfig();

   if(!initMotorShields())
      return false;

   return true;
}

const bool App::run(void)
{
   ros::Rate loop_rate(50.0);

   while(ros::ok())
   {
      ros::spinOnce();

      for(auto shield : _motor_shield_list)
      {
         const MotorShieldState state = shield->getState();

         switch(state)
         {
         case MOTOR_SHIELD_STS_ERR:
         {
            ROS_ERROR("Shield is in error state shutting down!");
            std::raise(SIGTERM);
         }
         break;

         case MOTOR_SHIELD_SYNC_ERR: { shield->resyncShield();
         }
         break;

         case MOTOR_SHIELD_TIMEOUT: ROS_INFO("Timeout!"); break;

         default: break;
         }
      }

      for(auto motor : _motor_list)
      {
         motor.update();
      }

      loop_rate.sleep();
   }

   return true;
}

void App::readGeneralConfig(void)
{
   _nh.getParam("CAN", _can_if_name);
   _nh.getParam("FirstID", _com_first_id);
   _nh.getParam("NumControllers", _num_shields);
   _nh.getParam("TimeoutMS", _com_timeout_ms);
}

void App::readMotorShieldConfig(void)
{
   for(auto idx = 0u; idx < (_num_shields * 2); idx++)
   {
      ROSMotor motor;

      const std::string drive_name =
          "/Motor" + std::to_string(_com_first_id + idx) + "/";
      std::cout << "Load config: " << drive_name << std::endl;
      _nh.getParam(drive_name + "Type", motor._type);
      _nh.getParam(drive_name + "ControlMode", motor._control_mode);
      _nh.getParam(drive_name + "PWMMax", motor._pwm_max);
      _nh.getParam(drive_name + "GearRatio", motor._gear_ratio);
      _nh.getParam(drive_name + "EncoderResolution", motor._encoder_resolution);
      _nh.getParam(drive_name + "MMPerRev", motor._mm_per_rev);
      _nh.getParam(drive_name + "LiftConvFacMMPerTick",
                   motor._lift_conv_fac_mm_per_tick);
      _nh.getParam(drive_name + "LiftOffsMM", motor._lift_offs_mm);
      _nh.getParam(drive_name + "PosKp", motor._position_kp);
      _nh.getParam(drive_name + "PosKi", motor._position_ki);
      _nh.getParam(drive_name + "PosKd", motor._position_kd);
      _nh.getParam(drive_name + "SpeedKp", motor._speed_kp);
      _nh.getParam(drive_name + "SpeedKi", motor._speed_ki);
      _nh.getParam(drive_name + "SpeedKd", motor._speed_kd);

      _motor_list.push_back(motor);
   }
}

const bool App::initMotorShields(void)
{
   unsigned int drive_idx = 0u;

   for(auto idx = 0u; idx < _num_shields; idx++)
   {
      std::shared_ptr<MotorShield> shield(
          new MotorShield(_com_first_id + idx, _com_server, 30.0, true));

      if(!shield->init())
         return false;

      if(!shield->setComTimeout(_com_timeout_ms))
         return false;

      // Config drives
      for(auto idy = 0u; idy < MOTOR_SHIELD_DRIVES; idy++)
      {
         auto drive  = shield->getMotor(idy);
         auto& motor = _motor_list[drive_idx];

         if(!drive->setType(static_cast<MotorType>(motor._type)))
            return false;
         if(!drive->setPWMLimit(motor._pwm_max))
            return false;
         if(!drive->setControlMode(
                static_cast<MotorControlMode>(motor._control_mode)))
            return false;
         if(!drive->setGearRatio(motor._gear_ratio))
            return false;
         if(!drive->setEncoderResolution(motor._encoder_resolution))
            return false;
         if(!drive->setConvFacAdcMMPerTick(motor._lift_conv_fac_mm_per_tick))
            return false;
         if(!drive->setOffsAdcMM(motor._lift_offs_mm))
            return false;

         if(!drive->setPositionKp(motor._position_kp))
            return false;
         if(!drive->setPositionKi(motor._position_ki))
            return false;
         if(!drive->setPositionKd(motor._position_kd))
            return false;

         if(!drive->setSpeedKp(motor._speed_kp))
            return false;
         if(!drive->setSpeedKi(motor._speed_ki))
            return false;
         if(!drive->setSpeedKd(motor._speed_kd))
            return false;

         // Init node for ros
         motor.init(_com_first_id + drive_idx, _nh, drive);

         drive_idx++;
      }

      _motor_shield_list.push_back(shield);
   }

   return true;
}

int main(int argc, char* argv[])
{
   ros::init(argc, argv, "evo_motor_shield_test_node");
   ros::NodeHandle node_handle("~");

   App app(node_handle);

   if(!app.init())
      return -1;

   if(!app.run())
      return -2;

   return 0;
}