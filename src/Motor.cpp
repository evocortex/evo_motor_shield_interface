//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file Motor.cpp
 * @author MBA (info@evocortex.com)
 *
 * @brief Source of Motor
 *
 * @version 1.0
 * @date 2019-04-17
 *
 * @copyright Copyright (c) 2019
 *
 */

/* Includes ----------------------------------------------------------------------*/
#include <evo_motor_shield_interface/Motor.h>
#include <evo_motor_shield_interface/MotorShield.h>
#include <evo_mbed/tools/Logging.h>
/*--------------------------------------------------------------------------------*/

using namespace evo_mbed;

/* Public Class Functions --------------------------------------------------------*/

Motor::~Motor(void)
{
   release();
}

const bool Motor::setType(const MotorType type)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   if(MOTOR_TYPE_NONE != type && MOTOR_TYPE_LIFT != type && MOTOR_TYPE_DRIVE != type)
   {
      LOG_ERROR("Type: " << +type << " not known!");
      return false;
   }

   // Lock CRC calculation
   std::lock_guard<std::mutex> guard(_motor_shield._config_mutex);

   _type = static_cast<uint8_t>(type);

   if(!_motor_shield.writeDataObject(_type, "Motor Type"))
      return false;

   return true;
}

const bool Motor::setPWMLimit(const float pwm_max_value)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   // Lock CRC calculation
   std::lock_guard<std::mutex> guard(_motor_shield._config_mutex);

   _pwm_max_value = pwm_max_value;

   if(!_motor_shield.writeDataObject(_pwm_max_value, "PWM Limit"))
      return false;

   return true;
}

const bool Motor::setMaxSpeedRPM(const float speed_max_rpm)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   // Only positive values are allowed (absolute speed limit)
   if(speed_max_rpm < 0.0)
   {
      LOG_ERROR("Maximum speed is an absolute value. " 
                "Limit must be greater or equal 0.0 rpm!");
      return false;
   }

   // CRC lock not necessarry -> only local parameter

   // Set speed limit
   _speed_max_rpm = speed_max_rpm;

   // Check if current speed value has to be limited
   if(fabs(static_cast<float>(_tgt_speed)) > _speed_max_rpm)
      return setTargetSpeed(_speed_max_rpm);

   return true;
}

const bool Motor::setControlMode(const MotorControlMode mode)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   if(MOTOR_CTRL_NONE != mode && MOTOR_CTRL_PWM != mode && MOTOR_CTRL_POS != mode &&
      MOTOR_CTRL_SPEED != mode)
   {
      LOG_ERROR("Control mode: " << +mode << " not known!");
      return false;
   }

   // Lock CRC calculation
   std::lock_guard<std::mutex> guard(_motor_shield._config_mutex);

   _ctrl_mode = static_cast<uint8_t>(mode);

   if(!_motor_shield.writeDataObject(_ctrl_mode, "Control Mode"))
      return false;

   return true;
}

const bool Motor::setGearRatio(const float gear_ratio)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   if(gear_ratio < 0.0f)
   {
      LOG_ERROR("Gear ratio < 0.0 is not supported!");
      return false;
   }

   // Lock CRC calculation
   std::lock_guard<std::mutex> guard(_motor_shield._config_mutex);

   _gear_ratio = gear_ratio;

   if(!_motor_shield.writeDataObject(_gear_ratio, "Gear Ratio"))
      return false;

   return true;
}

const bool Motor::setEncoderResolution(const uint16_t encoder_reso)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   // Lock CRC calculation
   std::lock_guard<std::mutex> guard(_motor_shield._config_mutex);

   _encoder_resolution = encoder_reso;

   if(!_motor_shield.writeDataObject(_encoder_resolution, "Encoder Resolution"))
      return false;

   return true;
}

const bool Motor::setConvFacAdcMMPerTick(const float conv_fac_adc_mm_per_tick)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   // Lock CRC calculation
   std::lock_guard<std::mutex> guard(_motor_shield._config_mutex);

   _conv_fac_adc_mm_per_tick = conv_fac_adc_mm_per_tick;

   if(!_motor_shield.writeDataObject(_conv_fac_adc_mm_per_tick,
                                     "Factor mm/ADCTick for lift drive"))
      return false;

   return true;
}

const bool Motor::setOffsAdcMM(const float offs_adc_mm)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   _offs_adc_mm = offs_adc_mm;

   if(!_motor_shield.writeDataObject(_offs_adc_mm, "Offset mm for lift drive"))
      return false;

   return true;
}

const bool Motor::setPositionKp(const float kp)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   _position_kp = kp;

   if(!_motor_shield.writeDataObject(_position_kp, "Position Kp"))
      return false;

   return true;
}

const bool Motor::setPositionKi(const float ki)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   _position_ki = ki;

   if(!_motor_shield.writeDataObject(_position_ki, "Position Ki"))
      return false;

   return true;
}

const bool Motor::setPositionKd(const float kd)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   _position_kd = kd;

   if(!_motor_shield.writeDataObject(_position_kd, "Position Kd"))
      return false;

   return true;
}

const bool Motor::setSpeedKp(const float kp)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   _speed_kp = kp;

   if(!_motor_shield.writeDataObject(_speed_kp, "Speed Kp"))
      return false;

   return true;
}

const bool Motor::setSpeedKi(const float ki)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   _speed_ki = ki;

   if(!_motor_shield.writeDataObject(_speed_ki, "Speed Ki"))
      return false;

   return true;
}

const bool Motor::setSpeedKd(const float kd)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   _speed_kd = kd;

   if(!_motor_shield.writeDataObject(_speed_kd, "Speed Kd"))
      return false;

   return true;
}

const bool Motor::resetRevs(const float init_revs)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   if(MOTOR_SHIELD_STS_OK != _motor_shield.getState())
   {
      return false;
   }

   _reset_revs = init_revs;

   if(!_motor_shield.writeDataObject(_reset_revs, "Reset Revolutions"))
      return false;

   return true;
}

const bool Motor::setOperationStatus(const MotorStatus state)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   if(MOTOR_SHIELD_STS_OK != _motor_shield.getState())
   {
      if(MOTOR_STS_DISABLED == state)
         return true;
      return false;
   }

   if(MOTOR_STS_DISABLED != state && MOTOR_STS_ENABLED != state)
   {
      LOG_ERROR("Uknown operation status!");
      return false;
   }

   // Lock status
   std::lock_guard<std::mutex> guard(_status_mutex);

   _status = static_cast<uint8_t>(state);

   if(!_motor_shield.writeDataObject(_status, "Operation Status"))
      return false;

   return true;
}

const bool Motor::setTargetPWM(const float value)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   if(MOTOR_SHIELD_STS_OK != _motor_shield.getState())
   {

      _tgt_pwm = 0.0f;

      return false;
   }

   if(static_cast<MotorStatus>((uint8_t) _status) != MOTOR_STS_ENABLED)
   {
      LOG_WARN("Cannot set pwm -> Drive is disabled!");
      return false;
   }

   if(_tgt_pwm.getDataUpdated())
      return true;
   if(value == (float) (_tgt_pwm))
      return true;

   _tgt_pwm = value;
   _tgt_pwm.setDataUpdated();

   return true;
}
   /* REV
   *  MMA ERROR: setTargetSpeedRPM would be precise. rad/s could be an alternative..
   *  
   */
const bool Motor::setTargetSpeed(const float value)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   if(MOTOR_SHIELD_STS_OK != _motor_shield.getState())
   {
      
      _tgt_speed = 0.0f;

      return false;
   }

   if(static_cast<MotorStatus>((uint8_t) _status) != MOTOR_STS_ENABLED)
   {
      LOG_WARN("Cannot set speed -> Drive is disabled!");
      return false;
   }

   // Limit speed -> only for type drive
   if(static_cast<MotorType>((uint8_t)_type) == MOTOR_TYPE_DRIVE)
   {
      if(fabsf(value) > _speed_max_rpm)
      {
         // Generate warning -> no error
         // Old speed is maintained

         LOG_WARN("Requested speed " << value << " rpm is higher than speed limit "
                   << _speed_max_rpm << " rpm");
         return false;
      }
   }

   if(_tgt_speed.getDataUpdated())
      return true;
   if(value == (float) (_tgt_speed))
      return true;

   _tgt_speed = value;
   _tgt_speed.setDataUpdated();

   return true;
}

const bool Motor::setTargetPosition(const float value)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   if(MOTOR_SHIELD_STS_OK != _motor_shield.getState())
   {

      _tgt_position = 0.0f;
      return false;
   }

   if(static_cast<MotorStatus>((uint8_t) _status) != MOTOR_STS_ENABLED)
   {
      LOG_WARN("Cannot set target position -> Drive is disabled!");
      return false;
   }

   if(_tgt_position.getDataUpdated())
      return true;
   if(value == (float) (_tgt_position))
      return true;

   _tgt_position = value;
   _tgt_position.setDataUpdated();

   return true;
}

const MotorStatus Motor::getOperationStatus(void)
{
   return static_cast<MotorStatus>((uint8_t) _status);
}

const MotorType Motor::getType(void)
{
   return static_cast<MotorType>((uint8_t) _type);
}

const float Motor::getCurrentAmp(void)
{
   return static_cast<float>(_current);
}

const float Motor::getPositionMM(void)
{
   return static_cast<float>(_position);
}

const float Motor::getRevolutions(void)
{
   return static_cast<float>(_revolutions);
}

const float Motor::getSpeedRPM(void)
{
   return static_cast<float>(_speed);
}

const float Motor::getMaxSpeedRPM(void) const
{
   return _speed_max_rpm;
}

/* !Public Class Functions -------------------------------------------------------*/

/* Private Class Functions -------------------------------------------------------*/

Motor::Motor(const unsigned int id, MotorShield& shield, const bool logging) :
    _id(id), _motor_shield(shield),
    _status(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_OPERATION_STS, true,
            uint8_t(0)),
    _tgt_pwm(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_TGT_PWM, true, float(0.0f)),
    _tgt_speed(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_TGT_SPD, true, float(0.0f)),
    _tgt_position(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_TGT_POS, true,
                  float(0.0f)),

    _type(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_TYPE, true, uint8_t(0)),
    _ctrl_mode(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_CTRL_MODE, true, uint8_t(0)),
    _pwm_max_value(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_PWM_MAX, true,
                   float(100.0f)),
    _gear_ratio(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_GEAR_RATIO, true, float(0)),
    _encoder_resolution(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_ENCODER_RESO, true,
                        uint16_t(0)),
    _conv_fac_adc_mm_per_tick(MSO_M_PARAM_BASE_IDX + (id * 1000u) +
                                  MO_CONV_FAC_ADC_MM_PER_TICK,
                              true, float(0)),
    _offs_adc_mm(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_OFFS_ADC_MM, true,
                 float(0)),
    _position_kp(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_POS_KP, true, float(0)),
    _position_ki(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_POS_KI, true, float(0)),
    _position_kd(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_POS_KD, true, float(0)),
    _speed_kp(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_SPD_KP, true, float(0)),
    _speed_ki(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_SPD_KI, true, float(0)),
    _speed_kd(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_SPD_KD, true, float(0)),
    _reset_revs(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_RESET_REVS, true, float(0)),
    _speed_max_rpm(std::numeric_limits<float>::max()),

    _current(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_CURRENT, false, float(0)),
    _position(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_POS, false, float(0)),
    _speed(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_SPD, false, float(0)),
    _revolutions(MSO_M_PARAM_BASE_IDX + (id * 1000u) + MO_REVS, false, float(0)),
    _logging(logging)
{
   if(_logging)
   {
      _log_module += "[" + std::to_string(_motor_shield._com_node_id) + ", " +
                     std::to_string(id) + "]:";
   }
}

const bool Motor::init(void)
{
   if(!_motor_shield.readConstObject(_type))
      return false;
   if(!_motor_shield.readConstObject(_ctrl_mode))
      return false;
   if(!_motor_shield.readConstObject(_pwm_max_value))
      return false;
   if(!_motor_shield.readConstObject(_gear_ratio))
      return false;
   if(!_motor_shield.readConstObject(_encoder_resolution))
      return false;
   if(!_motor_shield.readConstObject(_conv_fac_adc_mm_per_tick))
      return false;
   if(!_motor_shield.readConstObject(_offs_adc_mm))
      return false;
   if(!_motor_shield.readConstObject(_position_kp))
      return false;
   if(!_motor_shield.readConstObject(_position_ki))
      return false;
   if(!_motor_shield.readConstObject(_position_kd))
      return false;
   if(!_motor_shield.readConstObject(_speed_kp))
      return false;
   if(!_motor_shield.readConstObject(_speed_ki))
      return false;
   if(!_motor_shield.readConstObject(_speed_kd))
      return false;

   if(!_motor_shield.readConstObject(_status))
      return false;
   if(!_motor_shield.readConstObject(_revolutions))
      return false;

   _is_initialized = true;

   return true;
}

void Motor::release(void)
{
   if(!_is_initialized)
      return;

   _is_initialized = false;
}

void Motor::update(void)
{
   bool check_operation_status = false;

   // Read movement parameters
   switch(static_cast<MotorType>((uint8_t) _type))
   {
   case MOTOR_TYPE_LIFT: { _motor_shield.readConstObject(_position);
   }
   break;

   case MOTOR_TYPE_DRIVE:
   {
      _motor_shield.readConstObject(_speed);
      _motor_shield.readConstObject(_revolutions);
   }
   break;

   default:
   {
      // Unknown type
   }
   break;
   }

   // Check control mode -> update target values if neccessary
   switch(static_cast<MotorControlMode>((uint8_t)(_ctrl_mode)))
   {
   case MOTOR_CTRL_PWM:
   {
      if(_tgt_pwm.getDataUpdated())
      {
         LOG_INFO("Set PWM value: " << (float) (_tgt_pwm));
         if(!_motor_shield.writeDataObject(_tgt_pwm, "Target PWM"))
            check_operation_status = true;
      }
   }
   break;

   case MOTOR_CTRL_SPEED:
   {
      if(_tgt_speed.getDataUpdated())
      {
         if(!_motor_shield.writeDataObject(_tgt_speed, "Target Speed"))
            check_operation_status = true;
      }
   }
   break;

   case MOTOR_CTRL_POS:
   {
      if(_tgt_position.getDataUpdated())
      {
         std::cout << "Set target position: " << (float) (_tgt_position)
                   << std::endl;
         if(!_motor_shield.writeDataObject(_tgt_position, "Target Position"))
            check_operation_status = true;
      }
   }
   break;

   default:
   {
      // Do nothing
   }
   break;
   }

   if(check_operation_status)
   {

      _status_mutex.lock();
      _motor_shield.readConstObject(_status);
      _status_mutex.unlock();

      if(MOTOR_STS_DISABLED == static_cast<MotorStatus>((uint8_t) _status))
      {
         LOG_WARN("Motor is disabled -> enable to set target values!");
      }
   }
}

/* !Private Class Functions ------------------------------------------------------*/
