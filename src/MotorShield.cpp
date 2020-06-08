//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file MotorShield.cpp
 * @author MBA (info@evocortex.com)
 *
 * @brief Source of Motor Shield
 *
 * @version 1.0
 * @date 2019-04-17
 *
 * @copyright Copyright (c) 2019
 *
 */

/* Includes ----------------------------------------------------------------------*/
#include <evo_motor_shield_interface/MotorShield.h>
#include <evo_motor_shield_interface/Motor.h>
#include <evo_mbed/tools/Logging.h>

#include <iomanip>
#include <boost/crc.hpp>
/*--------------------------------------------------------------------------------*/

using namespace evo_mbed;

/* Public Class Functions --------------------------------------------------------*/

MotorShield::MotorShield(const uint8_t node_id,
                         std::shared_ptr<ComServer> com_server,
                         const double update_rate_hz, const bool logging) :
    _com_server(com_server),
    _com_node_id(node_id), _update_rate_hz(update_rate_hz), _logging(logging)
{
   if(_logging)
   {
      _log_module += "[" + std::to_string(node_id) + "]";
   }

   _run_update         = false;
   _is_shield_synced   = false;
   _motor_shield_state = MOTOR_SHIELD_STS_OK;
}

MotorShield::~MotorShield(void)
{
   release();
}

bool MotorShield::init(void)
{
   if(_is_initialized)
   {
      LOG_ERROR("Class is already initialzed");
      return false;
   }

   if(!_com_server)
   {
      LOG_ERROR("Pointer to _com_server is null");
      return false;
   }

   if(_com_node_id < 1 && _com_node_id > 127)
   {
      LOG_ERROR("Node ID is not valid! [1;127]");
      return false;
   }

   if(_update_rate_hz <= 0.1)
   {
      LOG_ERROR("Update rate has to be >= 0.1 (" << _update_rate_hz << ")");
      return false;
   }

   if(RES_OK != _com_server->registerNode(_com_node_id))
   {
      LOG_ERROR("Failed to register node to communciation server!");
      return false;
   }

   // Reset variables
   _is_shield_synced   = false;
   _motor_shield_state = MOTOR_SHIELD_STS_ERR;

   if(!readConstObject(_device_type))
      return false;

   // Check type
   if(1u != (uint8_t) _device_type)
   {
      LOG_ERROR("Motorshield error: Type of Node is '" << +(uint8_t) _device_type
                                                       << "' which is not a"
                                                       << " Motorshield (=1)!");
      return false;
   }

   if(!readConstObject(_do_fw_version))
      return false;
   if(!readConstObject(_do_com_version))
      return false;

   // Check communication version -> Check if com version fits
   // the supported stack
   if(MOTOR_SHIELD_COM_VER != (float) (_do_com_version))
   {
      LOG_ERROR("Motorshield reports communication version '"
                << (float) (_do_com_version) << "' but only version '"
                << MOTOR_SHIELD_COM_VER << "' is supported!");
      return false;
   }

   if(!readConstObject(_do_fw_build_date))
      return false;
   if(!readConstObject(_do_status))
      return false;
   if(!readConstObject(_do_cfg_crc))
      return false;

   if(!readConstObject(_do_reset_request))
      return false;
   if(!readConstObject(_do_com_timeout))
      return false;
   if(!readConstObject(_do_pwm_frequency))
      return false;

   // Create and intialize sensors
   unsigned int id = 0u;
   for(auto& motor : _motor_list)
   {
      motor = std::shared_ptr<Motor>(new Motor(id++, *this, _logging));

      if(!motor->init())
      {
         LOG_ERROR("Failed to initialized motor: " << +id << "!");
         return false;
      }
   }

   // Check sync state
   checkSyncStatus();

   if(!_is_shield_synced)
   {
      LOG_ERROR("Shield is not synchronized with settings in initialization phase!");
      return false;
   }

   _motor_shield_state = MOTOR_SHIELD_STS_OK;

   // Create update thread
   _update_thread = std::make_unique<std::thread>(&MotorShield::updateHandler, this);
   auto timer_ms  = 0u;
   while(timer_ms < 10u && !_run_update)
   {
      timer_ms++;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
   }

   if(!_run_update)
   {
      LOG_ERROR("Failed to start update thread!");
      return false;
   }

   LOG_INFO(" Initialized Motor Shield: "
            << " FW-Ver: " << (float) (_do_fw_version)
            << " FW-Build: " << std::setprecision(8) << (float) (_do_fw_build_date)
            << " COM-Ver: " << std::setprecision(5) << (float) (_do_com_version));

   _timeout_error_cnt      = 0u;
   _timeout_error_cnt_prev = 0u;

   _is_initialized = true;

   return true;
}

void MotorShield::release(void)
{
   if(!_is_initialized)
      return;

   if(_run_update)
   {
      _run_update = false;
      _update_thread->join();
   }

   for(auto motor : _motor_list)
   {
      if(motor)
      {
         motor->release();
      }
   }

   _is_initialized = false;
}

bool MotorShield::resyncShield(void)
{
   if(_is_shield_synced)
      return true;

   if(!setComTimeout(_do_com_timeout))
      return false;

   for(auto motor : _motor_list)
   {
      if(!motor->setType(static_cast<MotorType>((uint8_t) motor->_type)))
         return false;

      if(!motor->setPWMLimit(motor->_pwm_max_value))
         return false;

      if(!motor->setControlMode(
             static_cast<MotorControlMode>((uint8_t) motor->_ctrl_mode)))
         return false;

      if(!motor->setGearRatio(motor->_gear_ratio))
         return false;

      if(!motor->setEncoderResolution(motor->_encoder_resolution))
         return false;

      if(!motor->setConvFacAdcMMPerTick(motor->_conv_fac_adc_mm_per_tick))
         return false;

      if(!motor->setOffsAdcMM(motor->_offs_adc_mm))
         return false;

      if(!motor->setPositionKp(motor->_position_kp))
         return false;

      if(!motor->setPositionKi(motor->_position_ki))
         return false;

      if(!motor->setPositionKd(motor->_position_kd))
         return false;

      if(!motor->setSpeedKp(motor->_speed_kp))
         return false;

      if(!motor->setSpeedKi(motor->_speed_ki))
         return false;

      if(!motor->setSpeedKd(motor->_speed_kd))
         return false;

      if(MOTOR_TYPE_DRIVE == motor->getType())
      {
         motor->_reset_revs = (float) motor->_revolutions;

         // Reset revolutions
         if(!writeDataObject(motor->_reset_revs, "Reset Revolutions Resync!"))
            return false;
      }
   }

   return true;
}

bool MotorShield::setComTimeout(const uint32_t com_timeout_ms)
{
   // Lock CRC calculation
   std::lock_guard<std::mutex> guard(_config_mutex);

   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   _do_com_timeout = com_timeout_ms;

   if(!writeDataObject(_do_com_timeout, "ComTimeout"))
      return false;

   return true;
}

std::shared_ptr<Motor> MotorShield::getMotor(const unsigned int id)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return std::shared_ptr<Motor>();
   }

   if(id >= MOTOR_SHIELD_DRIVES)
   {
      LOG_ERROR("Requested motor id is invalid [0;1]");
      return std::shared_ptr<Motor>();
   }

   return _motor_list[id];
}

float MotorShield::getComID(void) const
{
   return _com_node_id;
}

MotorShieldState MotorShield::getState(void) const
{
   return _motor_shield_state;
}

bool MotorShield::isInitialized(void) const
{
   return _is_initialized;
}

/* !Public Class Functions -------------------------------------------------------*/

/* Private Class Functions -------------------------------------------------------*/

void MotorShield::updateHandler(void)
{
   _run_update = true;

   const std::chrono::duration<double, std::micro> loop_time_usec(1e6 /
                                                                  _update_rate_hz);

   while(_run_update)
   {
      const auto timestamp_start = std::chrono::high_resolution_clock::now();

      checkSyncStatus();

      checkShieldStatus();

      if(true == _is_shield_synced)
      {
         for(auto& motor : _motor_list)
         {
            motor->update();
         }
      }

      const auto timestamp_stop = std::chrono::high_resolution_clock::now();
      const std::chrono::duration<double, std::micro> exec_time_usec =
          timestamp_stop - timestamp_start;
      const auto sleep_time_usec = loop_time_usec - exec_time_usec;
      std::this_thread::sleep_for(sleep_time_usec);
   }
}

bool MotorShield::readConstObject(ComDataObject& object)
{
   std::lock_guard<std::mutex> guard(_com_mutex);

   ComMsgErrorCodes error_code;

   const Result com_result =
       _com_server->readDataObject(_com_node_id, object, error_code, 0u, 4u);

   switch(com_result)
   {
   case RES_OK:
   {
      if(COM_MSG_ERR_NONE != error_code)
         return false;

      return true;
   }
   break;

   case RES_TIMEOUT:
   {
      _timeout_error_cnt++;
      return false;
   }
   break;

   default:
   {
      // General error
      _motor_shield_state = MOTOR_SHIELD_STS_ERR;
      return false;
   }
   break;
   }

   return false;
}

bool MotorShield::writeDataObject(ComDataObject& object,
                                        const std::string name)
{
   std::lock_guard<std::mutex> guard(_com_mutex);

   ComMsgErrorCodes error_code = COM_MSG_ERR_NONE;

   const std::string log_info =
       " (Object-ID: " + std::to_string(object.getID()) +
       ", Raw-Value: " + std::to_string(object.getRawValue()) + ", Desc: " + name +
       ")";

   // Write data with timeout threshold = default and 2 retries
   const Result com_result =
       _com_server->writeDataObject(_com_node_id, object, error_code, 0, 2u);

   switch(com_result)
   {
   case RES_OK:
   {

      if(COM_MSG_ERR_NONE == error_code)
      {
         object.setDataReaded();
         return true;
      }

      if(_logging)
      {
         switch(error_code)
         {
         case COM_MSG_ERR_INVLD_CMD:
         {
            LOG_ERROR("Failed to write object: Invalid command" << log_info);
         }
         break;
         case COM_MSG_ERR_READ_ONLY:
         {
            LOG_ERROR("Failed to write object: Read-Only" << log_info);
         }
         break;
         case COM_MSG_ERR_OBJCT_INVLD:
         {
            LOG_ERROR("Failed to write object: Object unknown" << log_info);
         }
         break;
         case COM_MSG_ERR_INVLD_DATA_TYPE:
         {
            LOG_ERROR("Failed to write object: Invalid data type" << log_info);
         }
         break;
         case COM_MSG_ERR_VALUE_RANGE_EXCD:
         {
            LOG_ERROR("Failed to write object: Value out of range" << log_info);
         }
         break;
         case COM_MSG_ERR_COND_NOT_MET:
         {
            LOG_ERROR("Failed to write object: Conditions not met to write"
                      << log_info);
         }
         break;
         }
      }

      if(RES_OK !=
         _com_server->readDataObject(_com_node_id, object, error_code, 0u, 2u))
         LOG_ERROR("Failed to read data from device" << log_info);

      return false;
   }
   break;

   case RES_TIMEOUT:
   {
      _timeout_error_cnt++;

      return false;
   }
   break;

   default:
   {
      // General error
      _motor_shield_state = MOTOR_SHIELD_STS_ERR;
      return false;
   }
   break;
   }

   return false;
}

void MotorShield::checkSyncStatus(void)
{
   std::lock_guard<std::mutex> guard(_config_mutex);

   if(!readConstObject(_do_cfg_crc))
   {
      _is_shield_synced = false;
      return;
   }

   const uint32_t shield_crc = _do_cfg_crc;
   const uint32_t host_crc   = calcConfigCRC();

   if(shield_crc != host_crc)
   {
      LOG_WARN("Configuration between host and controll out of sync!");
      _is_shield_synced = false;
   }
   else
   {
      _is_shield_synced = true;
   }
}

void MotorShield::checkShieldStatus(void)
{
   // Check if error is present -> only healable by reset
   if(MOTOR_SHIELD_STS_ERR == _motor_shield_state)
   {
      return;
   }

   // Store timeout errors
   const unsigned int timeout_error_cnt = _timeout_error_cnt;
   const int timeout_errors = timeout_error_cnt - _timeout_error_cnt_prev;

   // If more than 1 timeout error occured in the last cycle
   // switch to timeout error
   if(timeout_errors > 0)
   {
      _motor_shield_state = MOTOR_SHIELD_TIMEOUT;
      LOG_INFO("Errors: " << +timeout_errors);
   }
   else
   {
      if(!_is_shield_synced)
      {
         _motor_shield_state = MOTOR_SHIELD_SYNC_ERR;
      }
      else
      {
         _motor_shield_state = MOTOR_SHIELD_STS_OK;
      }
   }

   _timeout_error_cnt_prev = timeout_error_cnt;
}

const uint32_t MotorShield::calcConfigCRC(void)
{
   uint32_t data[MSO_OBJ_SIZE] = {0};
   uint16_t idx                = 0;

   /* General config */
   data[idx++] = _do_com_timeout.getRawValue();
   data[idx++] = _do_pwm_frequency.getRawValue();

   /* Motor config */
   for(auto& motor : _motor_list)
   {
      data[idx++] = motor->_type.getRawValue();
      data[idx++] = motor->_ctrl_mode.getRawValue();
      data[idx++] = motor->_pwm_max_value.getRawValue();
      data[idx++] = motor->_gear_ratio.getRawValue();
      data[idx++] = motor->_encoder_resolution.getRawValue();
      data[idx++] = motor->_conv_fac_adc_mm_per_tick.getRawValue();
      data[idx++] = motor->_offs_adc_mm.getRawValue();
      data[idx++] = motor->_position_kp.getRawValue();
      data[idx++] = motor->_position_ki.getRawValue();
      data[idx++] = motor->_position_kd.getRawValue();
      data[idx++] = motor->_speed_kp.getRawValue();
      data[idx++] = motor->_speed_ki.getRawValue();
      data[idx++] = motor->_speed_kd.getRawValue();
   }
   const uint16_t u32_size = idx;

   /* Calculate CRC32 */
   boost::crc_32_type crc32;

   for(idx = 0u; idx < u32_size; idx++)
   {
      uint8_t* byte_ptr = (uint8_t*) (&data[idx]);
      crc32.process_byte(byte_ptr[3]);
      crc32.process_byte(byte_ptr[2]);
      crc32.process_byte(byte_ptr[1]);
      crc32.process_byte(byte_ptr[0]);
   }

   return crc32.checksum();
}

/* !Private Class Functions ------------------------------------------------------*/
