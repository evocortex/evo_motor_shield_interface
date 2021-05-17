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
#include <evo_mbed/com/ComDefs.h>

#include <iomanip>
#include <boost/crc.hpp>
/*--------------------------------------------------------------------------------*/

using namespace evo_mbed;

/* Makros ------------------------------------------------------------------------*/

#define CHECK_IS_INITIALIZED(x) \
            if(!_is_initialized) {LOG_ERROR("Class not initialized!"); return (x);}

/* Public Class Functions --------------------------------------------------------*/

MotorShield::MotorShield(const uint8_t node_id,
                         std::shared_ptr<ComServer> com_server,
                         const double update_rate_hz, const bool logging) :
    _com_server(com_server),
    _com_node_id(node_id), _update_rate_hz(update_rate_hz), _logging(true)
{
   if(_logging)
   {
      _log_module += "[" + std::to_string(node_id) + "]";
   }

   _run_update_thread  = false;
   _shield_synced      = false;
   _motor_shield_state = MOTOR_SHIELD_STS_ERR;
}

MotorShield::~MotorShield()
{
   release();
}

bool MotorShield::init()
{
   if(!checkInitConditions())
   {
      return false;
   }

   if(!checkDeviceType())
   {
      return false;
   }
   if(!checkFirmwareCompability())
   {
      return false;
   }
   if(!readAdditionalData())
   {
      return false;
   }
   if(!initMotors())
   {
      return false;
   }
   if(!runInitialSync())
   {
      return false;
   }

   _motor_shield_state = MOTOR_SHIELD_STS_OK;

   if(!createUpdateThread())
   {
      return false;
   }

   LOG_INFO(" Initialized Motor Shield: "
            << " FW-Ver: " << (float) (_do_fw_version)
            << " FW-Build: " << std::setprecision(8) << (float) (_do_fw_build_date)
            << " COM-Ver: " << std::setprecision(5) << (float) (_do_com_version));

   _timeout_error_cnt      = 0;
   _timeout_error_cnt_prev = 0;

   _is_initialized = true;

   return true;
}

void MotorShield::release()
{
   if(!_is_initialized)
   {
      return;
   }

   stopUpdateThread();

   for(auto& motor : _motor_list)
   {
      if(motor)
      {
         motor->release();
      }
   }

   _shield_synced   = false;
   _motor_shield_state = MOTOR_SHIELD_STS_ERR;
   _is_initialized  = false;
}

bool MotorShield::resyncShield()
{
   if(isShieldSynced())
   {
      return true;
   }

   if(!setComTimeout(_do_com_timeout))
   {
      return false;
   }

   for(auto& motor : _motor_list)
   {
      motor->_tgt_pwm = 0.0F;
      motor->_tgt_pwm.setDataUpdated();

      motor->_tgt_speed = 0.0F;
      motor->_tgt_speed.setDataUpdated();

      if(!motor->setType(static_cast<MotorType>((uint8_t) motor->_type)))
      {
         return false;
      }

      if(!motor->setPWMLimit(motor->_pwm_max_value))
      {
         return false;
      }

      if(!motor->setControlMode(
             static_cast<MotorControlMode>((uint8_t) motor->_ctrl_mode)))
      {
         return false;
      }

      if(!motor->setGearRatio(motor->_gear_ratio))
      {
         return false;
      }

      if(!motor->setEncoderResolution(motor->_encoder_resolution))
      {
         return false;
      }

      if(!motor->setConvFacAdcMMPerTick(motor->_conv_fac_adc_mm_per_tick))
      {
         return false;
      }

      if(!motor->setOffsAdcMM(motor->_offs_adc_mm))
      {
         return false;
      }

      if(!motor->setPositionKp(motor->_position_kp))
      {
         return false;
      }

      if(!motor->setPositionKi(motor->_position_ki))
      {
         return false;
      }

      if(!motor->setPositionKd(motor->_position_kd))
      {
         return false;
      }

      if(!motor->setSpeedKp(motor->_speed_kp))
      {
         return false;
      }

      if(!motor->setSpeedKi(motor->_speed_ki))
      {
         return false;
      }

      if(!motor->setSpeedKd(motor->_speed_kd))
      {
         return false;
      }

      if(MOTOR_TYPE_DRIVE == motor->getType())
      {
         motor->_reset_revs =  static_cast<float>(motor->_revolutions);

         if(!writeDataObject(motor->_reset_revs, "Reset Revolutions Resync!"))
         {
            return false;
         }
      }
   }

   return true;
}

bool MotorShield::setComTimeout(const uint32_t com_timeout_ms)
{
   CHECK_IS_INITIALIZED(false);

   std::lock_guard<std::mutex> guard(_config_mutex);

   _do_com_timeout = com_timeout_ms;

   return writeDataObject(_do_com_timeout, "ComTimeout");
}

std::shared_ptr<Motor> MotorShield::getMotor(const unsigned int id)
{
   CHECK_IS_INITIALIZED(nullptr);

   std::shared_ptr<Motor> motor = nullptr;

   if(0 == id)
   {
      motor = _motor_list[0];
   }
   else if(1 == id)
   {
      motor = _motor_list[1];
   }
   else
   {
      LOG_ERROR("Requested motor id is invalid [0;1]");
   }
   
   return motor;
}

unsigned int MotorShield::getComID() const
{
   return _com_node_id;
}

MotorShieldState MotorShield::getState() const
{
   return _motor_shield_state;
}

bool MotorShield::isShieldSynced() const
{
   return _shield_synced;
}

bool MotorShield::isInitialized() const
{
   return _is_initialized;
}

/* !Public Class Functions -------------------------------------------------------*/

/* Private Class Functions -------------------------------------------------------*/

bool MotorShield::checkInitConditions()
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

   if(_com_node_id < COM_NODE_ID_MIN && _com_node_id > COM_NODE_ID_MAX)
   {
      LOG_ERROR("Node ID is not valid! [1;127]");
      return false;
   }

   if(_update_rate_hz <= MOTOR_SHIELD_MIN_UPDATE_RATE_HZ)
   {
      LOG_ERROR("Update rate has to be >= " << MOTOR_SHIELD_MIN_UPDATE_RATE_HZ 
         << " (" << _update_rate_hz << ")");
      return false;
   }

   if(RES_OK != _com_server->registerNode(_com_node_id))
   {
      LOG_ERROR("Failed to register node to communciation server!");
      return false;
   }

   return true;
}

bool MotorShield::checkDeviceType()
{
   if(!readConstObject(_device_type))
   {
      return false;
   }

   if(1 != (uint8_t) _device_type)
   {
      LOG_ERROR("Motorshield error: Type of Node is '" << +(uint8_t) _device_type
                                                       << "' which is not a"
                                                       << " Motorshield (=1)!");
      return false;
   }

   return true;
}

bool MotorShield::checkFirmwareCompability()
{
   if(!readConstObject(_do_fw_version))
   {
      return false;
   }
   if(!readConstObject(_do_com_version))
   {
      return false;
   }

   if(!isCOMVersionCompatible())
   {
      LOG_ERROR("Motorshield firmware com version is not compatible with"
                << " node version! (Motorshield: " << std::setprecision(2)
                << (float(_do_com_version)) << " Node: " << MOTOR_SHIELD_COM_VER);

      return false;
   }

   return true;
}

bool MotorShield::readAdditionalData()
{
   if(!readConstObject(_do_fw_build_date))
   {
      return false;
   }
   if(!readConstObject(_do_status))
   {
      return false;
   }
   if(!readConstObject(_do_cfg_crc))
   {
      return false;
   }
   if(!readConstObject(_do_reset_request))
   {
      return false;
   }
   if(!readConstObject(_do_com_timeout))
   {
      return false;
   }
   if(!readConstObject(_do_pwm_frequency))
   {
      return false;
   }

   return true;
}

bool MotorShield::initMotors()
{
   unsigned int id = 0;
   for(auto& motor : _motor_list)
   {
      motor = std::shared_ptr<Motor>(new Motor(id++, *this, _logging));

      if(!motor->init())
      {
         LOG_ERROR("Failed to initialized motor: " << +id << "!");
         return false;
      }
   }

   return true;
}

bool MotorShield::runInitialSync()
{
   checkSyncStatus();

   if(!isShieldSynced())
   {
      _is_initialized   = true;
      const bool result = resyncShield();
      _is_initialized   = false;

      if(!result)
      {
         LOG_ERROR("Failed to synchronise settings with motor shield!");
         return false;
      }
   }

   return true;
}

bool MotorShield::createUpdateThread()
{
   constexpr unsigned int maximum_wait_time = 10;

   _update_thread = std::make_unique<std::thread>(&MotorShield::updateHandler, this);

   std::this_thread::sleep_for(std::chrono::milliseconds(maximum_wait_time));

   if(!_run_update_thread)
   {
      LOG_ERROR("Failed to start update thread!");
      return false;
   }

   return true;
}

void MotorShield::stopUpdateThread()
{
   if(_run_update_thread)
   {
      _run_update_thread = false;
      _update_thread->join();
   }
}

bool MotorShield::isCOMVersionCompatible()
{
   return std::floor(static_cast<float>(_do_com_version)) ==
          std::floor(MOTOR_SHIELD_COM_VER);
}

void MotorShield::printComError(const ComDataObject& object, const std::string& name, 
                                const ComMsgErrorCodes& error_code)
{
   std::string error_str;

   switch(error_code)
   {
   case COM_MSG_ERR_NONE: {
      return;
   }
   break;
   case COM_MSG_ERR_INVLD_CMD: {
      error_str = "Invalid command";
   }
   break;
   case COM_MSG_ERR_READ_ONLY: {
      error_str = "Read-Only";
   }
   break;
   case COM_MSG_ERR_OBJCT_INVLD: {
      error_str = "Object unknown";
   }
   break;
   case COM_MSG_ERR_INVLD_DATA_TYPE: {
      error_str = "Invalid data type";
   }
   break;
   case COM_MSG_ERR_VALUE_RANGE_EXCD: {
      error_str = "Value out of range";
   }
   break;
   case COM_MSG_ERR_COND_NOT_MET: {
      error_str = "Conditions not met to write";
   }
   break;
   }

   const std::string log_msg = "Failed to write object: ";

   const std::string object_info =
      " (Object-ID: " + std::to_string(object.getID()) +
      ", Raw-Value: " + std::to_string(object.getRawValue()) + ", Desc: " + name +
      ")";

   LOG_ERROR(log_msg << error_str << " " << object_info);
}

void MotorShield::updateHandler()
{
   _run_update_thread = true;

   const std::chrono::duration<double, std::micro> loop_time_usec(1e6 /
                                                                  _update_rate_hz);

   while(_run_update_thread)
   {
      const auto timestamp_start = std::chrono::high_resolution_clock::now();

      checkSyncStatus();

      checkShieldStatus();

      if(true == isShieldSynced())
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

   ComMsgErrorCodes error_code = {COM_MSG_ERR_NONE};

   const Result result =
       _com_server->readDataObject(_com_node_id, object, error_code, 
                                    0U, MOTOR_SHIELD_COM_RETRY_LIMIT);

   switch(result)
   {
   case RES_OK: 
   {
      return (COM_MSG_ERR_NONE == error_code);
   }

   case RES_TIMEOUT: 
   {
      _timeout_error_cnt++;
      return false;
   }

   default: 
   {
      _motor_shield_state = MOTOR_SHIELD_STS_ERR;
      return false;
   }
   }

   return false;
}

bool MotorShield::writeDataObject(ComDataObject& object, const std::string name)
{
   std::lock_guard<std::mutex> guard(_com_mutex);

   ComMsgErrorCodes error_code = {COM_MSG_ERR_NONE};
   const Result result =
       _com_server->writeDataObject(_com_node_id, object, error_code, 0, MOTOR_SHIELD_COM_RETRY_LIMIT);

   switch(result)
   {
   case RES_OK: {
      bool success = (COM_MSG_ERR_NONE == error_code);

      if(success)
      {
         object.setDataReaded();
      }
      else
      {
         if(_logging) 
         {
            printComError(object, name, error_code);
         }

         if(RES_OK != _com_server->readDataObject(_com_node_id, object, error_code, 0U, MOTOR_SHIELD_COM_RETRY_LIMIT))
         {
            LOG_ERROR("Failed to read data from device!");
         }
      }

      return success;
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
      _motor_shield_state = MOTOR_SHIELD_STS_ERR;
      return false;
   }
   break;
   }

   return false;
}

void MotorShield::checkSyncStatus()
{
   std::lock_guard<std::mutex> guard(_config_mutex);

   if(!readConstObject(_do_cfg_crc))
   {
      LOG_WARN("Failed to read CRC from drive!");
      _shield_synced = false;
      return;
   }

   const uint32_t shield_crc = _do_cfg_crc;
   const uint32_t host_crc   = calcConfigCRC();

   if(shield_crc != host_crc)
   {
      LOG_WARN("Configuration between host and motorshield out of sync!");
      _shield_synced = false;
   }
   else
   {
      _shield_synced = true;
   }
}

void MotorShield::checkShieldStatus()
{
   if(MOTOR_SHIELD_STS_ERR == _motor_shield_state)
   {
      return;
   }

   bool new_timeout_error_detected = (_timeout_error_cnt > _timeout_error_cnt_prev);

   if(new_timeout_error_detected)
   {
      _motor_shield_state = MOTOR_SHIELD_TIMEOUT;
      LOG_INFO("Timeout-Error! Overall count: " << + _timeout_error_cnt);
   }
   else
   {
      if(!isShieldSynced())
      {
         _motor_shield_state = MOTOR_SHIELD_SYNC_ERR;
      }
      else
      {
         _motor_shield_state = MOTOR_SHIELD_STS_OK;
      }
   }

   if(_timeout_error_cnt >= std::numeric_limits<unsigned int>::max())
   {
      _timeout_error_cnt = 0;
      _timeout_error_cnt_prev = 0;
   }
   else
   {
      _timeout_error_cnt_prev = _timeout_error_cnt;
   }
}

uint32_t MotorShield::calcConfigCRC() const
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
