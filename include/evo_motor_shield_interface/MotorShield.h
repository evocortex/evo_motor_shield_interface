//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file MotorShield.h
 * @author MBA (info@evocortex.com)
 *
 * @brief Motorshield Representation
 *
 * @version 1.0
 * @date 2019-04-17
 *
 * @copyright Copyright (c) 2019 Evocortex GmbH
 *
 */

#ifndef EVO_MOTOR_SHIELD_H_
#define EVO_MOTOR_SHIELD_H_

/* Includes ----------------------------------------------------------------------*/
#include <evo_mbed/Utils.h>
#include <evo_mbed/tools/com/ComServer.h>

#include <evo_motor_shield_interface/Motor.h>
/*--------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------*/
/** @addtogroup evocortex
 * @{
 */

namespace evo_mbed {

/*--------------------------------------------------------------------------------*/
/** @addtogroup evocortex_MotorShield
 * @{
 */

// Predefine
class Motor;

/** \brief Supported communication version */
constexpr float MOTOR_SHIELD_COM_VER = 1.0f;

/** \brief Number of drives on one shield */
constexpr unsigned int MOTOR_SHIELD_DRIVES = 2u;

/**
 * @brief States of the motor shield
 */
enum MotorShieldState : uint8_t
{
   MOTOR_SHIELD_STS_ERR  = 0u, //!< General error
   MOTOR_SHIELD_STS_OK   = 1u, //!< Ok
   MOTOR_SHIELD_SYNC_ERR = 2u, //!< Config on host and shield out of sync
   MOTOR_SHIELD_TIMEOUT  = 3u //!< Timeout (e.g. Shield is off due to emergency stop)
};

/**
 * @brief Communication Object IDs of motor shield
 */
enum MotorShieldObjects : uint16_t
{
   /* General data */
   MSO_DEV_TYPE           = 10001u, //!< Device Type ID
   MSO_FW_VER             = 10002u, //!< Firmware Version of motor shield
   MSO_FW_COM_VER         = 10003u, //!< Communication stack version
   MSO_FW_BUILD_DATE      = 10004u, //!< Build date of the firmware
   MSO_STS                = 10011u, //!< Motorshield status
   MSO_CFG_CRC            = 10012u, //!< Checksum of all configuration parameters
   MSO_STAT_LOOP_TIME_MAX = 10013u, //!< Maximum loop time in usec

   /* General settings */
   MSO_RESET         = 10101u, //!< Reset device if set to true
   MSO_COM_TIMEOUT   = 10102u, //!< Communication timeout in ms
   MSO_PWM_FREQUENCY = 10103u, //!< PWM Frequency of powerstage in kHz

   /* Start of M1 objects */
   MSO_M_PARAM_BASE_IDX = 11000u, //!< Base index of drive parameters

   MSO_OBJ_SIZE = 54u, //!< Num of communication objects
};

/**
 * @brief Communication Object IDs of motor
 *
 */
enum MotorObjects : uint16_t
{
   MO_TYPE = 1u,    //!< Motor type
   MO_CTRL_MODE,    //!< Motor control mode
   MO_PWM_MAX,      //!< Absolute maximum allowed pwm value in percent
   MO_GEAR_RATIO,   //!< Gear ratio motor revolutions per shaft revolution
   MO_ENCODER_RESO, //!< Encoder resolution ticks per motor revolution
   MO_CONV_FAC_ADC_MM_PER_TICK, //!< Conversion factor from ADC to mm (mm/tick)
   MO_OFFS_ADC_MM,              //!< Offset value in mm of lift drive
   MO_POS_KP,                   //!< Position controller Kp value
   MO_POS_KI,                   //!< Position controller Ki value
   MO_POS_KD,                   //!< Position controller Kd value
   MO_SPD_KP,                   //!< Speed controller Kp value
   MO_SPD_KI,                   //!< Speed controller Ki value
   MO_SPD_KD,                   //!< Speed controller Kd value
   MO_RESET_REVS, //!< Resets the wheel revolutions to the specified value

   MO_CURRENT = 101u, //!< Actual current in Ampere
   MO_POS,            //!< Actual position in mm
   MO_SPD,            //!< Actual speed in rpm
   MO_REVS,           //!< Actual absolute revolutions of the wheel

   MO_OPERATION_STS = 201u, //!< Operation status of drive
   MO_TGT_PWM,              //!< Target PWM frequency in percent [-100.0;100.0]
   MO_TGT_SPD,              //!< Target speed in rpm
   MO_TGT_POS               //!< Target position in mm
};

/**
 * @brief Motor shield representation
 *
 */
class MotorShield
{
 public:
   /**
    * @brief Default constructor of a motorshield object
    *
    * @param node_id Communication ID of the motor shield [1;127]
    * @param com_server Pointer to communication server
    * @param update_rate Update rate of the async thread in hz
    * @param logging true Enable logging output (default=false)
    */
   MotorShield(const uint8_t node_id, std::shared_ptr<ComServer> com_server,
               const double update_rate_hz = 30.0, const bool logging = false);

   /**
    * @brief Destructor of motorshield object
    */
   ~MotorShield(void);

   /**
    * @brief Initializes the motor shield
    *        Checks if motor shield is reachable, if communication
    *        version is supported and starts async update thread.
    *
    * @return true Success
    * @return false Error
    */
   bool init(void);

   /**
    * @brief Releases the object stops threads and releases
    *        memory
    */
   void release(void);

   /**
    * @brief Synchronises the shield again with the host pc, by
    *        writing configuration from host to shield
    *
    * TODO: Mit marco reden ob das automatisch erfolgen soll
    *
    * @return true
    * @return false
    */
   bool resyncShield(void);

   /**
    * @brief Sends request to perform a complete reset of the motorcontroller
    *
    * @return true
    * @return false
    */
   bool resetShield(void);

   /**
    * @brief Set maximum communication timeout
    *
    * @param com_timeout Communication timeout limit in milliseconds
    *
    * @return true
    * @return false
    */
   bool setComTimeout(const uint32_t com_timeout_ms);

   /**
    * @brief Sets the PWM frequency of the powerstage (NOT SUPPORTED YET)
    *
    * @param frequency Frequency of powerstage in kHz
    *
    * @return true
    * @return false
    */
   // const bool setPWMFrequency(const uint8_t frequency_khz);

   /**
    * @brief Get the motor
    *
    * @param id ID of the motor [0;1]
    *
    * @return std::shared_ptr<Motor> Requested object
    */
   std::shared_ptr<Motor> getMotor(const unsigned int id);

   /* Getters */
   float getComID(void) const;
   MotorShieldState getState(void) const;

   /** \brief Check if class is initialized */
   bool isInitialized(void) const;

 private:
   /**
    * @brief Updates the motor shield
    */
   void updateHandler(void);

   /**
    * @brief Reads a constant data object
    *
    * @param object Object to read
    *
    * @return true Reading data was successful
    * @return false Failed reading data
    */
   bool readConstObject(ComDataObject& object);

   /**
    * @brief Writes a data object via can
    *
    * @param object Object to write
    * @param name Name of the object for logging
    *
    * @return true Successfully written value
    * @return false Error during writting
    */
   bool writeDataObject(ComDataObject& object, const std::string name);

   /**
    * @brief Checks if shield and host is synchronized
    */
   void checkSyncStatus(void);

   /**
    * @brief Checks the current shield status
    */
   void checkShieldStatus(void);

   /**
    * @brief Calculates the CRC32 value of the
    *        configuration parameters
    *
    * @return const uint32_t CRC32 value of config
    */
   const uint32_t calcConfigCRC(void);

   /** \brief Used communication server */
   std::shared_ptr<ComServer> _com_server;

   /** \brief Node ID of the client */
   const unsigned int _com_node_id = 0u;

   /** \brief Update rate of the async data in hz */
   const double _update_rate_hz = 30.0f;

   /** \brief Update thread for asnyc tx/rx */
   std::unique_ptr<std::thread> _update_thread;

   /** \brief Set to false to stop update thread */
   std::atomic<bool> _run_update;

   /** \brief List containing motor objects */
   std::array<std::shared_ptr<Motor>, MOTOR_SHIELD_DRIVES> _motor_list;

   /** \brief True if config on host and shield is synchronized */
   std::atomic<bool> _is_shield_synced;

   /** \brief Mutex for synchronization of communication access */
   std::mutex _com_mutex;

   /** \brief Locks checksum calculation during config update */
   std::mutex _config_mutex;

   /** \brief State of the motorshield */
   std::atomic<MotorShieldState> _motor_shield_state;

   /** \brief Count of timeout errors occured */
   std::atomic<unsigned int> _timeout_error_cnt;

   /** \brief Old timeout error count */
   unsigned int _timeout_error_cnt_prev;

   /* Motor shield read only objects */
   ComDataObject _device_type      = ComDataObject(MSO_DEV_TYPE, false, uint8_t(0));
   ComDataObject _do_fw_version    = ComDataObject(MSO_FW_VER, false, 0.0f);
   ComDataObject _do_com_version   = ComDataObject(MSO_FW_COM_VER, false, 0.0f);
   ComDataObject _do_fw_build_date = ComDataObject(MSO_FW_BUILD_DATE, false, 0.0f);
   ComDataObject _do_status        = ComDataObject(MSO_STS, false, uint8_t(0));
   ComDataObject _do_cfg_crc       = ComDataObject(MSO_CFG_CRC, false, uint32_t(0));

   /* Motor shield settings */
   ComDataObject _do_reset_request = ComDataObject(MSO_RESET, true, bool(false));
   ComDataObject _do_com_timeout = ComDataObject(MSO_COM_TIMEOUT, true, uint32_t(0));
   ComDataObject _do_pwm_frequency =
       ComDataObject(MSO_PWM_FREQUENCY, true, uint8_t(0));

   /** \brief Logging option: set to true to enable logging */
   const bool _logging = false;

   /** \brief Logging module name */
   std::string _log_module = "MotorShield";

   /** \brief True class is initialized */
   bool _is_initialized = false;

   friend Motor;
};

/**
 * @}
 */ // evocortex_MotorShield
/*--------------------------------------------------------------------------------*/

}; // namespace evo_mbed

/**
 * @}
 */ // evocortex
/*--------------------------------------------------------------------------------*/

#endif /* EVO_MOTOR_SHIELD_H_ */