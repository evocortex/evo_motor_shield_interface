//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file Motor.h
 * @author MBA (info@evocortex.com)
 *
 * @brief Motor Representation
 *
 * @version 1.0
 * @date 2019-04-17
 *
 * @copyright Copyright (c) 2019 Evocortex GmbH
 *
 */

#ifndef EVO_MOTOR_H_
#define EVO_MOTOR_H_

/* Includes ----------------------------------------------------------------------*/
#include <atomic>

#include <evo_mbed/Utils.h>
#include <evo_mbed/tools/com/ComServer.h>
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
class MotorShield;

/**
 * @brief Enumeration of motor types
 */
enum MotorType : uint8_t
{
   MOTOR_TYPE_NONE  = 0U, //!< No type specified
   MOTOR_TYPE_LIFT  = 1U, //!< Lift motor
   MOTOR_TYPE_DRIVE = 2U  //!< Drive motor
};

/**
 * @brief Enumeration of motor control modes
 */
enum MotorControlMode : uint8_t
{
   MOTOR_CTRL_NONE  = 0u, //!< Noe control mode selected
   MOTOR_CTRL_PWM   = 1U, //!< Directly control pwm value
   MOTOR_CTRL_SPEED = 2U, //!< Control speed
   MOTOR_CTRL_POS   = 4U  //!< Control position
};

/**
 * @brief Status of the motor
 */
enum MotorStatus : uint8_t
{
   MOTOR_STS_DISABLED = 0U, //!< Drive disabled
   MOTOR_STS_ENABLED  = 1U  //!< Drive enabled but stopped
};

/**
 * @brief Motor representation
 *
 */
class Motor
{
 public:
   /** \brief Default destructor */
   ~Motor(void);

   /**
    * @brief Set the drive type
    *
    * @param type Motortype @ref MotorType
    *
    * @return true Success
    * @return false Error
    */
   const bool setType(const MotorType type);

   /**
    * @brief Sets the maximum allowed PWM value
    *
    * @param pwm_max_value Maximum settable pwm value in percent [0;100.0]
    *
    * @return true Success
    * @return false Error
    */
   const bool setPWMLimit(const float pwm_max_value);

   /**
    * @brief Set the drive control mode
    *
    * @param mode @ref MotorControlMode
    *
    * @return true Success
    * @return false Error
    */
   const bool setControlMode(const MotorControlMode mode);

   /**
    * @brief Sets the gear ratio of the drive
    *        n_in / n_out = gear_ratio
    *
    * @param gear_ratio Gear ratio of the drive
    *
    * @return true Success
    * @return false Error
    */
   const bool setGearRatio(const float gear_ratio);

   /**
    * @brief Sets the encoder resolution of the drive
    *
    * @param encoder_reso Encoder resolution of the drive
    *
    * @return true Success
    * @return false Error
    */
   const bool setEncoderResolution(const uint16_t encoder_reso);

   /**
    * @brief Set the conversion factor mm/revolution for position
    *        calculation
    *
    * @param conv_fac_mm_per_rev MM per wheel revolution (w/o gearbox)
    *
    * @return true Success
    * @return false Error
    */
   // const bool setConvFacMMPerRev(const float conv_fac_mm_per_rev);

   /**
    * @brief Sets the conversion factor from adc tick/raw value to mm (mm/tick)
    *
    * @param conv_fac_adc_mm_per_rev Conversion factor mm/tick
    *
    * @return true Success
    * @return false Error
    */
   const bool setConvFacAdcMMPerTick(const float conv_fac_adc_mm_per_tick);

   /**
    * @brief Sets an offset value in mm for position calculation via ADC sensor
    *
    * @param offs_adc_mm Offset to add to value in mm
    *
    * @return true Success
    * @return false Error
    */
   const bool setOffsAdcMM(const float offs_adc_mm);

   /**
    * @brief Set the P-part of the position controller
    *
    * @param kp P-part of PID-controller
    *
    * @return true Success
    * @return false Error
    */
   const bool setPositionKp(const float kp);

   /**
    * @brief Set the P-part of the position controller
    *
    * @param ki I-part of PID-controller
    *
    * @return true Success
    * @return false Error
    */
   const bool setPositionKi(const float ki);

   /**
    * @brief Set the D-part of the position controller
    *
    * @param kd D-part of PID-controller
    *
    * @return true Success
    * @return false Error
    */
   const bool setPositionKd(const float kd);

   /**
    * @brief Set the P-part of the speed controller
    *
    * @param kp P-part of PID-controller
    *
    * @return true Success
    * @return false Error
    */
   const bool setSpeedKp(const float kp);

   /**
    * @brief Set the I-part of the speed controller
    *
    * @param ki I-part of PID-controller
    *
    * @return true Success
    * @return false Error
    */
   const bool setSpeedKi(const float ki);

   /**
    * @brief Set the D-part of the speed controller
    *
    * @param kd D-part of PID-controller
    *
    * @return true Success
    * @return false Error
    */
   const bool setSpeedKd(const float kd);

   /**
    * @brief Sets the operation status of the drive enable/disable
    *
    * @param state Target state to set
    *
    * @return true Success
    * @return false Error
    */
   const bool setOperationStatus(const MotorStatus state);

   /**
    * @brief Resets the revolution counter to the
    *        specified value
    *
    * @param init_revs Value the revolutions are setted to
    *
    * @return const Result
    */
   const bool resetRevs(const float init_revs = 0.0f);

   /**
    * @brief Set target pwm value in percent
    *        This value is limited by setPWMLimit()
    *
    * @param value PWM value in percent [-100.0;100.0]
    *
    * @return true Success
    * @return false Error
    */
   const bool setTargetPWM(const float value);

   /**
    * @brief Set target speed in rpm
    *
    * @param value Target speed value in rpm
    *
    * @return true Success
    * @return false Error
    */
   const bool setTargetSpeed(const float value);

   /**
    * @brief Set target position in mm
    *
    * @param value Target position in mm
    *
    * @return true Success
    * @return false Error
    */
   const bool setTargetPosition(const float value);

   /* Getters */
   const MotorStatus getOperationStatus(void);
   const MotorType getType(void);

   const float getCurrentAmp(void);
   const float getPositionMM(void);
   const float getRevolutions(void);
   const float getSpeedRPM(void);

 private:
   /**
    * @brief Constructs a new motor
    *
    * @param id ID of the board
    * @param shield Reference to the shield
    * @param logging Set to true to enable logging output
    */
   Motor(const unsigned int id, MotorShield& shield, const bool logging = false);

   /**
    * @brief Initializes the motor
    *
    * @return true Success
    * @return false Error
    */
   const bool init(void);

   /**
    * @brief Releases the drive
    */
   void release(void);

   /**
    * @brief Updates the drives variabes (called by update thread)
    */
   void update(void);

   const unsigned int _id = 0u; //!< ID of the sensor
   MotorShield& _motor_shield;  //!< Reference of shield instance holding motor

   /* Control parameters of the drive */
   ComDataObject _status;    //!< Status of the motor
   std::mutex _status_mutex; //!< Mutex of motor status

   ComDataObject _tgt_pwm;      //!< Target pwm value in percent [-100.0;100.0]
   ComDataObject _tgt_speed;    //!< Target speed value in rpm
   ComDataObject _tgt_position; //!< Target position

   /* Settings of the drive */
   ComDataObject _type;               //!< Type of the motor
   ComDataObject _ctrl_mode;          //!< Control mode of the drive
   ComDataObject _pwm_max_value;      //!< Absolute maximum pwm value in percent
   ComDataObject _gear_ratio;         //!< Gear ratio
   ComDataObject _encoder_resolution; //!< Encoder resolution
   ComDataObject
       _conv_fac_adc_mm_per_tick; //!< Conversion factor from ADC to mm (mm/tick)
   ComDataObject _offs_adc_mm;    //!< Offset value in mm of lift drive
   ComDataObject _position_kp;    //!< Position controller Kp value
   ComDataObject _position_ki;    //!< Position controller Ki value
   ComDataObject _position_kd;    //!< Position controller Kd value
   ComDataObject _speed_kp;       //!< Speed controller Kp value
   ComDataObject _speed_ki;       //!< Speed controller Ki value
   ComDataObject _speed_kd;       //!< Speed controller Kd value
   ComDataObject
       _reset_revs; //!< Resets the traveled distance to the specified value

   /* Status of the drive */
   ComDataObject _current;     //!< Current in ampere
   ComDataObject _position;    //!< Current position in mm
   ComDataObject _speed;       //!< Current speed in rpm
   ComDataObject _revolutions; //!< Current revolutions (output shaft)

   /** \brief Logging option: set to true to enable logging */
   const bool _logging = false;

   /** \brief Logging module name */
   std::string _log_module = "Motor";

   /** \brief True class is initialized */
   bool _is_initialized = false;

   friend MotorShield;
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

#endif /* EVO_MOTOR_H_ */