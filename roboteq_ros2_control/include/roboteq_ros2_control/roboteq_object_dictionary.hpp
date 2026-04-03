#ifndef ROBOTEQ_OBJECT_DICTIONARY_HPP
#define ROBOTEQ_OBJECT_DICTIONARY_HPP

#include <cstdint>

/// @file roboteq_object_dictionary.hpp
/// @brief Roboteq-specific CANopen object dictionary entries.
///
/// This header defines Roboteq object dictionary keys as index/subindex pairs
/// for use when constructing CANopen SDO requests and interpreting SDO responses.
///
/// @note This initial wrapper assumes the single-channel Roboteq motor controllers
/// used in the current system. Objects that are channel/motor/encoder-based are exposed using
/// subindex 0x01 where applicable.

namespace roboteq
{
  /// @brief Identifies a CANopen object dictionary entry.
  ///
  /// A CANopen object is addressed by a 16-bit object index and an 8-bit subindex.
  /// These values are used in SDO read and write requests.
  struct ObjectKey
  {
    /// @brief 16-bit CANopen object index.
    std::uint16_t index;

    /// @brief 8-bit CANopen object subindex.
    std::uint8_t subindex;
  };

  // Communication Profile Objects
  inline constexpr ObjectKey DeviceType = {0x1000, 0x00};
  inline constexpr ObjectKey ErrorRegister = {0x1001, 0x00};
  inline constexpr ObjectKey ManufacturerDeviceName = {0x1008, 0x00};
  inline constexpr ObjectKey ManufacturerHWVersion = {0x1009, 0x00};
  inline constexpr ObjectKey ManufacturerSWVersion = {0x100A, 0x00};
  inline constexpr ObjectKey GuardTime = {0x100C, 0x00};
  inline constexpr ObjectKey LifeTimeFactor = {0x100D, 0x00};
  inline constexpr ObjectKey ConsumerHeartbeatTime = {0x1016, 0x01};
  inline constexpr ObjectKey ProducerHeartbeatTime = {0x1017, 0x00};
  inline constexpr ObjectKey VendorID = {0x1018, 0x01};

  // Runtime Commands
  inline constexpr ObjectKey SetMotCmd = {0x2000, 0x01};
  inline constexpr ObjectKey SetPosition = {0x2001, 0x01};
  inline constexpr ObjectKey SetVelocity = {0x2002, 0x01};
  inline constexpr ObjectKey SetEncCounter = {0x2003, 0x01};
  inline constexpr ObjectKey SetBrushlessCounter = {0x2004, 0x01};
  inline constexpr ObjectKey SetUsrIntVar = {0x2005, 0x01};
  inline constexpr ObjectKey SetAccel = {0x2006, 0x01};
  inline constexpr ObjectKey SetDecel = {0x2007, 0x01};
  inline constexpr ObjectKey SetAllDigiOut = {0x2008, 0x00};
  inline constexpr ObjectKey SetSingleDigiOut = {0x2009, 0x00};
  inline constexpr ObjectKey ResetSingleDigiOut = {0x200A, 0x00};
  inline constexpr ObjectKey LoadHomeCounter = {0x200B, 0x01};
  inline constexpr ObjectKey EmergencyShutdown = {0x200C, 0x00};
  inline constexpr ObjectKey ReleaseShutdown = {0x200D, 0x00};
  inline constexpr ObjectKey StopInAllModes = {0x200E, 0x00};
  inline constexpr ObjectKey SetPosRel = {0x200F, 0x01};
  inline constexpr ObjectKey SetNextPosAbs = {0x2010, 0x01};
  inline constexpr ObjectKey SetNextPosRel = {0x2011, 0x01};
  inline constexpr ObjectKey SetNextAccel = {0x2012, 0x01};
  inline constexpr ObjectKey SetNextDecel = {0x2013, 0x01};
  inline constexpr ObjectKey SetNextVel = {0x2014, 0x01};
  inline constexpr ObjectKey SetUsrBoolVar = {0x2015, 0x01};
  inline constexpr ObjectKey SaveConfigToFlash = {0x2017, 0x00};
  inline constexpr ObjectKey RunMicroBasicScript = {0x2018, 0x00};
  inline constexpr ObjectKey SetAbsSSICounter = {0x201F, 0x01};
  inline constexpr ObjectKey SafetyStop = {0x202C, 0x00};
  inline constexpr ObjectKey MotSensorSetup = {0x202D, 0x00};
  inline constexpr ObjectKey BrakeOverride = {0x2034, 0x00};

  // Runtime Queries
  inline constexpr ObjectKey ReadMotorAmps = {0x2100, 0x01};
  inline constexpr ObjectKey ReadActMotCmd = {0x2101, 0x01};
  inline constexpr ObjectKey ReadAppliedPwrLvl = {0x2102, 0x01};
  inline constexpr ObjectKey ReadEncMotSpeed = {0x2103, 0x01};
  inline constexpr ObjectKey ReadAbsEncCounter = {0x2104, 0x01};
  inline constexpr ObjectKey ReadAbsBrushless = {0x2105, 0x01};
  inline constexpr ObjectKey ReadUsrIntVar = {0x2106, 0x01};
  inline constexpr ObjectKey ReadRelEncMtrSpeed = {0x2107, 0x01};
  inline constexpr ObjectKey ReadEncCountRel = {0x2108, 0x01};
  inline constexpr ObjectKey ReadBrushlessEncCountRel = {0x2109, 0x01};
  inline constexpr ObjectKey ReadBLMotSpeedInRPM = {0x210A, 0x01};
  inline constexpr ObjectKey ReadRelBLMotSpeed = {0x210B, 0x01};
  inline constexpr ObjectKey ReadBattAmps = {0x210C, 0x01};
  inline constexpr ObjectKey ReadIntVolts = {0x210D, 0x01};
  inline constexpr ObjectKey ReadIntVoltsBatt = {0x210D, 0x02};
  inline constexpr ObjectKey ReadIntVolts5Vout = {0x210D, 0x03};
  inline constexpr ObjectKey ReadAllDigiIn = {0x210E, 0x00};
  inline constexpr ObjectKey ReadMCUTemp = {0x210F, 0x01};
  inline constexpr ObjectKey ReadTransTemp = {0x210F, 0x02};
  inline constexpr ObjectKey ReadFeedback = {0x2110, 0x01};
  inline constexpr ObjectKey ReadStatusFlag = {0x2111, 0x00};
  inline constexpr ObjectKey ReadFaultFlag = {0x2112, 0x00};
  inline constexpr ObjectKey ReadCurrDigitOut = {0x2113, 0x00};
  inline constexpr ObjectKey ReadCloseLoopError = {0x2114, 0x01};
  inline constexpr ObjectKey ReadUsrBoolVar = {0x2115, 0x01};
  inline constexpr ObjectKey ReadIntSerialCmd = {0x2116, 0x01};
  inline constexpr ObjectKey ReadIntAnalogCmd = {0x2117, 0x01};
  inline constexpr ObjectKey ReadIntPulseCmd = {0x2118, 0x01};
  inline constexpr ObjectKey ReadTime = {0x2119, 0x00};
  inline constexpr ObjectKey ReadSpekRadioCap = {0x211A, 0x01};
  inline constexpr ObjectKey DestPosReachFlag = {0x211B, 0x01};
  inline constexpr ObjectKey ReadFOCMotAmps = {0x211C, 0x01};
  inline constexpr ObjectKey ReadMotStatusFlags = {0x2122, 0x01};
  inline constexpr ObjectKey ReadHallSensorState = {0x2123, 0x01};
  inline constexpr ObjectKey ReadLockStatus = {0x2124, 0x00};
  inline constexpr ObjectKey ReadDestTracking = {0x2125, 0x01};
  inline constexpr ObjectKey ReadRotorAngle = {0x2132, 0x01};
  inline constexpr ObjectKey ReadScriptChecksum = {0x2133, 0x00};
  inline constexpr ObjectKey ReadNodeIsAlive = {0x2134, 0x00};
  
}

#endif