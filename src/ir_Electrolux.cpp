// Copyright 2019 David Conran

/// @file
/// @brief Support for Electrolux protocols.

#include "ir_Electrolux.h"
#include <algorithm>
#include <cstring>
#ifndef ARDUINO
#include <string>
#endif
#include "IRremoteESP8266.h"
#include "IRtext.h"
#include "IRutils.h"

// Constants

using irutils::addBoolToString;
using irutils::addFanToString;
using irutils::addIntToString;
using irutils::addLabeledString;
using irutils::addModeToString;
using irutils::addTempFloatToString;
using irutils::minsToString;

#if SEND_ELECTROLUX_PO12F_AC
/// Send a Electrolux 112-bit A/C message.
/// Status: Beta / Probably working.
/// @param[in] data The message to be sent.
/// @param[in] nbytes The number of bytes of message to be sent.
/// @param[in] repeat The number of times the command is to be repeated.
void IRsend::sendElectroluxPO12FAC(const unsigned char data[], const uint16_t nbytes,
                                   const uint16_t repeat)
{
  sendGeneric(kElectroluxPO12FACHdrMark, kElectroluxPO12FACHdrSpace,
              kElectroluxPO12FACBitMark, kElectroluxPO12FACOneSpace,
              kElectroluxPO12FACBitMark, kElectroluxPO12FACZeroSpace,
              kElectroluxPO12FACBitMark, kElectroluxPO12FACGap,
              data, nbytes, kElectroluxPO12FACFreq, false, repeat, 50);
}
#endif // SEND_ELECTROLUX_PO12F_AC

/// Class constructor
/// @param[in] pin GPIO to be used when sending.
/// @param[in] inverted Is the output signal to be inverted?
/// @param[in] use_modulation Is frequency modulation to be used?
IRElectroluxPO12FAC::IRElectroluxPO12FAC(const uint16_t pin, const bool inverted,
                                         const bool use_modulation)
    : _irsend(pin, inverted, use_modulation) { stateReset(); }

/// Set up hardware to be able to send a message.
void IRElectroluxPO12FAC::begin(void) { _irsend.begin(); }

#if SEND_ELECTROLUX_PO12F_AC
/// Send the current internal state as an IR message.
/// @param[in] repeat Nr. of times the message will be repeated.
void IRElectroluxPO12FAC::send(const uint16_t repeat)
{
  _irsend.sendElectroluxPO12FAC(getRaw(), kElectroluxPO12FACStateLength, repeat);
}
#endif // SEND_ELECTROLUX_PO12F_AC

/// Calculate the checksum for a given state.
/// @param[in] state The array to calc the checksum of.
/// @param[in] length The length/size of the array.
/// @return The calculated checksum value.
uint8_t IRElectroluxPO12FAC::calcChecksum(uint8_t state[], const uint16_t length)
{
  if (length)
    return sumBytes(state, length - 1);
  else
    return 0;
}

/// Calculate & set the checksum for the current internal state of the remote.
/// @param[in] length The length/size of the internal array to checksum.
void IRElectroluxPO12FAC::checksum(const uint16_t length)
{
  // Stored the checksum value in the last byte.
  if (length > 1)
    _.Sum = calcChecksum(_.raw, length);
}

/// Verify the checksum is valid for a given state.
/// @param[in] state The array to verify the checksum of.
/// @param[in] length The length/size of the array.
/// @return true, if the state has a valid checksum. Otherwise, false.
bool IRElectroluxPO12FAC::validChecksum(uint8_t state[], const uint16_t length)
{
  return (length > 1 && state[length - 1] == calcChecksum(state, length));
}

/// Reset the internal state of the emulation. (On, Cool, 24C)
void IRElectroluxPO12FAC::stateReset(void)
{
  // A known good state. (On, Cool, 24C)
  std::memcpy(_.raw, kElectroluxPO12FACDefaultState, kElectroluxPO12FACStateLength);
}

/// Get a PTR to the internal state/code for this protocol.
/// @return PTR to a code for this protocol based on the current internal state.
uint8_t *IRElectroluxPO12FAC::getRaw(void)
{
  checksum();
  return _.raw;
}

/// Set the internal state from a valid code for this protocol.
/// @param[in] new_code A valid code for this protocol.
/// @param[in] length The length/size of the new_code array.
void IRElectroluxPO12FAC::setRaw(const uint8_t new_code[], const uint16_t length)
{
  std::memcpy(_.raw, new_code, std::min(length, kElectroluxPO12FACStateLength));
}

/// Set the requested power state of the A/C to on.
void IRElectroluxPO12FAC::on(void) { setPower(true); }

/// Set the requested power state of the A/C to off.
void IRElectroluxPO12FAC::off(void) { setPower(false); }

/// Change the power setting.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRElectroluxPO12FAC::setPower(const bool on)
{
  _.Power = on;
}

/// Get the value of the current power setting.
/// @return true, the setting is on. false, the setting is off.
bool IRElectroluxPO12FAC::getPower(void) const
{
  return _.Power;
}

/// Get the operating mode setting of the A/C.
/// @return The current operating mode setting.
uint8_t IRElectroluxPO12FAC::getMode(void) const
{
  return _.Mode;
}

/// Set the operating mode of the A/C.
/// @param[in] mode The desired operating mode.
/// @note Fan/Ventilation mode sets the fan speed to high.
///   Unknown values default to Auto.
void IRElectroluxPO12FAC::setMode(const uint8_t mode)
{
  // If we get an unexpected mode, default to AUTO.
  switch (mode)
  {
  case kElectroluxPO12FACFan:
    setFan(kElectroluxPO12FACFanHigh);
    // FALLTHRU
  case kElectroluxPO12FACAuto:
  case kElectroluxPO12FACCool:
  case kElectroluxPO12FACHeat:
  case kElectroluxPO12FACDry:
    _.Mode = mode;
    break;
  default:
    _.Mode = kElectroluxPO12FACAuto;
  }
}

/// Set the temperature.
/// @param[in] celsius The temperature in degrees celsius.
/// @note The temperature resolution is 0.5 of a degree.
void IRElectroluxPO12FAC::setTemp(const float celsius)
{
  // Make sure we have desired temp in the correct range.
  float safecelsius = std::max(celsius, kElectroluxPO12FACTempMin);
  safecelsius = std::min(safecelsius, kElectroluxPO12FACTempMax);
  _.Temp = static_cast<uint8_t>(kElectroluxPO12FACTempMax - safecelsius + 3);
}

/// Get the current temperature setting.
/// @return The current setting for temp. in degrees celsius.
/// @note The temperature resolution is 0.5 of a degree.
float IRElectroluxPO12FAC::getTemp(void) const
{
  float result = kElectroluxPO12FACTempMax + _.Temp - 3;
  return result;
}

/// Set the speed of the fan.
/// @param[in] speed The desired setting.
/// @note Unknown speeds will default to Auto.
void IRElectroluxPO12FAC::setFan(const uint8_t speed)
{
  switch (speed)
  {
  case kElectroluxPO12FACFanAuto:
  case kElectroluxPO12FACFanLow:
  case kElectroluxPO12FACFanMed:
  case kElectroluxPO12FACFanHigh:
    _.Fan = speed;
    break;
  default:
    _.Fan = kElectroluxPO12FACFanAuto;
  }
}

/// Get the current fan speed setting.
/// @return The current fan speed/mode.
uint8_t IRElectroluxPO12FAC::getFan(void) const
{
  return _.Fan;
}

/// Set the economy setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRElectroluxPO12FAC::setEcono(const bool on)
{
  if (on)
    _.Fan = kElectroluxPO12FACFanEco;
}

/// Get the economy setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRElectroluxPO12FAC::getEcono(void) const { return _.Fan == kElectroluxPO12FACFanEco; }

/// Set the horizontal swing setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRElectroluxPO12FAC::setSwing(const bool on)
{
  _.Swing = on;
}

/// Get the horizontal swing setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRElectroluxPO12FAC::getSwing(void) const
{
  return _.Swing;
}

/// Set the Turbo setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRElectroluxPO12FAC::setTurbo(const bool on)
{
  _.Turbo = on;
  if (on)
  {
    _.Fan = kElectroluxPO12FACFanHigh;
    setTemp(18);
  }
}

/// Get the Turbo setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRElectroluxPO12FAC::getTurbo(void) const
{
  return _.Turbo;
}

/// Set the Follow me setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRElectroluxPO12FAC::setFollow(const bool on)
{
  _.Follow = on;
}

/// Get the Follow me setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRElectroluxPO12FAC::getFollow(void) const
{
  return _.Follow;
}

/// Set the Sleep off timer setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRElectroluxPO12FAC::setOffSleepTimer(const uint8_t hours)
{
  _.OffSleepTimer = hours * 6;
}

/// Get the Sleep off timer setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
uint8_t IRElectroluxPO12FAC::getOffSleepTimer(void) const
{
  return _.OffSleepTimer;
}

/// Convert a stdAc::opmode_t enum into its native mode.
/// @param[in] mode The enum to be converted.
/// @return The native equivalent of the enum.
uint8_t IRElectroluxPO12FAC::convertMode(const stdAc::opmode_t mode)
{
  switch (mode)
  {
  case stdAc::opmode_t::kCool:
    return kElectroluxPO12FACCool;
  case stdAc::opmode_t::kHeat:
    return kElectroluxPO12FACHeat;
  case stdAc::opmode_t::kDry:
    return kElectroluxPO12FACDry;
  case stdAc::opmode_t::kFan:
    return kElectroluxPO12FACFan;
  default:
    return kElectroluxPO12FACAuto;
  }
}

/// Convert a stdAc::fanspeed_t enum into it's native speed.
/// @param[in] speed The enum to be converted.
/// @return The native equivalent of the enum.
uint8_t IRElectroluxPO12FAC::convertFan(const stdAc::fanspeed_t speed)
{
  switch (speed)
  {
  case stdAc::fanspeed_t::kMin:
  case stdAc::fanspeed_t::kLow:
    return kElectroluxPO12FACFanLow;
  case stdAc::fanspeed_t::kMedium:
    return kElectroluxPO12FACFanMed;
  case stdAc::fanspeed_t::kHigh:
  case stdAc::fanspeed_t::kMax:
    return kElectroluxPO12FACFanHigh;
  default:
    return kElectroluxPO12FACFanAuto;
  }
}

/// Convert a native mode into its stdAc equivalent.
/// @param[in] mode The native setting to be converted.
/// @return The stdAc equivalent of the native setting.
stdAc::opmode_t IRElectroluxPO12FAC::toCommonMode(const uint8_t mode)
{
  switch (mode)
  {
  case kElectroluxPO12FACCool:
    return stdAc::opmode_t::kCool;
  case kElectroluxPO12FACHeat:
    return stdAc::opmode_t::kHeat;
  case kElectroluxPO12FACDry:
    return stdAc::opmode_t::kDry;
  case kElectroluxPO12FACFan:
    return stdAc::opmode_t::kFan;
  default:
    return stdAc::opmode_t::kAuto;
  }
}

/// Convert a native fan speed into its stdAc equivalent.
/// @param[in] spd The native setting to be converted.
/// @return The stdAc equivalent of the native setting.
stdAc::fanspeed_t IRElectroluxPO12FAC::toCommonFanSpeed(const uint8_t spd)
{
  switch (spd)
  {
  case kElectroluxPO12FACFanHigh:
    return stdAc::fanspeed_t::kMax;
  case kElectroluxPO12FACFanMed:
    return stdAc::fanspeed_t::kMedium;
  case kElectroluxPO12FACFanLow:
    return stdAc::fanspeed_t::kMin;
  default:
    return stdAc::fanspeed_t::kAuto;
  }
}

/// Convert the current internal state into its stdAc::state_t equivalent.
/// @return The stdAc equivalent of the native settings.
stdAc::state_t IRElectroluxPO12FAC::toCommon(void) const
{
  stdAc::state_t result;
  result.protocol = decode_type_t::ELECTROLUX_PO12F_AC;
  result.model = -1; // Not supported.
  result.power = _.Power;
  result.mode = toCommonMode(_.Mode);
  result.celsius = true;
  result.degrees = getTemp();
  result.fanspeed = toCommonFanSpeed(_.Fan);
  result.swingv = _.Swing ? stdAc::swingv_t::kAuto : stdAc::swingv_t::kOff;
  result.swingh = _.Swing ? stdAc::swingh_t::kAuto : stdAc::swingh_t::kOff;
  result.turbo = _.Turbo;
  result.econo = getEcono();
  result.sleep = _.OffSleepTimer;
  // Not supported.
  result.light = false;
  result.filter = false;
  result.quiet = false;
  result.clean = false;
  result.beep = false;
  result.clock = -1;
  return result;
}

/// Convert the current internal state into a human readable string.
/// @return A human readable string.
String IRElectroluxPO12FAC::toString(void) const
{
  String result = "";
  result.reserve(140); // Reserve some heap for the string to reduce fragging.
  result += addBoolToString(_.Power, kPowerStr, false);
  result += addModeToString(_.Mode, kElectroluxPO12FACAuto, kElectroluxPO12FACCool,
                            kElectroluxPO12FACHeat, kElectroluxPO12FACDry, kElectroluxPO12FACFan);
  if (_.Mode == kElectroluxPO12FACCool)
    result += addTempFloatToString(getTemp());
  result += addFanToString(_.Fan, kElectroluxPO12FACFanHigh, kElectroluxPO12FACFanLow,
                           kElectroluxPO12FACFanAuto, kElectroluxPO12FACFanAuto, kElectroluxPO12FACFanMed);
  result += addBoolToString(getEcono(), kEconoStr);
  result += addBoolToString(_.Turbo, kTurboStr);
  result += addBoolToString(_.Swing, kSwingHStr);
  result += addBoolToString(_.Swing, kSwingVStr);
  result += addBoolToString(_.Follow, kFollowStr);
  float mins = 0;
  if (_.OffSleepTimer > 0)
  {
    // Only hours supported in Electrolux devices
    mins = (_.OffSleepTimer / 6) * 60;
  }
  result += addLabeledString(mins ? minsToString(mins) : kOffStr, kOffTimerStr);
  return result;
}

#if DECODE_ELECTROLUX_PO12F_AC
bool IRrecv::decodeElectroluxPO12FAC(decode_results *results, uint16_t offset,
                                     const uint16_t nbits, const bool strict)
{
  // It's the same as `decodeMitsubishi112()`. A shared routine is used.
  if (!IRrecv::decodeMitsubishi112(results, offset, nbits, strict))
    return false;
  results->decode_type = decode_type_t::ELECTROLUX_PO12F_AC;
  results->bits = nbits;
  return true;
}
#endif // DECODE_ELECTROLUX_PO12F_AC
