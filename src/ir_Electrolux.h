// Copyright 2019 David Conran

/// @file
/// @brief Support for Electrolux protocols.

// Supports:
//   Brand: Leberg,  Model: LBS-TOR07 A/C

#ifndef IR_ELECTROLUX_H_
#define IR_ELECTROLUX_H_

#ifndef UNIT_TEST
#include <Arduino.h>
#endif
#include "IRremoteESP8266.h"
#include "IRsend.h"
#include "IRrecv.h"
#ifdef UNIT_TEST
#include "IRsend_test.h"
#endif

/// Native representation of a TCL 112 A/C message.
union ElectroluxPO12FProtocol
{
  uint8_t raw[kElectroluxPO12FACStateLength]; ///< The State in IR code form.
  struct
  {
    // Byte 0~4
    uint8_t pad0[5];
    // Byte 5
    uint8_t : 2;
    uint8_t Power : 1;
    uint8_t : 5;
    // Byte 6
    uint8_t Mode : 4;
    uint8_t : 2;
    uint8_t Turbo : 1;
    uint8_t Follow : 1;
    // Byte 7
    uint8_t Temp : 4;
    uint8_t : 4;
    // Byte 8
    uint8_t Fan : 3;
    uint8_t Swing : 3;
    uint8_t : 2;
    // Byte 9
    uint8_t OffSleepTimer : 8;
    // Byte 10~11
    uint8_t pad1[2];
    // Byte 12
    uint8_t : 8;
    // Byte 13
    uint8_t Sum : 8;
  };
};

// Constants
const uint16_t kElectroluxPO12FACHdrMark = 3000;
const uint16_t kElectroluxPO12FACHdrSpace = 1650;
const uint16_t kElectroluxPO12FACBitMark = 500;
const uint16_t kElectroluxPO12FACOneSpace = 1050;
const uint16_t kElectroluxPO12FACZeroSpace = 325;
const uint32_t kElectroluxPO12FACGap = kDefaultMessageGap; // Just a guess.
// Total tolerance percentage to use for matching the header mark.
const uint8_t kElectroluxPO12FACHdrMarkTolerance = 6;
const uint8_t kElectroluxPO12FACTolerance = 5; // Extra Percentage for the rest.
const uint16_t kElectroluxPO12FACFreq = 38000;

const uint8_t kElectroluxPO12FACDefaultState[] = {0x23, 0xCB, 0x26,
                                                  0x01, 0x00, 0x24,
                                                  0x03, 0x05, 0x02,
                                                  0x00, 0x00, 0x1B,
                                                  0x03, 0x61};

const uint8_t kElectroluxPO12FACHeat = 1;
const uint8_t kElectroluxPO12FACDry = 2;
const uint8_t kElectroluxPO12FACCool = 3;
const uint8_t kElectroluxPO12FACFan = 7;
const uint8_t kElectroluxPO12FACAuto = 8;

const uint8_t kElectroluxPO12FACFanAuto = 0b000;
const uint8_t kElectroluxPO12FACFanLow = 0b010;
const uint8_t kElectroluxPO12FACFanMed = 0b011;
const uint8_t kElectroluxPO12FACFanHigh = 0b101;
const uint8_t kElectroluxPO12FACFanEco = 0b001;

const float kElectroluxPO12FACTempMax = 28.0;
const float kElectroluxPO12FACTempMin = 18.0;

const uint8_t kElectroluxPO12FACSwingVOn = 0b111;
const uint8_t kElectroluxPO12FACSwingVOff = 0b000;

// Classes
/// Class for handling detailed TCL A/C messages.
class IRElectroluxPO12FAC
{
public:
  explicit IRElectroluxPO12FAC(const uint16_t pin, const bool inverted = false,
                               const bool use_modulation = true);
#if SEND_ELECTROLUX_PO12F_AC
  void send(const uint16_t repeat = kElectroluxPO12FACDefaultRepeat);
  /// Run the calibration to calculate uSec timing offsets for this platform.
  /// @return The uSec timing offset needed per modulation of the IR Led.
  /// @note This will produce a 65ms IR signal pulse at 38kHz.
  ///   Only ever needs to be run once per object instantiation, if at all.
  int8_t calibrate(void) { return _irsend.calibrate(); }
#endif // SEND_TCL
  void begin(void);
  void stateReset(void);
  uint8_t *getRaw(void);
  void setRaw(const uint8_t new_code[],
              const uint16_t length = kElectroluxPO12FACStateLength);
  void on(void);
  void off(void);
  void setPower(const bool on);
  bool getPower(void) const;
  void setTemp(const float celsius); // Celsius in 0.5 increments
  float getTemp(void) const;
  void setMode(const uint8_t mode);
  uint8_t getMode(void) const;
  static uint8_t calcChecksum(uint8_t state[],
                              const uint16_t length = kElectroluxPO12FACStateLength);
  static bool validChecksum(uint8_t state[],
                            const uint16_t length = kElectroluxPO12FACStateLength);
  void setFan(const uint8_t speed);
  uint8_t getFan(void) const;
  void setEcono(const bool on);
  bool getEcono(void) const;
  void setSwing(const bool on);
  bool getSwing(void) const;
  void setTurbo(const bool on);
  bool getTurbo(void) const;
  void setFollow(const bool on);
  bool getFollow(void) const;
  void setOffSleepTimer(const uint8_t on);
  uint8_t getOffSleepTimer(void) const;
  static uint8_t convertMode(const stdAc::opmode_t mode);
  static uint8_t convertFan(const stdAc::fanspeed_t speed);
  static stdAc::opmode_t toCommonMode(const uint8_t mode);
  static stdAc::fanspeed_t toCommonFanSpeed(const uint8_t speed);
  stdAc::state_t toCommon(void) const;
  String toString(void) const;
#ifndef UNIT_TEST

private:
  IRsend _irsend; ///< Instance of the IR send class
#else             // UNIT_TEST
  /// @cond IGNORE
  IRsendTest _irsend; ///< Instance of the testing IR send class
                      /// @endcond
#endif            // UNIT_TEST
  ElectroluxPO12FProtocol _;
  void checksum(const uint16_t length = kElectroluxPO12FACStateLength);
};

#endif // IR_ELECTROLUX_H_
