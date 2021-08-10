#include "WrapEngine.hpp"

void WrapEngine::init() {
  POWER_FULL_DIFF = MAX_POWER - MIN_POWER;

  pinMode(PIN_FR, OUTPUT);
  pinMode(PIN_FL, OUTPUT);
  pinMode(PIN_RR, OUTPUT);
  pinMode(PIN_RL, OUTPUT);

  motorFR.attach(PIN_FR);
  motorFL.attach(PIN_FL);
  motorRR.attach(PIN_RR);
  motorRL.attach(PIN_RL);

  motorFR.writeMicroseconds(MAX_POWER);
  motorFL.writeMicroseconds(MAX_POWER);
  motorRR.writeMicroseconds(MAX_POWER);
  motorRL.writeMicroseconds(MAX_POWER);
  delay(2000);

  motorFR.writeMicroseconds(MIN_POWER);
  motorFL.writeMicroseconds(MIN_POWER);
  motorRR.writeMicroseconds(MIN_POWER);
  motorRL.writeMicroseconds(MIN_POWER);
  delay(2000);
}

void WrapEngine::apply(uint32_t ms)
{
  if (millis() - _prev_millis > ms) {
    _prev_millis = millis(); // запоминаем момент времени

    // Diag FR <-o-> RL
    if (POWER_FR < MIN_POWER) POWER_FR = MIN_POWER;
    if (POWER_FR > MAX_POWER) POWER_FR = MAX_POWER;
    motorFR.writeMicroseconds(POWER_FR);

    if (POWER_RL < MIN_POWER) POWER_RL = MIN_POWER;
    if (POWER_RL > MAX_POWER) POWER_RL = MAX_POWER;
    motorRL.writeMicroseconds(POWER_RL);

    // Diag FL <-o-> RR
    if (POWER_FL < MIN_POWER) POWER_FL = MIN_POWER;
    if (POWER_FL > MAX_POWER) POWER_FL = MAX_POWER;
    motorFL.writeMicroseconds(POWER_FL);

    if (POWER_RR < MIN_POWER) POWER_RR = MIN_POWER;
    if (POWER_RR > MAX_POWER) POWER_RR = MAX_POWER;
    motorRR.writeMicroseconds(POWER_RR);
  }
}

void WrapEngine::apply(uint32_t ms, float gap_perc)
{
  if (millis() - _prev_millis > ms) {
    _prev_millis = millis(); // запоминаем момент времени

    // Diag FR <-o-> RL
    
    // FR----------------------------------------
    // check dispersion
    if (POWER_FR > POWER_IN_Diag_FRRL) {
      uint16_t max_fr = POWER_IN_Diag_FRRL + (POWER_FULL_DIFF * gap_perc);
      if (POWER_FR > max_fr) POWER_FR = max_fr;
    }
    else {
      uint16_t min_fr = POWER_IN_Diag_FRRL - (POWER_FULL_DIFF * gap_perc);
      if (POWER_FR < min_fr) POWER_FR = min_fr;
    }
    // check min/max
    if (POWER_FR < MIN_POWER) POWER_FR = MIN_POWER;
    if (POWER_FR > MAX_POWER) POWER_FR = MAX_POWER;
    motorFR.writeMicroseconds(POWER_FR);

    // RL----------------------------------------
    // check dispersion
    if (POWER_RL > POWER_IN_Diag_FRRL) {
      uint16_t max_rl = POWER_IN_Diag_FRRL + (POWER_FULL_DIFF * gap_perc);
      if (POWER_RL > max_rl) POWER_RL = max_rl;
    }
    else {
      uint16_t min_rl = POWER_IN_Diag_FRRL - (POWER_FULL_DIFF * gap_perc);
      if (POWER_RL < min_rl) POWER_RL = min_rl;
    }
    // check min/max
    if (POWER_RL < MIN_POWER) POWER_RL = MIN_POWER;
    if (POWER_RL > MAX_POWER) POWER_RL = MAX_POWER;
    motorRL.writeMicroseconds(POWER_RL);

    // Diag FL <-o-> RR
    
    // FL----------------------------------------
    // check dispersion
    if (POWER_FL > POWER_IN_Diag_FLRR) {
      uint16_t max_fl = POWER_IN_Diag_FLRR + (POWER_FULL_DIFF * gap_perc);
      if (POWER_FL > max_fl) POWER_FL = max_fl;
    }
    else {
      uint16_t min_fl = POWER_IN_Diag_FLRR - (POWER_FULL_DIFF * gap_perc);
      if (POWER_FL < min_fl) POWER_FL = min_fl;
    }
    // check min/max
    if (POWER_FL < MIN_POWER) POWER_FL = MIN_POWER;
    if (POWER_FL > MAX_POWER) POWER_FL = MAX_POWER;
    motorFL.writeMicroseconds(POWER_FL);
    
    // RR----------------------------------------
    // check dispersion
    if (POWER_RR > POWER_IN_Diag_FLRR) {
      uint16_t max_rr = POWER_IN_Diag_FLRR + (POWER_FULL_DIFF * gap_perc);
      if (POWER_RR > max_rr) POWER_RR = max_rr;
    }
    else {
      uint16_t min_rr = POWER_IN_Diag_FLRR - (POWER_FULL_DIFF * gap_perc);
      if (POWER_RR < min_rr) POWER_RR = min_rr;
    }
    // check min/max
    if (POWER_RR < MIN_POWER) POWER_RR = MIN_POWER;
    if (POWER_RR > MAX_POWER) POWER_RR = MAX_POWER;
    motorRR.writeMicroseconds(POWER_RR);
  }
}

// TODO: delete uint16_t fr, fl, ...
