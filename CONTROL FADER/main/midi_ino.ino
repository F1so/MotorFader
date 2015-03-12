
void updateFaderMidi() {
  int  velocity    = faderPosition();
  byte channelData = 0xE0 + (faderChannel - 1);
  // MIDI Message:
  Serial.write(channelData);               //  E(PitchBend)  Channel (0-9)
  Serial.write(velocity & 0x7F);           //  Least Sig Bits of Data
  Serial.write((velocity >> 7) & 0x7F);    //  Most  Sig Bits of Data
}


//Returns a MIDI pitch bend value for the fader's current position
//Cases ensure that there is a -infinity (min) and max value despite possible math error
int faderPosition() {
  int position = analogRead(faderPosPin);
  int returnValue = 0;

  if (position <= faderMin) {
    returnValue = 0;
  }
  else if (position >= faderMax) {
    returnValue = 16383;
  }
  else {
    returnValue = ((float)(position - faderMin) / (faderMax - faderMin)) * 16383;		 
  }

  return returnValue;
}

