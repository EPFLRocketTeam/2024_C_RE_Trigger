//Light e-match
void setSecondEventCurrentImpulse() {
  digitalWrite(9,HIGH);
}

//turn e-match off
void resetSecondEventCurrentImpulse() {
  digitalWrite(9,LOW);
}

void secondEvent() {
  if((states.minimumHeight == true)) {
    if(states.states == READY) {
      if (barometerHeight() < 400) {
        setSecondEventCurrentImpulse();
        delay(50);
        resetSecondEventCurrentImpulse();
      }
    }
  }
}
