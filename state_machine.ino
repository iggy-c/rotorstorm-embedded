String statemachine(float currentaltitude, float previousaltitude, float ppreviousaltitude, float pppreviousaltitude, float ppppreviousaltitude, float pppppreviousaltitude, float maximumaltitude, float bmaltitude) {
  static String state = "LAUNCH_PAD";

  if (launch_pad == true || ascent == true || apogee == true || descent) {
    if (currentaltitude > truemaximumaltitude) {
      truemaximumaltitude = currentaltitude;
    }
  }

  if (currentaltitude < 10 && launch_pad) {
    state = "LAUNCH_PAD";
    launch_pad = true;
    ascent = false;
    apogee = false;
    descent = false;
    probe_release = false;
  }
  
  else if (currentaltitude > 50 && launch_pad) {
    state = "ASCENT";
    ascent = true;
    launch_pad = false;
    apogee = false;
    descent = false;
    probe_release = false;
  }

  else if (currentaltitude > 100 && (currentaltitude - previousaltitude) < 0 && (previousaltitude - ppreviousaltitude < 0) && ascent) {
    state = "APOGEE";
    apogee = true;
    launch_pad = false;
    ascent = false;
    descent = false;
    probe_release = false;
  }

  else if (apogee == true) {
    state = "DESCENT";
    descent = true;
    launch_pad = false;
    ascent = false;
    apogee = false;
    probe_release = false;
  }

  else if (currentaltitude < .75 * truemaximumaltitude && descent == true) {
    state = "PROBE_RELEASE";
    release_servo.write(65); // turns servo to release payload by assigning pos value to 60
    probe_release = true;
    launch_pad = false;
    ascent = false;
    apogee = false;
    descent = false;
  }

  else if (abs((currentaltitude) - (previousaltitude)) < 1 && abs((previousaltitude) - (ppreviousaltitude)) < 1 && abs((ppreviousaltitude) - (pppreviousaltitude)) < 1 && abs((pppreviousaltitude) - (ppppreviousaltitude)) < 1 && abs((ppppreviousaltitude) - (pppppreviousaltitude)) < 1 && probe_release == true){
    state = "LANDED";
  }

  if (descent == true && currentaltitude <= 400) {  //failsafe: if altitude falls below 400m descent bool will be set to true, the string will stay the same
    release_servo.write(60);
  }
  return state;
}