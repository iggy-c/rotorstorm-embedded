String statemachine(float currentaltitude, float previousaltitude, float ppreviousaltitude, float pppreviousaltitude, float ppppreviousaltitude, float pppppreviousaltitude, float maximumaltitude, float bmaltitude)
{
 static String state = "LAUNCH_PAD";
// States:
// LAUNCH_PAD
// ASCENT
// APOGEE
// DESCENT
// PROBE_RELEASE
// LANDED
// velocity = change in position in meters per second
//maxaltitude = altitude when altitude values before were slowly increasing and after where slowly decreasing
if (launch_pad == true || ascent == true || apogee == true || descent)
{
  if (currentaltitude > truemaximumaltitude)
  {
    truemaximumaltitude = currentaltitude;
  }
}
// float currentaltitude;
// float previousaltitude;
// float ppreviousaltitude;
// float pppreviousaltitude;
// float ppppreviousaltitude;
// float pppppreviousaltitude;
// float maximumaltitude;
// bool launch_pad = true;
// bool ascent = false;
// bool apogee = false;
// bool descent = false;
// bool probe_release = false;

// if (currentaltitude < previousaltitude && previousaltitude < ppreviousaltitude && ppreviousaltitude < pppreviousaltitude && pppreviousaltitude < ppppreviousaltitude && ppppreviousaltitude < pppppreviousaltitude && apogee == true)
// {
//   maxaltitude = (pppppreviousaltitude + maximumaltitude + bmaltitude) / 3; //averages atltitude of agpogee and right before and right after to create maximum height value
// }

if (currentaltitude < 10 && launch_pad)
{
state = "LAUNCH_PAD";
 launch_pad = true;
 ascent = false;
 apogee = false;
 descent = false;
probe_release = false;


}
else if (currentaltitude > 50 && launch_pad) //&& launch_pad == true
{

state = "ASCENT";
ascent = true;
launch_pad = false;

apogee = false;
descent = false;
probe_release = false;
}

else if (currentaltitude > 100 && (currentaltitude - previousaltitude) < 0 && (previousaltitude - ppreviousaltitude < 0) && ascent)
{


state = "APOGEE";
apogee = true;
launch_pad = false;
ascent = false;

descent = false;
probe_release = false;

}

// static bool maxaltitude_craeted = false;
// if (currentaltitude < previousaltitude && previousaltitude < ppreviousaltitude && ppreviousaltitude < pppreviousaltitude && pppreviousaltitude < ppppreviousaltitude && ppppreviousaltitude < pppppreviousaltitude && apogee == true && maxaltitude_craeted == false)
// {
//     Serial.println("State 3");

  
// maxaltitude = (pppppreviousaltitude + maximumaltitude + bmaltitude) / 3; //averages atltitude of agpogee and right before and right after to create maximum height value
// maxaltitude_craeted = true;
// //may need to then make this a const int to pass for refernce laterprolly wont work otherwise
// }



else if (apogee == true)
{


state = "DESCENT";
descent = true;
launch_pad = false;
ascent = false;
apogee = false;

probe_release = false;

}
else if (currentaltitude < .75*truemaximumaltitude && descent == true)
{


state = "PROBE_RELEASE";
// myServo.write(20?);
// delay(time);
// digitalWrite(11,LOW);
release_servo.write(65); //turns servo to release payload by assigning pos value to 60
probe_release = true;
launch_pad = false;
ascent = false;
apogee = false;
descent = false;

}

else if (abs((currentaltitude) - (previousaltitude)) < 1 && abs((previousaltitude) - (ppreviousaltitude)) < 1 && abs((ppreviousaltitude) - (pppreviousaltitude)) < 1 && abs((pppreviousaltitude) - (ppppreviousaltitude)) < 1 && abs((ppppreviousaltitude) - (pppppreviousaltitude)) < 1 && probe_release == true)
{


state = "LANDED";
}

if (descent == true && currentaltitude <= 400){
  release_servo.write(60);
}
return state;
}