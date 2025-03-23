void statemachine(String & state)
{
// States:
// LAUNCH_PAD
// ASCENT
// APOGEE
// DESCENT
// PROBE_RELEASE
// LANDED
// velocity = change in position in meters per second
//maxaltitude = altitude when altitude values before were slowly increasing and after where slowly decreasing

// float currentaltitude;
// float previousaltitude;
// float ppreviousaltitude;
// float pppreviousaltitude;
// float ppppreviousaltitude;
// float pppppreviousaltitude;
// float maximumaltitude;
bool launch_pad = true;
bool ascent = false;
bool apogee = false;
bool descent = false;
bool probe_release = false;

// if (currentaltitude < previousaltitude && previousaltitude < ppreviousaltitude && ppreviousaltitude < pppreviousaltitude && pppreviousaltitude < ppppreviousaltitude && ppppreviousaltitude < pppppreviousaltitude && apogee == true)
// {
//   maxaltitude = (pppppreviousaltitude + maximumaltitude + bmaltitude) / 3; //averages atltitude of agpogee and right before and right after to create maximum height value
// }


if (bmp.readAltitude(SEALEVELPRESSURE_HPA) < 10)
{
state = "LAUNCH_PAD";
launch_pad = true;

ascent = false;
apogee = false;
descent = false;
probe_release = false;
}

if (bmp.readAltitude(SEALEVELPRESSURE_HPA) > 10 && (abs(currentaltitude) - abs(previousaltitude)) > 2 && (abs(previousaltitude) - abs(ppreviousaltitude)) > 2 && launch_pad == true)
{
state = "ASCENT";
ascent = true;
launch_pad = false;

apogee = false;
descent = false;
probe_release = false;
}

if ((abs(currentaltitude) - abs(previousaltitude)) < (abs(previousaltitude) - abs(ppreviousaltitude)) && (abs(currentaltitude) - abs(previousaltitude)) < 5 && (abs(previousaltitude) - abs(ppreviousaltitude)) < 5 && ascent == true)
{
state = "APOGEE";
apogee = true;
launch_pad = false;
ascent = false;

descent = false;
probe_release = false;
}

if (currentaltitude < previousaltitude && previousaltitude < ppreviousaltitude && ppreviousaltitude < pppreviousaltitude && pppreviousaltitude < ppppreviousaltitude && ppppreviousaltitude < pppppreviousaltitude && apogee == true)
{
  maxaltitude = (pppppreviousaltitude + maximumaltitude + bmaltitude) / 3; //averages atltitude of agpogee and right before and right after to create maximum height value
}

if ((abs(currentaltitude) - abs(previousaltitude)) > 5 && (abs(previousaltitude) - abs(ppreviousaltitude)) > 5 && (abs(currentaltitude) - abs(previousaltitude)) < (abs(previousaltitude) - abs(ppreviousaltitude)) && apogee == true)
{
state = "DESCENT";
descent = true;
launch_pad = false;
ascent = false;
apogee = false;

probe_release = false;
}

if (currentaltitude < .75*maxaltitude && previousaltitude < .75*maxaltitude && descent == true)
{
state = "PROBE_RELEASE";
// myServo.write(20?);
// delay(time);
// digitalWrite(11,LOW);
probe_release = true;
launch_pad = false;
ascent = false;
apogee = false;
descent = false;

}

if ((abs(currentaltitude) - abs(previousaltitude)) < 1 && (abs(previousaltitude) - abs(ppreviousaltitude)) < 1 && (abs(ppreviousaltitude) - abs(pppreviousaltitude)) < 1 && (abs(pppreviousaltitude) - abs(ppppreviousaltitude)) < 1 && (abs(ppppreviousaltitude) - abs(pppppreviousaltitude)) < 1 && probe_release == true)
{
state = "LANDED";
}

}