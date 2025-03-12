void statemachine(float bmp.readAltitude(SEALEVELPRESSURE_HPA), float velocity, String & state)
// States:
// LAUNCH_PAD
// ASCENT
// APOGEE
// DESCENT
// PROBE_RELEASE
// LANDED
// velocity = change in position in meters per second
//maxaltitude = altitude when altitude values before were slowly increasing and after where slowly decreasing
//maxaltitude = 

if (currentaltitude < altitude 1 second ago = true for five times after velocity has been low)
{
  maxaltitude = the value average of the 3 altitude values taken 
}
if (bmp.readAltitude(SEALEVELPRESSURE_HPA) <= 10)
{
state = LAUNCH_PAD;
launch_pad = true;
}

if (bmp.readAltitude(SEALEVELPRESSURE_HPA) > 10 && velocity > 2 && launch_pad == true)
{
state = ASCENT;
ascent = true;
}

if (velocity < 2 for at least 3 seconds && ascent == true)
{
state = APOGEE;
apogee = true;
}

if(bmp.readAltitude(SEALEVELPRESSURE_HPA) < maxaltitude && velocity > 2 && apogee == true)
{
state = DESCENT;
descent = true;
}

if (bmp.readAltitude(SEALEVELPRESSURE_HPA) <= 75%maxaltitude for at least 2 seconds && velocity >2 && descent == true)
{
state = PROBE_RELEASE;
myServo.write(20?);
delay(time);
digitalWrite(11,LOW);
probe_release = true;
}

if (bmp.readAltitude(SEALEVELPRESSURE_HPA) < 50%maxaltitude && velocity < 0.5 for at least 5 seconds && probe_release == true)
{
state = LANDED;
}
