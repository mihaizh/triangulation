#include <L3G4200D.h>
#include <Wire.h>
#include <math.h>

const float CTG_MAX = 100000000.f;
const float CTG_MIN = -CTG_MAX;
const float DEG2RAD = PI / 180.f;
const float EPSILON = 1e-6;

const String comma(",");

inline float minmax(float minimum, float value, float maximum)
{
    return ((value > maximum) ? maximum : ((value < minimum) ? minimum : value));
}

inline float ctg(float value)
{
    return 1.f / tan(value);
}

inline void doOneStep(byte stepPin, short motorDelay, unsigned long& lastCommandTime, short& stepVal)
{
    unsigned long pt = micros() - lastCommandTime;
    if (pt < motorDelay)
    {
        delayMicroseconds(motorDelay - pt);
    }
    
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(motorDelay);
    digitalWrite(stepPin, LOW);
    //delayMicroseconds(motorDelay);

    ++stepVal;
    lastCommandTime = micros();
}

struct point
{
    int x;
    int y;
};

const byte MAX_BEACONS = 3;
const point beaconsLoc[MAX_BEACONS] = {
    { 60, 175 },
    { 180, 0 },
    { 0, 0 }
};

const byte stepPin = 3; ///< pin number for step
const byte dirPin = 4;  ///< pin number for direction

const short minDelayMotor = 600;   ///< minimum delay for stepper
const short accMotor = 2;           ///< motor acceleration

const byte laserInterruptPin = 2;   ///< pin number for laser interrupt

short motorDelay = 3500;    ///< motor speed (lower is faster)
short stepVal = 0;          ///< current step of motor

const byte MAX_ANGLES = 3;                      ///< maximum measureable angles
float angles[MAX_ANGLES] = { 0.F, 0.F, 0.F };   ///< measured angles
byte anglesCount = 0;                           ///< number of measured angles

L3G4200D gyro;
unsigned long lastCycleTime = 0u; ///< last time angle was measured
float yaw = 0.f; ///< current measured yaw

float triangulation(float &x, float &y,
                    float alpha1, float alpha2, float alpha3,
                    float x1, float y1, float x2, float y2, float x3, float y3)
{
    float cot_12 = ctg(alpha2 - alpha1);
    float cot_23 = ctg(alpha3 - alpha2);
    cot_12 = minmax(CTG_MIN, cot_12, CTG_MAX);
    cot_23 = minmax(CTG_MIN, cot_23, CTG_MAX);
    float cot_31 = (1.0 - (cot_12 * cot_23)) / (cot_12 + cot_23);
    cot_31 = minmax(CTG_MIN, cot_31, CTG_MAX);
    
    const float x1_ = x1 - x2;
    const float y1_ = y1 - y2;
    const float x3_ = x3 - x2;
    const float y3_ = y3 - y2;

    const float c12x = x1_ + cot_12 * y1_;
    const float c12y = y1_ - cot_12 * x1_;

    const float c23x = x3_ - cot_23 * y3_;
    const float c23y = y3_ + cot_23 * x3_;

    const float c31x = (x3_ + x1_) + cot_31 * (y3_ - y1_);
    const float c31y = (y3_ + y1_) - cot_31 * (x3_ - x1_);
    const float k31 = (x3_ * x1_) + (y3_ * y1_) + cot_31 * ( (y3_ * x1_) - (x3_ * y1_) );
  
    const float D = (c12x - c23x) * (c23y - c31y) - (c23x - c31x) * (c12y - c23y);
    float invD = -1.f;
    
    if (abs(D) > EPSILON)
    {
        invD = 1.f / D;
        const float K = k31 * invD;
          
        x = K * (c12y - c23y) + x2;
        y = K * (c23x - c12x) + y2;
    }
    
    return invD; /* return 1/D */
}

void laserInterrupt()
{
    // determine which was the last measured angle
    const byte lastAngleIndex = (anglesCount == 0) ? (MAX_ANGLES - 1) : (anglesCount - 1);
    // calculate the current angle
    const float angle = stepVal * 0.9f;
    // validate measured angle
    if (abs(angle - angles[lastAngleIndex]) > 10.f)
    {
        // save angle
        angles[anglesCount] = angle;
        ++anglesCount;
    }
}

void setup()
{
    // set driver pins mode
    pinMode(stepPin, OUTPUT); 
    pinMode(dirPin, OUTPUT);

    // set motor direction
    digitalWrite(dirPin, HIGH);

    // enable interrupt
    pinMode(laserInterruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(laserInterruptPin), laserInterrupt, FALLING);

    // serial
    Serial.begin(115200);

    Wire.begin();
    gyro.initialize(2000);
    delay(1500); // wait for sensor to be ready
}

void loop()
{
    // make motor rotate: 400 = 360 deg
    // skip last 4 steps, and do them while making calculations to avoid motor being stuck
    for(stepVal = 0; stepVal < 395; ++stepVal)
    {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(motorDelay);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(motorDelay);
        
        if (motorDelay > minDelayMotor)
        {
            // accelerate
            motorDelay -= accMotor;
        }
    }
    
    unsigned long lastCommandTime = micros();
    doOneStep(stepPin, motorDelay, lastCommandTime, stepVal);
    
    const unsigned long now = micros();
    const unsigned long dt = (now - lastCycleTime);
    lastCycleTime = now;

    // update yaw
    const float z = gyro.getY() * 0.07f * dt / 1000000.f;
    yaw = yaw - z - 0.2f;

    doOneStep(stepPin, motorDelay, lastCommandTime, stepVal);

    // if enough angles were measured, do triangulation
    if (anglesCount == 3)
    {
        // compensate angles with current yaw
        for (byte i = 0; i < anglesCount; ++i)
        {
            //angles[i] += yaw;
        }

        doOneStep(stepPin, motorDelay, lastCommandTime, stepVal);

        const byte fb = (yaw < 90.f) ? 0 : (yaw < 180.f) ? 1 : 2;
        const byte sb = (fb + 1) % MAX_BEACONS;
        const byte tb = (fb + 2) % MAX_BEACONS;

        // calculate x and y of the robot
        float x = 0.f, y = 0.f;
        const float err = triangulation(x, y,
                                        angles[0] * DEG2RAD, angles[1] * DEG2RAD, angles[2] * DEG2RAD,
                                        beaconsLoc[fb].x, beaconsLoc[fb].y,
                                        beaconsLoc[sb].x, beaconsLoc[sb].y,
                                        beaconsLoc[tb].x, beaconsLoc[tb].y);

        doOneStep(stepPin, motorDelay, lastCommandTime, stepVal);

        if (err > 0.f)
        {
            Serial.println(String(x) + comma + String(y) + comma + String(yaw));
        }

        doOneStep(stepPin, motorDelay, lastCommandTime, stepVal);
    }

    for (; stepVal < 400; ++stepVal)
    {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(motorDelay);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(motorDelay);
    }

    // reset measured angles
    anglesCount = 0;
    angles[0] = 0.f;
    angles[1] = 0.f;
    angles[2] = 0.f;
}
