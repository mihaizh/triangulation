#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <math.h>
#include <stdint.h>

#define _DEBUG_MSG

const float CTG_MAX = 100000000.f;
const float CTG_MIN = -CTG_MAX;
const float DEG2RAD = PI / 180.f;
const float RAD2DEG = 180.f / PI;
const float EPSILON = 1e-6;

inline float minmax(float minimum, float value, float maximum)
{
    return ((value > maximum) ? maximum : ((value < minimum) ? minimum : value));
}

inline float ctg(float value)
{
    return 1.f / tan(value);
}

struct point
{
    int16_t x;
    int16_t y;
};

const byte MAX_BEACONS = 3;             ///< beacons on scene
const point beaconsLoc[MAX_BEACONS] = { ///< beacons locations
    { 60, 175 },
    { 180, 0 },
    { 0, 0 }
};

const byte stepPin = 4; ///< pin number for step
const byte dirPin = 5;  ///< pin number for direction

const byte laserInterruptPin = 2;   ///< pin number for laser interrupt
const byte mpuInterruptPin = 3;     ///< pin number for mpu interrupt

int16_t stepVal = 0;          ///< current step of motor

const int16_t freqHigh = 877; ///< ~3.5ms
const int16_t freqLow = 150;  ///< ~0.6ms

const byte MAX_ANGLES = 3;                      ///< maximum measureable angles
float angles[MAX_ANGLES] = { 0.F, 0.F, 0.F };   ///< measured angles
byte anglesCount = 0;                           ///< number of measured angles

MPU6050 mpu;

bool dmpReady = false;  ///< set true if DMP init was successful
uint8_t mpuIntStatus;   ///< holds actual interrupt status byte from MPU
uint8_t devStatus;      ///< return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    ///< expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     ///< count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; ///< FIFO storage buffer
float ypr[3];           ///< [yaw, pitch, roll]
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t yaw = 0;

float x = 0.f;
float y = 0.f;

// Estimates robot position based on measured angles to each beacon
// and beacon locations.
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

// send robot odometry [x, y, yaw] through I2C
byte data[sizeof(float) * 3];
inline void sendOdometry(float x, float y, float yaw)
{
    memcpy(&data[0], &x, sizeof(x));
    memcpy(&data[sizeof(x)], &y, sizeof(y));
    memcpy(&data[sizeof(x) + sizeof(y)], &yaw, sizeof(yaw));

    Wire.beginTransmission(10); // TBD
    Wire.write(data, sizeof(data));
    Wire.endTransmission();
}

bool pwm_high = true;               ///< indicates if pwm should be HIGH or LOW for this half of cycle
int16_t crtFreq = freqHigh;         ///< current frequency (lowers in time -> motor accelerates)
const short rotationSteps = 400;    ///< how many steps means a complete rotation
bool rotationDone = false;
ISR(TIMER1_COMPA_vect)
{
    // no need to deactivate/activate global interrupts?
    // all interrupts are not "heavy" on computation

    // writes current pwm level
    digitalWrite(stepPin, pwm_high);
    // pwm toggle
    pwm_high = !pwm_high;

    // count steps
    // assumption: true = 1, false = 0
    stepVal += pwm_high;
    if (stepVal == rotationSteps)
    {
        // reset measured angles
        anglesCount = 0;
        angles[0] = 0.f;
        angles[1] = 0.f;
        angles[2] = 0.f;

        rotationDone = true;
    }
    
    // reset steps
    stepVal = stepVal % rotationSteps;

    // lower frequency -> increase motor speed
    // each cycle, until lower bound frequency, decrease current frequency
    if ((crtFreq > freqLow) && pwm_high)
    {
        --crtFreq;
        OCR1A = crtFreq;
    }
}

volatile bool mpuInterrupt = false;     ///< indicates whether MPU interrupt pin has gone high
void mpuInterruptFcn()
{
    mpuInterrupt = true;
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
#ifdef _DEBUG_MSG
    // serial
    Serial.begin(115200);
#endif

    // set driver pins mode
    pinMode(stepPin, OUTPUT); 
    pinMode(dirPin, OUTPUT);

    // set motor direction
    digitalWrite(dirPin, LOW);

    // laser interrupt
    pinMode(laserInterruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(laserInterruptPin), laserInterrupt, FALLING);

    Wire.begin();
    TWBR = 24; // 400kHz i2c clock
    
    mpu.initialize();
    if (mpu.testConnection())
    {
        devStatus = mpu.dmpInitialize();

        mpu.setZGyroOffset(0); // TBD
        mpu.setZAccelOffset(0); // TBD

        if (devStatus == 0)
        {
            mpu.setDMPEnabled(true);

            attachInterrupt(digitalPinToInterrupt(mpuInterruptPin), mpuInterruptFcn, RISING);
            mpuIntStatus = mpu.getIntStatus();

            packetSize = mpu.dmpGetFIFOPacketSize();

            dmpReady = true;
#ifdef _DEBUG_MSG
            Serial.println("MPU Ready!");
#endif
        }
        else
        {
            // TODO: Report problem!
#ifdef _DEBUG_MSG
            Serial.println("devStatus != 0");
#endif
        }
    }
    else
    {
        // TODO: Report problem!
#ifdef _DEBUG_MSG
        Serial.println("!mpu.testConnection()");
#endif
    }

    // setup motor timer
    TCCR1A = 0; // no flags needed here
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC & 64 prescaler
    OCR1A = freqHigh; // start with interrupt @ ~3.5ms
    TIMSK1 |= (1 << OCIE1A);
    sei(); // global interrupts
}

void loop()
{
    // if gyroscope is ready, read it
    if (mpuInterrupt || (fifoCount >= packetSize))
    {
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();

        fifoCount = mpu.getFIFOCount();

         if ((mpuIntStatus & 0x10) || fifoCount == 1024)
         {
#ifdef _DEBUG_MSG
            Serial.println("OVERFLOW");
#endif
            mpu.resetFIFO();
         }
         else if (mpuIntStatus & 0x02)
         {
            while (fifoCount < packetSize)
            {
                fifoCount = mpu.getFIFOCount();
            }

            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            yaw = ypr[0] * RAD2DEG; // cast to int16_t
            yaw = (360 + yaw) % 360; // map to [0; 360)
            
#ifdef _DEBUG_MSG
            //Serial.println(ypr[0] * RAD2DEG);
#endif
         }
    }

    // if enough angles were measured, do triangulation
    if (rotationDone && (anglesCount == 3))
    {
        // compute the order of beacons that were seen
        const byte fb = (ypr[0] < 90.f) ? 0 : (ypr[0] < 180.f) ? 1 : 2;
        const byte sb = (fb + 1) % MAX_BEACONS;
        const byte tb = (fb + 2) % MAX_BEACONS;

        // calculate x and y of the robot
        const float err = triangulation(x, y,
                                        angles[0] * DEG2RAD, angles[1] * DEG2RAD, angles[2] * DEG2RAD,
                                        beaconsLoc[fb].x, beaconsLoc[fb].y,
                                        beaconsLoc[sb].x, beaconsLoc[sb].y,
                                        beaconsLoc[tb].x, beaconsLoc[tb].y);
        
        // send odometry if everything is ok
        if (err > 0.f)
        {
            sendOdometry(x, y, ypr[0]);
#ifdef _DEBUG_MSG
            Serial.print(x);
            Serial.print(",");
            Serial.println(y);
#endif
        }

        rotationDone = false;
    }
}
