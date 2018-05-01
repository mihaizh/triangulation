#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <math.h>
#include <stdint.h>

//#define _DEBUG_MSG

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

template <typename _Ty>
struct point
{
    _Ty x;
    _Ty y;
};

const byte MAX_BEACONS = 3; ///< beacons on scene
const point<float> beaconsLoc[MAX_BEACONS] = { ///< beacons locations
    { 303.3f, 4.0f },
    { 304.0f, 196.f },
    { -3.3f, 98.f }
};

const byte stepPin = 4; ///< pin number for step
const byte dirPin = 5; ///< pin number for direction

const byte laserInterruptPin = 2; ///< pin number for laser interrupt
const byte mpuInterruptPin = 3; ///< pin number for mpu interrupt

int16_t stepVal = 0; ///< current step of motor
const float STEP2DEG = 0.9f; ///< convert from steps to degrees
const float DEG2STEP = 1.f / STEP2DEG; ///< convert from degrees to steps

const int16_t freqHigh = 877; ///< ~3.5ms
const int16_t freqLow = 150; ///< ~0.6ms

const byte MAX_ANGLES = 3; ///< maximum measureable angles
float angles[MAX_ANGLES] = { 0.F, 0.F, 0.F }; ///< measured angles
byte anglesCount = 0; ///< number of measured angles

MPU6050 mpu;

bool dmpReady = false;  ///< set true if DMP init was successful
uint8_t mpuIntStatus;   ///< holds actual interrupt status byte from MPU
uint8_t devStatus;      ///< return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    ///< expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     ///< count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; ///< FIFO storage buffer
float ypr[3];           ///< [yaw, pitch, roll]
float ypr_offset[3];    ///< [yaw, pitch, roll]
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

int16_t yaw = 0;

bool initialized = false;

point<float> location = { 0.f, 0.f };

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

// send robot odometry [x, y, yaw] through Serial
inline void sendOdometry()
{
    byte data[(sizeof(float) * 2) + sizeof(int16_t)];
    if (initialized)
    {
        memcpy(&data[0], &location.x, sizeof(location.x));
        memcpy(&data[sizeof(location.x)], &location.y, sizeof(location.y));
        memcpy(&data[sizeof(location.x) + sizeof(location.y)], &yaw, sizeof(yaw));
    
        Serial.write(data, sizeof(data));
    }
    else
    {
        memcpy(&data[0], &yaw, sizeof(yaw));
        Serial.write(data, sizeof(yaw));
    }
}

bool pwm_high = true;         ///< indicates if pwm should be HIGH or LOW for this half of cycle
int16_t crtFreq = freqHigh;   ///< current frequency (lowers in time -> motor accelerates)
short rotationSteps = 400;    ///< how many steps means a complete rotation
float lastCorrectionYaw = 0.f;///< last yaw angle the correction was done
bool rotationDone = false;    ///< indicates whether a complete rotation has been done

inline int16_t getCorrectionYaw()
{
    int16_t correctionYaw = 0;
    if ((yaw < 90) && (lastCorrectionYaw > 270)) // handle jump from 360 to 0
    {
        correctionYaw = 360 + yaw - lastCorrectionYaw;
    }
    else if ((yaw > 270) && (lastCorrectionYaw < 90)) // handle jump from 0 to 360
    {
        correctionYaw = -(360 - yaw + lastCorrectionYaw);
    }
    else
    {
        correctionYaw = yaw - lastCorrectionYaw; // no jump
    }

    return correctionYaw;
}

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
        const int16_t correctionYaw = getCorrectionYaw();
        
        rotationDone = true;
        rotationSteps = 400 - (correctionYaw * DEG2STEP);
        lastCorrectionYaw = yaw;

        stepVal = 0;
    }

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

void initialInterrupt()
{
    // set offsets
    memcpy(&ypr_offset, &ypr, sizeof(ypr_offset));
    // activate laser interrupt
    attachInterrupt(digitalPinToInterrupt(laserInterruptPin), laserInterrupt, FALLING);
    // activate PWM timer
    TIMSK1 |= (1 << OCIE1A);
    
    initialized = true;
}

void laserInterrupt()
{
    // determine which was the last measured angle
    const byte lastAngleIndex = (anglesCount == 0) ? (MAX_ANGLES - 1) : ((anglesCount < 4) ? (anglesCount - 1) : 0);
    // calculate the current angle
    const float angle = stepVal * STEP2DEG;
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
    Serial.begin(115200);

    // set driver pins mode
    pinMode(stepPin, OUTPUT); 
    pinMode(dirPin, OUTPUT);

    // set motor direction
    digitalWrite(dirPin, LOW);

    // laser interrupt
    pinMode(laserInterruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(laserInterruptPin), initialInterrupt, FALLING);

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

            yaw = (ypr[0] - ypr_offset[0]) * RAD2DEG; // implicit cast to int16_t
            yaw = (360 - yaw) % 360; // map to [0; 360)

            if (!initialized)
            {
                sendOdometry();
            }
         }
    }

    // if enough angles were measured, do triangulation
    if (rotationDone && (anglesCount == 3))
    {
        // calculate x and y of the robot
        const float err = triangulation(location.x, location.y,
                                        angles[0] * DEG2RAD, angles[1] * DEG2RAD, angles[2] * DEG2RAD,
                                        beaconsLoc[0].x, beaconsLoc[0].y,
                                        beaconsLoc[1].x, beaconsLoc[1].y,
                                        beaconsLoc[2].x, beaconsLoc[2].y);
        
        // send odometry if everything is ok
        if (err > -0.99f)
        {
            sendOdometry();
        }
    }

    if (rotationDone)
    {
        // reset measured angles
        anglesCount = 0;
        angles[0] = 0.f;
        angles[1] = 0.f;
        angles[2] = 0.f;
        
        rotationDone = false;
    }
}
