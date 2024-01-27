#include <Arduino.h>
#include <Wire.h>

#include "ICM42670P.h"
#include "QMC5883LCompass.h"

#include "Fusion.h"

ICM42670P IMU(Wire,0);
QMC5883LCompass compass;
FusionAhrs ahrs;

float aRes;
float gRes;
float mRes;

volatile uint8_t irq_received = 0;

void webserver_setup();
void webserver_loop();
void webserver_update(float roll, float pitch, float yaw);

IRAM_ATTR void irq_handler(void) {
  irq_received = 1;
}

// Normalizes any number to an arbitrary range
// by assuming the range wraps around when going below min or above max
float normalize(float value, float start, float end) {
    float width = end - start;         //
    float offsetValue = value - start; // value relative to 0

    return (offsetValue - (floor(offsetValue / width) * width)) + start;
    // + start to reset back to start of original range
}

// Rotate a point (pn) in space in Order X -> Y -> Z
void rotate(float pn[3], const float rot[3]) {
    float out[3];

    // X-axis Rotation
    if (rot[0] != 0) {
        out[0] = pn[0] * 1 + pn[1] * 0 + pn[2] * 0;
        out[1] = pn[0] * 0 + pn[1] * cos(rot[0]) - pn[2] * sin(rot[0]);
        out[2] = pn[0] * 0 + pn[1] * sin(rot[0]) + pn[2] * cos(rot[0]);
        memcpy(pn, out, sizeof(out[0]) * 3);
    }

    // Y-axis Rotation
    if (rot[1] != 0) {
        out[0] = pn[0] * cos(rot[1]) - pn[1] * 0 + pn[2] * sin(rot[1]);
        out[1] = pn[0] * 0 + pn[1] * 1 + pn[2] * 0;
        out[2] = -pn[0] * sin(rot[1]) + pn[1] * 0 + pn[2] * cos(rot[1]);
        memcpy(pn, out, sizeof(out[0]) * 3);
    }

    // Z-axis Rotation
    if (rot[2] != 0) {
        out[0] = pn[0] * cos(rot[2]) - pn[1] * sin(rot[2]) + pn[2] * 0;
        out[1] = pn[0] * sin(rot[2]) + pn[1] * cos(rot[2]) + pn[2] * 0;
        out[2] = pn[0] * 0 + pn[1] * 0 + pn[2] * 1;
        memcpy(pn, out, sizeof(out[0]) * 3);
    }
}

void setup()
{
    Serial.begin(460800);
    delay(3000);
    Wire.begin(2, 4); // set master mode

    // Compass init first
    compass.init();
	compass.setMode(0x01,0x08,0x10,0X00); // continuous, 100Hz, 8G, 512 over sample

    Serial.println("CALIBRATING. Keep moving your sensor...");
    compass.calibrate();
    mRes = 0.73;

    Serial.println("DONE.");

    // Initializing the ICM42607C
    int ret = IMU.begin();
    if (ret != 0) {
        Serial.print("ICM42607C initialization failed: ");
        Serial.println(ret);
        while(1);
    }
    if ((ret = IMU.enableDataInterrupt(9, irq_handler))) {
        Serial.println("Interrupt enable failed: ");
        Serial.println(ret);
        while(1);
    }
    // Accel ODR = 100 Hz and Full Scale Range = 16G
    IMU.startAccel(100, 16);
    aRes = 16.0/32768;
    // Gyro ODR = 100 Hz and Full Scale Range = 2000 dps
    IMU.startGyro(100, 2000);
    gRes = 2000.0/32768;

    FusionAhrsInitialise(&ahrs);
    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
            .accelerationRejection = 10.0f,
            .magneticRejection = 10.0f,
            .recoveryTriggerPeriod = 5 * 100, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    webserver_setup();
}

float orientation[3] = {0.0, 0.0, 0.0};
float xRot[3] = {-90*DEG_TO_RAD, 0.0f, 0.0f};
float yRot[3] = {0.0f, -90*DEG_TO_RAD, 0.0f};

void loop()
{
    static boolean running = true;
    static int counter = 0;
    static float roll, pitch, yaw;
    static float rollHome = 0, pitchHome = 0, yawHome = 0;

    webserver_loop();

    if(irq_received) {
        irq_received = 0;

        inv_imu_sensor_event_t imu_event;
        IMU.getDataFromRegisters(&imu_event);

        FusionVector a;
        a.axis.x =  imu_event.accel[0] * aRes;
        a.axis.y =  imu_event.accel[1] * aRes;
        a.axis.z =  imu_event.accel[2] * aRes;
        rotate(a.array, orientation);

        FusionVector g;
        g.axis.x =  imu_event.gyro[0] * gRes;
        g.axis.y =  imu_event.gyro[1] * gRes;
        g.axis.z =  imu_event.gyro[2] * gRes;
        rotate(g.array, orientation);

        compass.read();

        FusionVector m;
        m.axis.x = compass.getX();
        m.axis.y = compass.getY();
        m.axis.z = compass.getZ();
        rotate(m.array, orientation);

        // Calculate delta time (in seconds) to account for gyroscope sample clock error
        const clock_t timestamp = micros();
        static clock_t previousTimestamp;
        const float deltaTime = (float) (timestamp - previousTimestamp) / (float) 1000000;
        previousTimestamp = timestamp;

        FusionAhrsUpdate(&ahrs, g, a, m, deltaTime);

        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        euler.angle.roll -= rollHome;
        euler.angle.pitch -= pitchHome;
        euler.angle.yaw -= yawHome;

        if (counter++ == 10)
        {
            counter = 0;
            webserver_update(euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
            if (running)
            {
    #ifdef DEBUG_LOG
                Serial.printf("ax: %7d ", imu_event.accel[0]);
                Serial.printf("ay: %7d ", imu_event.accel[1]);
                Serial.printf("az: %7d ", imu_event.accel[2]);

                Serial.printf("gx: %7d ", imu_event.gyro[0]);
                Serial.printf("gy: %7d ", imu_event.gyro[1]);
                Serial.printf("gz: %7d ", imu_event.gyro[2]);

                Serial.printf("T: %2.2fC ", (float)imu_event.temperature / 128 + 25);

                Serial.print("X: ");
                Serial.print(x);
                Serial.print(" Y: ");
                Serial.print(y);
                Serial.print(" Z: ");
                Serial.print(z);

                int a = compass.getAzimuth();
                Serial.print(" A: ");
                Serial.print(a);
    #endif
                Serial.printf("X: %8.2f Y: %8.2f Z: %8.2f", a.axis.x, a.axis.y, a.axis.z);
                Serial.printf(" R: %8.2f P: %8.2f Y: %8.2f", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
                Serial.println();
            }
        }
        if (Serial.available()) {
            String s = Serial.readString();
            if (s == "z") {
                rollHome += euler.angle.roll;
                pitchHome += euler.angle.pitch;
                yawHome += euler.angle.yaw;
                rollHome = normalize(rollHome, -180.0, 180.0);
                pitchHome = normalize(pitchHome, -180.0, 180.0);
                yawHome = normalize(yawHome, -180.0, 180.0);
                return;
            }
            if (running) {
                orientation[0] = 0;
                orientation[1] = 0;
                orientation[2] = 0;
                rollHome = 0;
                pitchHome = 0;
                yawHome = 0;
                running = false;
                FusionAhrsReset(&ahrs);
                Serial.println("Start calibration, press f when device is flat");
            }
            if (s == "r") {
                Serial.println("Start running");
                running = true;
                counter = 0;
            }
            else {
                if (s == "f") {
                    // remember the current angles
                    Serial.println("Now align the box so it is flat and press 'c'");
                    roll = euler.angle.roll;
                    pitch = euler.angle.pitch;
                    yaw = euler.angle.yaw;
                    Serial.printf("rX: %8.2f rY: %8.2f rZ: %8.2f", roll, pitch, yaw);
                    Serial.println();
                }
                else if (s == "c") {
                    orientation[0] = (roll - euler.angle.roll)*DEG_TO_RAD;
                    orientation[1] = (pitch - euler.angle.pitch)*DEG_TO_RAD;
                    orientation[2] = (yaw - euler.angle.yaw)*DEG_TO_RAD;
                    Serial.printf("oX: %8.2f oY: %8.2f oZ: %8.2f", roll - euler.angle.roll, pitch - euler.angle.pitch, yaw - euler.angle.yaw);
                    Serial.println();
                    FusionAhrsReset(&ahrs);
                    running = true;
                    counter = 0;
                }
            }
        }
    }
}