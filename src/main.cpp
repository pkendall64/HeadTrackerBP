#include <Arduino.h>
#include <Wire.h>

#include "ICM42670P.h"
#include "QMC5883LCompass.h"

#include "MadgwickAHRS.h"

ICM42670P IMU(Wire,0);
QMC5883LCompass compass;

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
    // compass.calibrate();
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

        compass.read();

        int x, y, z;
        x = compass.getX();
        y = compass.getY();
        z = compass.getZ();

        float a[3];
        a[0] =  imu_event.accel[0] * aRes;
        a[1] =  imu_event.accel[1] * aRes;
        a[2] =  imu_event.accel[2] * aRes;
        rotate(a, orientation);

        float g[3];
        g[0] =  imu_event.gyro[0] * gRes * DEG_TO_RAD;
        g[1] =  imu_event.gyro[1] * gRes * DEG_TO_RAD;
        g[2] =  imu_event.gyro[2] * gRes * DEG_TO_RAD;
        rotate(g, orientation);

        MadgwickAHRSupdateIMU(
            g[0], g[1], g[2],
            a[0], a[1], a[2]
            // x * mRes, y * mRes, z * mRes
        );

        if (counter++ == 10 && running)
        {
            counter = 0;
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
            Serial.printf("X: %8.2f Y: %8.2f Z: %8.2f", a[0], a[1], a[2]);
            Serial.printf(" R: %8.2f P: %8.2f Y: %8.2f", (getRoll()-rollHome)*RAD_TO_DEG, (getPitch()-pitchHome)*RAD_TO_DEG, (getYaw()-yawHome)*RAD_TO_DEG);
            Serial.println();
        }
        webserver_update(-(getRoll()-rollHome)*RAD_TO_DEG, (getPitch()-pitchHome)*RAD_TO_DEG, -(getYaw()-yawHome)*RAD_TO_DEG);
        if (Serial.available()) {
            String s = Serial.readString();
            if (s == "z") {
                rollHome = getRoll();
                pitchHome = getPitch();
                yawHome = getYaw();
                rollHome = constrain(rollHome, 0.0, 360.0);
                pitchHome = constrain(pitchHome, 0.0, 360.0);
                yawHome = constrain(yawHome, 0.0, 360.0);
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
                    roll = getRoll();
                    pitch = getPitch();
                    yaw = getYaw();
                    Serial.printf("rX: %8.2f rY: %8.2f rZ: %8.2f", roll*RAD_TO_DEG, pitch*RAD_TO_DEG, yaw*RAD_TO_DEG);
                    Serial.println();
                }
                else if (s == "c") {
                    orientation[0] = roll - getRoll();
                    orientation[1] = pitch - getPitch();
                    orientation[2] = yaw - getYaw();
                    Serial.printf("oX: %8.2f oY: %8.2f oZ: %8.2f", orientation[0]*RAD_TO_DEG, orientation[1]*RAD_TO_DEG, orientation[2]*RAD_TO_DEG);
                    Serial.println();
                    running = true;
                    counter = 0;
                }
            }
        }
    }
}