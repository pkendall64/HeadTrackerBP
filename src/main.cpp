#include <Arduino.h>
#include <Wire.h>

#include "ICM42670P.h"
#include "QMC5883LCompass.h"

#include "MadgwickAHRS.h"

ICM42670P IMU(Wire,0);
QMC5883LCompass compass;

#define DEG_TO_RAD 0.017453295199

float aRes;
float gRes;
float mRes;

volatile uint8_t irq_received = 0;

IRAM_ATTR void irq_handler(void) {
  irq_received = 1;
}

void setup()
{
    Serial.begin(115200);
    delay(3000);
    Wire.begin(2, 4); // set master mode

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

    compass.init();
    Serial.println("CALIBRATING. Keep moving your sensor...");
    compass.calibrate();
    mRes = 0.73;

    Serial.println("DONE.");
}

void loop()
{
    if(irq_received) {
        irq_received = 0;
        inv_imu_sensor_event_t imu_event;

        // Get last event
        IMU.getDataFromRegisters(&imu_event);

        Serial.printf("ax: %7d ", imu_event.accel[0]);
        Serial.printf("ay: %7d ", imu_event.accel[1]);
        Serial.printf("az: %7d ", imu_event.accel[2]);

        Serial.printf("gx: %7d ", imu_event.gyro[0]);
        Serial.printf("gy: %7d ", imu_event.gyro[1]);
        Serial.printf("gz: %7d ", imu_event.gyro[2]);

        Serial.printf("T: %2.2fC ", (float)imu_event.temperature / 128 + 25);

        int x, y, z, a;

        // Read compass values
        compass.read();

        // Return XYZ readings
        x = compass.getX();
        y = compass.getY();
        z = compass.getZ();

        Serial.print("X: ");
        Serial.print(x);
        Serial.print(" Y: ");
        Serial.print(y);
        Serial.print(" Z: ");
        Serial.print(z);

        a = compass.getAzimuth();
        Serial.print(" A: ");
        Serial.print(a);

        MadgwickAHRSupdate(
            imu_event.gyro[0] * gRes * DEG_TO_RAD, imu_event.gyro[1] * gRes * DEG_TO_RAD, imu_event.gyro[2] * gRes * DEG_TO_RAD,
            imu_event.accel[0] * aRes, imu_event.accel[1] * aRes, imu_event.accel[2] * aRes,
            x * mRes, y * mRes, z * mRes
        );
        Serial.printf(" R: %8.2f P: %8.2f Y: %8.2f ", getRoll(), getPitch(), getYaw());
        Serial.println();
    }
}