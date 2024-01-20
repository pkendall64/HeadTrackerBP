#include <Arduino.h>
#include <Wire.h>

#include "ICM42670P.h"

ICM42670P IMU(Wire,0);

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
    // Gyro ODR = 100 Hz and Full Scale Range = 2000 dps
    IMU.startGyro(100, 2000);

    compass.init();
}

void loop()
{
    if(irq_received) {
        irq_received = 0;
        inv_imu_sensor_event_t imu_event;

        // Get last event
        IMU.getDataFromRegisters(&imu_event);

        // Format data for Serial Plotter
        Serial.printf("ax: %7d ", imu_event.accel[0]);
        Serial.printf("ay: %7d ", imu_event.accel[1]);
        Serial.printf("az: %7d ", imu_event.accel[2]);

        Serial.printf("gx: %7d ", imu_event.gyro[0]);
        Serial.printf("gy: %7d ", imu_event.gyro[1]);
        Serial.printf("gz: %7d ", imu_event.gyro[2]);

        Serial.printf("T: %2.2fC\r\n", (float)imu_event.temperature / 128 + 25);
    }
}