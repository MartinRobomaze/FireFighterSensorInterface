#include <Sensors/IMUSensor.h>

IMUSensor mpu(15);
TaskHandle_t readIMUSensor;

[[noreturn]] void readMPU(void * param) {
    for (;;) {
        mpu.readIMU();
        
    }
}

void setup() {
    Serial.begin(9600);
    mpu.init();
    mpu.initDMP(220, 76, -20, 2008);
    xTaskCreatePinnedToCore(
                    readMPU,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &readIMUSensor,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */
}

void loop() {
    Serial.println(mpu.getYawAngle());
}