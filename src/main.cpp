    #include "I2Cdev.h"
    #include "MPU6050_6Axis_MotionApps20.h"
    MPU6050 mpu;


    #define ENC1A 2
    #define ENC1B 5
    #define ENC2A 3
    #define ENC2B 6


    volatile long enc1 = 0;
    volatile long enc2 = 0;

    #define motorA1 13 
    #define motorA2 12 
    #define motorB1 8
    #define motorB2 7
    #define motorAPWM 11
    #define motorBPWM  10

    #define MinPWM 35

    // Характеристики вашего энкодера (укажите сколько тиков на 1 оборот)
    const float pulsesPerRevolution = 540.0; 

    // Переменные для расчета скорости
    unsigned long lastSpeedCheck = 0;
    const unsigned long speedCheckInterval = 50; // Расчет скорости каждые 50 мс

    long prevEnc1 = 0;
    long prevEnc2 = 0;

    float speedRPM1 = 0.0;
    float speedRPM2 = 0.0;


    struct PID {
        float kp;
        float ki;
        float kd;
        float setpoint;
        float integral;
        float integralMin;
        float integralMax;
        float lastError;
        float outMin;
        float outMax;
        float pTerm;
        float iTerm;
        float dTerm;
        unsigned long lastTime;

        PID(float Kp = 0, float Ki = 0, float Kd = 0, float minOut = -255, float maxOut = 255)
        : kp(Kp), ki(Ki), kd(Kd), setpoint(0), integral(0), integralMin(-1000), integralMax(1000), lastError(0), outMin(minOut), outMax(maxOut), pTerm(0), iTerm(0), dTerm(0), lastTime(0) {}

        void setTunings(float Kp, float Ki, float Kd) {
            kp = Kp;
            ki = Ki;
            kd = Kd;
        }

        void setOutputLimits(float minOut, float maxOut) {
            outMin = minOut;
            outMax = maxOut;
        }

        void setIntegralLimits(float minI, float maxI) {
            integralMin = minI;
            integralMax = maxI;
        }

        void SetSetpoint(float value ){
            setpoint = value;
        }

        float getP() const { return pTerm; }
        float getI() const { return iTerm; }
        float getD() const { return dTerm; }

        float compute(float input, unsigned long now ) {
            float error = setpoint - input;
            if (lastTime == 0) {
                lastTime = now;
                lastError = error;
            }
            float dt = (now - lastTime) / 1000.0f;
            if (dt <= 0) {
                dt = 0.001f;
            }
            integral += error * dt;
            if (integral > integralMax) integral = integralMax;
            if (integral < integralMin) integral = integralMin;
            float derivative = (error - lastError) / dt;
            pTerm = kp * error;
            iTerm = ki * integral;
            dTerm = kd * derivative;
            float output = pTerm + iTerm + dTerm;
            if (output > outMax) output = outMax;
            if (output < outMin) output = outMin;
            lastError = error;
            lastTime = now;
            return output;
        }

        void reset() {
            integral = 0;
            lastError = 0;
            lastTime = 0;
        }
    };




    PID pidA(2.0f, 0.02f, 0.005f, -255, 255);
    PID pidB(2.0f, 0.02f, 0.005f, -255, 255);
    // PID pidBalance(0.8f, 0.02f, 0.02f, -255, 255);




    void setSpeedA(int speed){
        if(speed < 0 ){
            digitalWrite(motorA1 , false);
            digitalWrite(motorA2 , true);
        }
        else{
            digitalWrite(motorA1 , true);
            digitalWrite(motorA2 , false);
        }
    analogWrite(motorAPWM , abs(speed));
    }


    void setSpeedB(int speed){
        if(speed < 0 ){
            digitalWrite(motorB1 , false);
            digitalWrite(motorB2 , true);
        }
        else{
            digitalWrite(motorB1 , true);
            digitalWrite(motorB2 , false);
        }
    analogWrite(motorBPWM , abs(speed));
    }
    void flag1(){
        if(digitalRead(ENC1A) == digitalRead(ENC1B)){
            enc1++;
        }
        else{
            enc1--;
        }
    }

    void flag2(){
        if(digitalRead(ENC2A) == digitalRead(ENC2B)){
            enc2++;
        }
        else{
            enc2--;
        }
    }

    void setup() {
    Serial.begin(115200);
    Wire.begin();
    //Wire.setClock(1000000UL);   // разгоняем шину на максимум
    pinMode(motorAPWM , OUTPUT);
    pinMode(motorBPWM , OUTPUT);
    pinMode(motorA1 , OUTPUT);
    pinMode(motorA2 , OUTPUT);
    pinMode(motorB1 , OUTPUT);
    pinMode(motorB2 , OUTPUT);

    pinMode(ENC1A , INPUT_PULLUP);
    pinMode(ENC1B , INPUT_PULLUP);
    pinMode(ENC2A  , INPUT_PULLUP);
    pinMode(ENC2B , INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC1A) , flag1 , RISING);
    attachInterrupt(digitalPinToInterrupt(ENC2A) , flag2 , RISING);


    // инициализация DMP
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setDMPEnabled(true);
        lastSpeedCheck = millis();
    }



    void loop() {
    unsigned long now = millis();

        if (now - lastSpeedCheck >= speedCheckInterval) {
            
            noInterrupts();
            long currentEnc1 = enc1;
            long currentEnc2 = enc2;
            interrupts();

            long deltaEnc1 = currentEnc1 - prevEnc1;
            long deltaEnc2 = currentEnc2 - prevEnc2;

            prevEnc1 = currentEnc1;
            prevEnc2 = currentEnc2;

            

            float dt = (now - lastSpeedCheck) / 1000.0f;
            lastSpeedCheck = now;

            speedRPM1 = ((float)deltaEnc1 / pulsesPerRevolution) / dt * 60.0f;
            speedRPM2 = ((float)deltaEnc2 / pulsesPerRevolution) / dt * 60.0f;

            float pidAC = pidA.compute(speedRPM1,now);
            float pidBC = pidB.compute(speedRPM2,now); 

            setSpeedA(pidAC);
            setSpeedB(pidBC);

            Serial.print(">");
            Serial.print("speed A:" );
            Serial.print(pidAC );
            Serial.print(",speed B:" );
            Serial.print(pidBC );
            
            


            // Вывод в Serial (удобно смотреть через "Плоттер по прерываниям" / Serial Plotter)
            Serial.print(",RPM1:");
            Serial.print(speedRPM1);
            Serial.print(",");
            Serial.print("RPM2:");
            Serial.println(speedRPM2);
        }



        // 2. ТЕСТОВОЕ УПРАВЛЕНИЕ: отправьте число от -220 до 220 в Монитор Порта
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim();
            
            // Проверяем, что ввели число (учитывая возможный минус)
            if (input.length() != 0 && (isdigit(input[0]) || input[0] == '-')) {
                int targetSpeed = 100;

                pidA.SetSetpoint(targetSpeed);
                pidB.SetSetpoint(targetSpeed);

                Serial.print("-> PWM set to: ");
                Serial.println(targetSpeed);
            }
        }
    }