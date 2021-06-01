#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include <PlotPlus.h>
#include "Wire.h"


int ForwardPin_L = 3;
int BackwardPin_L = 5;
int ForwardPin_R = 6;
int BackwardPin_R = 9;

#define TO_DEG 57.29577951308232087679815481410517033f
#define TO_DEGSEC 1/131

#define T_OUT 10 // каждый 20 миллисекунд будем проводить вычисления
#define P_OUT 50 // каждый 50 миллисекунд будем выводить данные
#define FK 0.3 // коэффициент комплементарного фильтра

#define Kp 0.5 // коэффициент пропорциональной составляющей 12.5 0 0
#define Ki 0.04 // коэффициент интегральной составляющей
#define Kd -0.4 // коэффициент дифференциальной составляющей
#define Kspd 1  // коэффициент отношения цифрового значения к об/с
#define d 5.5  // высота до датчика MPU6050
#define C 21.3 // длина окружности колес

#define BUFFER_SIZE 100 //из функции calibration

float Xstd = -2;
float Ystd = 11;

MPU6050 mpu;

float angle_ax, angle_ay, angle_az;
float angle_gx, angle_gy, angle_gz; //Скорости поворота
float angle_x, angle_y, angle_z, alfa; //Углы наклона
float angle_ex, angle_ox, angle_ex_past;
float angle_ey, angle_oy, angle_ey_past, angle_spdy;
double spd;
double max_spd = 255;
float P, I, D;
float balancing_zero;
int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
int16_t aclx, acly, aclz, gclx, gcly, gclz;
int dt = 0;
long int t_next, p_next;


// функция, которая не даёт значению выйти за пределы
float clamp(float v, float minv, float maxv){
    if( v>maxv )
        return maxv;
    else if( v<minv )
        return minv;
    return v;
}

void setup() {
  pinMode(ForwardPin_L, OUTPUT);
  pinMode(BackwardPin_L, OUTPUT);
  pinMode(ForwardPin_R, OUTPUT);
  pinMode(BackwardPin_R, OUTPUT);

    // инициализация MPU6050

    Wire.begin();
    Serial.begin(9600);
    Wire.setClock(400000);
    mpu.initialize();
    initDMP();


    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
//    calibration();
//    Serial.println("Calibration success");

    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
    aclx = ax_raw;
    acly = ay_raw;
    aclz = az_raw;

    gclx = gx_raw;
    gcly = gy_raw;
    gclz = gz_raw;
    alfa = 0.75;
    Serial.print(aclx);
    Serial.print("\t");
    Serial.print(acly);
    Serial.print("\t");
    Serial.println(aclz);
    Serial.println(alfa);
}

void loop() {
    long int t = millis();
    long int t_old, dt;
    // каждые T_TO миллисекунд выполняем рассчет угла наклона
    if( t_next < t ){

        t_next = t + T_OUT;

    	getAngles(); // Плучение углов наклона
    	if(angle_y <= -30){
    		stop();
    		return;
    	}
    	if(angle_y >= 30){
    		stop();
    		return;
    	}
	//PID
		dt = millis() - t_old;

		angle_ey = angle_y - Ystd;
		P = Kp * angle_ey;
		I = I + Ki * angle_ey;
//		D = Kd*(angle_ey - angle_ey_past)/dt;
		D = Kd*angle_gy;
		angle_ey_past = angle_ey;
		angle_oy = P+D+I;
//		angle_oy = clamp(angle_oy, -1*max_spd, max_spd);

		t_old = millis();

//        spd = 0;
        if(angle_oy > 0)
        {
        	if(angle_oy <= 0.7)
        		stop();
        	else if (angle_oy <= 5){
				spd = map(abs(angle_oy), 28, 200, 100, 255);
				spd = clamp(spd, -1*max_spd, max_spd);
        	}
        	else spd = 255;
				backL(spd);
				backR(spd);

//				Ystd += angle_oy*0.016;

        }
        else
        {
        	if(angle_oy >= -0.7)
        		stop();
        	else if (angle_oy >= -5){
				spd = map(abs(angle_oy), 28, 200, 100, 255);
				spd = clamp(spd, -1*max_spd, max_spd);
        	}
        	else spd = 255;
				forwardL(spd);
				forwardR(spd);

//				Ystd += angle_oy*0.016;

        }
    }

    t = millis();
    // каждые P_OUT миллисекунд выводим результат в COM порт
    if( p_next < t ){
        plot(angle_y*10, angle_oy*10);
		Serial.print(angle_y);Serial.print("\t");Serial.print(angle_oy);
		Serial.print("\t");Serial.print(Ystd);Serial.print("\n");
    //Driver
        p_next = t + P_OUT;
    }
}

float scaleL = 1.5;
float scaleR = 1;

void forwardL(int spd){
//	Serial.print("forwardL ");Serial.print(spd);Serial.print("\n");
	digitalWrite(BackwardPin_L, LOW);
	analogWrite(ForwardPin_L, spd*scaleL);
}
void backL(int spd){
//	Serial.print("backL ");Serial.print(spd);Serial.print("\n");
	digitalWrite(ForwardPin_L, LOW);
	analogWrite(BackwardPin_L, spd*scaleL);
}
void forwardR(int spd){
//	Serial.print("forwardR ");Serial.print(spd);Serial.print("\n");
	digitalWrite(BackwardPin_R, LOW);
	analogWrite(ForwardPin_R, spd*scaleR);
}
void backR(int spd){
//	Serial.print("backR ");Serial.print(spd);Serial.print("\n");
	digitalWrite(ForwardPin_R, LOW);
	analogWrite(BackwardPin_R, spd*scaleR);
}
void stop(){
	digitalWrite(BackwardPin_L, LOW);
	digitalWrite(ForwardPin_L, LOW);
	digitalWrite(BackwardPin_R, LOW);
	digitalWrite(ForwardPin_R, LOW);
}


// НУЖНЫЕ ПЕРЕМЕННЫЕ
const float toDeg = 180.0 / M_PI;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 gy;
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// инициализация
void initDMP() {
  devStatus = mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
}
// получение углов в angleX, angleY, angleZ
void getAngles() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angle_x = ypr[2] * toDeg;
    angle_y = ypr[1] * toDeg;
    angle_z = ypr[0] * toDeg;
    mpu.dmpGetGyro(&gy, fifoBuffer);
    angle_gx = gy.x * TO_DEGSEC;
    angle_gy = gy.y * TO_DEGSEC;
    angle_gz = gy.z * TO_DEGSEC;
  }
}

// ======= ФУНКЦИЯ КАЛИБРОВКИ =======
void calibration() {
  long offsets[6];
  long offsetsOld[6];
  int16_t mpuGet[6];
  // используем стандартную точность
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  // обнуляем оффсеты
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  delay(10);
  for (byte n = 0; n < 10; n++) {     // 10 итераций калибровки
    for (byte j = 0; j < 6; j++) {    // обнуляем калибровочный массив
      offsets[j] = 0;
    }
    for (byte i = 0; i < 100 + BUFFER_SIZE; i++) { // делаем BUFFER_SIZE измерений для усреднения mpu.getMotion6(&mpuGet[0], &mpuGet[1], &mpuGet[2], &mpuGet[3], &mpuGet[4], &mpuGet[5]); if (i >= 99)                         // пропускаем первые 99 измерений
        for (byte j = 0; j < 6; j++) {
          offsets[j] += (long)mpuGet[j];   // записываем в калибровочный массив
        }
    }
    for (byte i = 0; i < 6; i++) {
      offsets[i] = offsetsOld[i] - ((long)offsets[i] / BUFFER_SIZE); // учитываем предыдущую калибровку
      if (i == 2) offsets[i] += 16384;                               // если ось Z, калибруем в 16384
      offsetsOld[i] = offsets[i];
    }
    // ставим новые оффсеты
    mpu.setXAccelOffset(offsets[0] / 8);
    mpu.setYAccelOffset(offsets[1] / 8);
    mpu.setZAccelOffset(offsets[2] / 8);
    mpu.setXGyroOffset(offsets[3] / 4);
    mpu.setYGyroOffset(offsets[4] / 4);
    mpu.setZGyroOffset(offsets[5] / 4);
    delay(2);
  }
}
