// ---------- БИБЛИОТЕКИ -----------
#include "LedControl.h"
#include "I2Cdev.h"
#include "MPU6050.h"
MPU6050 mpu;
// ---------- БИБЛИОТЕКИ -----------

// --------------------- НАСТРОЙКИ ----------------------

//ПОДКЛЮЧЕНИЕ МАТРИЦЫ
// pin 13 - CLK 
// pin 11 - DIN
// pin 10 - CS
LedControl lc = LedControl(11,13,10,1); 

#define MIN_STEP 30     // минимальный шаг интегрирования (миллисекункды)
#define PIXELSIZE 3     // размер пикселя
#define NUMBER 10       // количество "живых" пикселей
#define BRIGHTNESS 8    // яркость матрицы 0-15
#define G_CONST 9.81    // ускорение свободного падения

// оффсеты для акселерометра
int offsets[6] = { -117, -1171, 2145, 6, 82, 153};

/*
  ПОСМОТРИ, КАК ВЕДУТ СЕБЯ ШАРИКИ НА ДРУГИХ ПЛАНЕТАХ!!!
  Земля     9.81  м/с2
  Солнце    273.1 м/с2
  Луна      1.62  м/с2
  Меркурий  3.68  м/с2
  Венера    8.88  м/с2
  Марс      3.86  м/с2
  Юпитер    23.95 м/с2
  Сатурн    10.44 м/с2
  Уран      8.86  м/с2
  Нептун    11.09 м/с2
*/
// --------------------- НАСТРОЙКИ ----------------------

// --------------------- ДЛЯ РАЗРАБОТЧИКОВ ----------------------

//// СТРУКТУРА ПИКСЕЛЕЙ С НАЧАЛЬНЫМИ УСЛОВИЯМИ ////
struct Pixels{
  float m_cf;      // ТРЕНИЕ
  float bounce;    // ОТСКОК
  int pos_x = 3;   // начальная координата по x
  int pos_y = 3;   // начальная координата по y
  float vel_x = 0; // начальная скорость по x
  float vel_y = 0; // начальная скорость по y
};

Pixels pix[NUMBER];

float mpuPitch;
float mpuRoll;

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

unsigned long integrTimer, loopTimer;
float stepTime;
// --------------------- ДЛЯ РАЗРАБОТЧИКОВ ----------------------

void setup() {
  lc.shutdown(0,false);
  lc.setIntensity(0,BRIGHTNESS);
  lc.clearDisplay(0);
  
  // рандомные значения для коэффициента трения и отскока для пикселей
  for(byte i = 0;i < NUMBER;i++){
    lc.setLed(0,pix[i].pos_x,pix[i].pos_y,true);
    pix[i].m_cf = random(0,30);
    pix[i].bounce = random(60,95); 
  }
  
  Serial.begin(9600);
  mpuSetup();
}

void loop() {
  if (millis() - loopTimer > MIN_STEP) {
    loopTimer = millis();
    find_position();
    lc.clearDisplay(0);
    
    for(byte i = 0;i < NUMBER;i++){
      byte nowDistX = floor(pix[i].pos_x / PIXELSIZE);   // перевести миллиметры в пиксели
      byte nowDistY = floor(pix[i].pos_y / PIXELSIZE);   // перевести миллиметры в пиксели
      lc.setLed(0,nowDistX,nowDistY,true);               // рисуем пиксель
    }
  }
}

////////// НАХОДИМ КООРДИНАТЫ ПИКСЕЛЕЙ //////////
void find_position(){
  mpu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);        // получить ускорения
  mpuPitch = (float)accX / 16384;                                     // 16384 это величина g с акселерометра
  mpuRoll = (float)accY / 16384;

  stepTime = (float)((long)millis() - integrTimer) / 1000;            // расчёт времени шага интегрирования
  integrTimer = millis();
  for(byte i = 0;i < NUMBER;i++){
    int thisAccel_x, thisAccel_y;
    int grav, frict;
  
    ///////////////////// ОСЬ Х /////////////////////
    grav = (float)G_CONST * mpuPitch * 1000;    // сила тяжести
    frict = (float)G_CONST * (1 - mpuPitch) * pix[i].m_cf * 10; // сила трения
    if (pix[i].vel_x > 0) frict = -frict;   // знак силы трения зависит от направления вектора скорости
    if (pix[i].vel_x == 0 && abs(grav) < frict) thisAccel_x = 0;// трение покоя
    else thisAccel_x = (grav + frict);                          // ускорение
  
    /////////////////////// ОСЬ У /////////////////////
    grav = (float)G_CONST * mpuRoll * 1000;
    frict = (float)G_CONST * (1 - mpuRoll) * pix[i].m_cf * 10;
    if (pix[i].vel_y > 0) frict = -frict;
    if (pix[i].vel_y == 0 && abs(grav) < frict) thisAccel_y = 0;
    else thisAccel_y = (grav + frict);
  
    ///////////////////// ИНТЕГРИРУЕМ ///////////////////
    // скорость на данном шаге V = V0 + ax*dt
    pix[i].vel_x += (float)thisAccel_x * stepTime;
    pix[i].vel_y += (float)thisAccel_y * stepTime;
  
    // координата на данном шаге X = X0 + Vx*dt
    pix[i].pos_x += (float)pix[i].vel_x * stepTime;
    pix[i].pos_y += (float)pix[i].vel_y * stepTime;
  
    /////////////////// ПОВЕДЕНИЕ У СТЕНОК /////////////////
    // рассматриваем 4 стенки матрицы
    if (pix[i].pos_x < 0) {     // если пробили край матрицы
      pix[i].pos_x = 0;         // возвращаем на край
      pix[i].vel_x = -pix[i].vel_x * (float)pix[i].bounce / 100;    // скорость принимаем с обратным знаком и * на коэффициент отскока
    }
    if (pix[i].pos_x > 21) {
      pix[i].pos_x = 21;
      pix[i].vel_x = -pix[i].vel_x * (float)pix[i].bounce / 100;
    }
  
    if (pix[i].pos_y < 0) {     // если пробили край матрицы
      pix[i].pos_y = 0;         // возвращаем на край
      pix[i].vel_y = -pix[i].vel_y * (float)pix[i].bounce / 100;    // скорость принимаем с обратным знаком и * на коэффициент отскока
    }
    if (pix[i].pos_y > 21) {
      pix[i].pos_y = 21;
      pix[i].vel_y = -pix[i].vel_y * (float)pix[i].bounce / 100;
    }
  }
}

void mpuSetup() {
  Wire.begin();
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // ставим оффсеты
  mpu.setXAccelOffset(offsets[0]);
  mpu.setYAccelOffset(offsets[1]);
  mpu.setZAccelOffset(offsets[2]);
  mpu.setXGyroOffset(offsets[3]);
  mpu.setYGyroOffset(offsets[4]);
  mpu.setZGyroOffset(offsets[5]);
}
