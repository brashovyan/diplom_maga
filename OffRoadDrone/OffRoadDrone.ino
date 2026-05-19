#include <WiFi.h>
#include <AsyncUDP.h>
#include <SD.h>                   //Включение в проект библиотеки для модулем SD-карты
#include <FS.h>                   //Включение в проект библиотеки для поддержки файловой системы
#include <sqlite3.h>              //Включение в проект библиотеки драйвера SQLite3
#include <SPI.h>                  //Включение в проект библиотеки для управления внутренним модулем SPI          

#define CS_SD_PIN 5              //Пин SD карты

#include <queue>
#include <cstring>

#define CONTROLLER_ID 1  // Обозначение номера контроллера

#include <WebServer.h>
#include <ESPmDNS.h>

WebServer server(80);

#include <Ticker.h>

//Управляющие линии колес
#define PIN_RIGHT_F     13
#define PIN_RIGHT_R     12
#define PIN_LEFT_F      14
#define PIN_LEFT_R      27

//Сигнальные линии энкодера левого колеса
#define PIN_ENC_LEFT_A  33
#define PIN_ENC_LEFT_B  32
//Сигнальные линии энкодера правого колеса
#define PIN_ENC_RIGHT_A  26
#define PIN_ENC_RIGHT_B  25

Ticker timerPID;              //Таймер ПИД-регулятора

// Структура для хранения UDP-пакета
struct QueuedPacket {
    char data[600];  // Буфер приема пакетов UDP
    size_t len;
};

std::queue<QueuedPacket> packetQueue;

//Параметры коммуникационного процессора=============================================================
char ssid[] = "KeyTim";          //Идентификатор WiFi-сети робота (в режиме: "точка доступа" - AP)
char pass[] = "18273645";         //Пароль доступа к WiFi-сети
IPAddress localIP;                // Локальный IP-адрес
AsyncUDP udp;                     //UDP-сервер
IPAddress groupIP(239,1,1,1);     //Прослушиваемый групповой IP-адрес
const uint32_t portUDP = 7856;    //UDP-порт командных сигналов
long simulationId = 0;

sqlite3 *db;          //Указатель на объект БД типа SQLite3
sqlite3_stmt *res;    //Указатель на объект с параметрами или результатами выполнения SQL-запроса
int rc;               //Переменная с кодом результата текущей операции с БД
char *zErrMsg = 0;    //Переменная для получения информации об ошибке SQLite

// Глобальные SQL-запросы (хранятся во flash, не в ОЗУ)
const char sqlCreateVectors[] = \
"CREATE TABLE IF NOT EXISTS Vectors (Id INTEGER PRIMARY KEY AUTOINCREMENT, Simulation_id INTEGER NOT NULL, V REAL NOT NULL, \
A REAL NOT NULL, W REAL NOT NULL, T REAL NOT NULL, Created_at DATETIME DEFAULT CURRENT_TIMESTAMP);";

const char sqlDropVectors[] = "DROP TABLE IF EXISTS Vectors";

const char sqlCreateLogs[] = \
"CREATE TABLE IF NOT EXISTS Logs (Id INTEGER PRIMARY KEY AUTOINCREMENT, Simulation_id INTEGER NOT NULL, Vector_id INTEGER NOT NULL, Need_w REAL NOT NULL, Fact_w REAL NOT NULL, Created_at DATETIME DEFAULT CURRENT_TIMESTAMP);";

const char sqlDropLogs[] = "DROP TABLE IF EXISTS Logs";

//Структура данных для описания вектора
struct vector_t {
  uint32_t Id; // Айди вектора
  uint32_t  Simulation_id; // Айди симуляции
  float V; // Скорость (м/с)
  float A; // Угол (рад)
  float W; // Угловая скорость (рад/с) 
  float T; // Время (с)
};
vector_t actualVector;

volatile bool inMovement = false;      // Флаг движения
uint32_t currentVectorId = 0;          // ID текущего вектора
uint32_t previousVectorId = 0;         // ID предыдущего вектора (для логов)
float lastNeedW = 0;                   // Последняя заданная мощность двигателя (или скорость)
float lastFactW = 0;                   // Последняя фактическая мощность двигателя (или скорость)

// Параметры вектора для воспроизведения
float currentV, currentA, currentW, currentT;
float vectorStartTime = 0;             // Время начала текущего вектора
float vectorElapsedTime = 0;           // Прошедшее время текущего вектора
uint32_t lastMovementTime = 0;        // Время последнего обновления движения
const uint32_t MOVEMENT_INTERVAL_MS = 10;  // 10 мс = 100 Гц
//===================================================================================================

//Класс управления мотором постоянного тока (JGB37-520)
class Motor{
  public:
    Motor(uint8_t pin_Forward, uint8_t pin_Reverse, uint8_t pin_EncA, uint8_t pin_EncB, bool isRightMotor);
    void setEncoderISR(void (*ISR_EncA)(void), void (*ISR_EncB)(void));
    void ISR_EncA();
    void ISR_EncB();
    void setPower(int16_t power);
    double getOmega();
    void setOmega(double omega);
    double getAngle();
    void ControlOmega();
    void setPosition(double pos);
    void Action(double dtime, double startPos, double omega);
  private:
    bool isRight;
    uint8_t lineForward;
    uint8_t lineReverse;
    uint8_t lineEncA;
    uint8_t lineEncB;
    uint32_t timeA, timeB;
    int32_t dt;
    int32_t countTickEnc;
    double stepAngle;
    double desiredOmega;
    uint32_t t0;
    double angle0, prevAngle;
    double Kp_omega, Ki_omega, Kp_angle, Kd_angle;
};

Motor::Motor(uint8_t pin_Forward, uint8_t pin_Reverse, uint8_t pin_EncA, uint8_t pin_EncB, bool isRightMotor){
  //Настройка и инициализацияя линий управления мотором
  lineForward=pin_Forward;
  lineReverse=pin_Reverse;
  pinMode(lineForward,OUTPUT);
  pinMode(lineReverse,OUTPUT);
  digitalWrite(lineForward, LOW);
  digitalWrite(lineReverse, LOW);
  //Настройка и инициализация линий контроля квадратурного энкодера
  lineEncA=pin_EncA;
  lineEncB=pin_EncB;
  pinMode(lineEncA,INPUT_PULLUP);
  pinMode(lineEncB,INPUT_PULLUP);
  //Инициализация параметров мотора
  isRight=isRightMotor;     //Признак правого мотора: для правых (перевернутых) моторов должен меняться знак желаемой скорости
  dt=0x7fffffff;            //Временной интервал между прерываниями энкодера: инициализация максимально возможным значением
  timeA=timeB=micros();     //Время крайнего прерывания на каналах (А и В) энкодера
  countTickEnc=0;           //Количество шагов (stepAngle) в повороте колеса
  stepAngle=PI/1850;        //Шаг угла поворота колеса между прерываниями на одном канале квадратурного энкодера
  Kp_omega=200;             //Коэффициент пропорционального регулятора угловой скорости
  Ki_omega=1000;            //Коэффициент интегрального регулятора угловой скорости
  Kp_angle=4000;            //Коэффициент пропорционального регулятора угла
  Kd_angle=1e5;             //Коэффициент дифференциального регулятора угла
  desiredOmega=0;           //Желаемое значение угловой скорости колеса в радианах
  t0=timeA;                 //Время в момент изменения желаемой скорости
  angle0=prevAngle=0;       //Угол поворота оси в момент измененияя желаемой скорости                 
}

void Motor::ISR_EncA(){
  long t=micros();
  unsigned char levelA = digitalRead(lineEncA);
  unsigned char levelB = digitalRead(lineEncB);
  dt=timeA-t;
  timeA=t;
  if(levelA==!isRight){
    if(levelB==HIGH) dt=-dt;
  }else{
    if(levelB==LOW) dt=-dt;
  }
  if(dt>0)
    countTickEnc++;
  else
    countTickEnc--;
  //Serial.println(countTickEnc);
}

void Motor::ISR_EncB(){
  long t=micros();
  unsigned char levelA = digitalRead(lineEncA);
  unsigned char levelB = digitalRead(lineEncB);
  dt=timeB-t;
  timeB=t;
  if(levelB==!isRight){
    if(levelA==LOW) dt=-dt;
  }else{
    if(levelA==HIGH) dt=-dt;
  }
}

void Motor::setEncoderISR(void (*ISR_EncA)(void), void (*ISR_EncB)(void)){
  attachInterrupt(digitalPinToInterrupt(lineEncA),ISR_EncA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(lineEncB),ISR_EncB,CHANGE);
}

double Motor::getOmega(){
  uint32_t t=micros();
  uint32_t dt3=3*abs(dt);
  if((t-timeA>dt3)||(t-timeB>dt3)) 
    return 0;
  else
    return stepAngle*1e6/dt;
}

void Motor::setOmega(double omega){
    t0=micros();
    angle0=countTickEnc*stepAngle;
    desiredOmega=omega;
}

void Motor::setPosition(double pos){
  countTickEnc = uint32_t(pos / stepAngle);
  setOmega(0);
}

void Motor::Action(double dtime, double startPos, double omega){
    t0=micros();
    angle0=startPos;
    desiredOmega=omega;
    delayMicroseconds(dtime);
}

void Motor::setPower(int16_t power){
  if(power>255) power=255;
  if(power<-255) power=-255;
  if(power>0){
    analogWrite(lineReverse,0);
    analogWrite(lineForward,power);
  }else{
    analogWrite(lineForward,0);
    analogWrite(lineReverse,abs(power));
  }
}

double Motor::getAngle(){
  return countTickEnc*stepAngle;
}

void Motor::ControlOmega(){
  double t=(micros()-t0)/1e6;
  double angle = getAngle();
  if (desiredOmega!=0) {
    setPower(Kp_omega*(desiredOmega-getOmega())+Ki_omega*(angle0+desiredOmega*t-angle)); //ПИ-регулятор угловой скорости
  } else {
    setPower(Kp_angle*(angle0-angle)+Kd_angle*(prevAngle-angle));   //ПД-регулятор углового положения 
  }
  prevAngle=angle;
}
//================================================================================================================


//Создание объекта управления задним левым колесом и глобальных обработчиков прерываний его энкодера==============
Motor LeftMotor(PIN_LEFT_F,PIN_LEFT_R,PIN_ENC_LEFT_A,PIN_ENC_LEFT_B, false);
//Глобальная процедура - обработчик прерывания для канала А энкодера заднего левого колеса
void ISR_LeftEncA(){
  LeftMotor.ISR_EncA();
}
//Глобальная процедура - обработчик прерывания для канала B энкодера заднего левого колеса
void ISR_LeftEncB(){
  LeftMotor.ISR_EncB();
}
//================================================================================================================

//Создание объекта управления передним правым колесом и глобальных обработчиков прерываний его энкодера============
Motor RightMotor(PIN_RIGHT_F,PIN_RIGHT_R,PIN_ENC_RIGHT_A,PIN_ENC_RIGHT_B, false);
//Глобальная процедура - обработчик прерывания для канала А энкодера переднего правого колеса
void ISR_RightEncA(){
  RightMotor.ISR_EncA();
}
//Глобальная процедура - обработчик прерывания для канала B энкодера переднего правого колеса
void ISR_RightEncB(){
  RightMotor.ISR_EncB();
}
//==================================================================================================================


//Глобальная процедура таймера для обеспечения работы PID-регуляторов всех колес====================================
void timer_ISR(){
  LeftMotor.ControlOmega();
  RightMotor.ControlOmega();
}
//===================================================================================================================

// Функции управления колесами (заглушки - реализовать позже)
void setWheelSpeeds(float V, float A, float W) {
    // TODO: рассчитать скорости для 12 колес и отправить на драйверы
    // Пока просто выводим в Serial

    // double A1 = 0;
    // double Vx1 = V * cos(A-A1);
    // double Vy1= V * sin(A-A1);

    // -Vx1-Vy1-W
    // -Vx1+Vy1-W
    // -Vx1-Vy1+W
    // -Vx1+Vy1+W

    double A2 = 3.14159265 / 3;
    double Vx2 = V * cos(A-A2);
    double Vy2 = V * sin(A-A2);

    // Левое колесо (5)
    lastNeedW = -Vx2-Vy2-W;
    Serial.println(lastNeedW);
    LeftMotor.setOmega(lastNeedW);

    // Правое колесо (6)
    lastNeedW = -Vx2+Vy2-W;
    //Serial.println(lastNeedW);
    RightMotor.setOmega(lastNeedW);


    // -Vx2-Vy2-W
    // -Vx2+Vy2-W
    // -Vx2-Vy2+W
    // -Vx2+Vy2+W

    // double A3 = 2 * 3.14 / 3;
    // double Vx3 = V * cos(A-A3);
    // double Vy3 = V * sin(A-A3);

    // -Vx3-Vy3-W
    // -Vx3+Vy3-W
    // -Vx3-Vy3+W
    // -Vx3+Vy3+W

    //BackLeftMotor.setOmega(lbW); 



    //Serial.printf("setWheelSpeeds: V=%.3f, A=%.3f, W=%.3f\n", V, A, W);
}

void stopAllWheels() {
    // TODO: остановить все моторы
    LeftMotor.setOmega(0);
    RightMotor.setOmega(0);
    Serial.println("stopAllWheels: Моторы остановлены");
}

float getCurrentWheelSpeed() {
    // TODO: получить фактическую угловую скорость с энкодеров
    // Пока возвращаем 0
    return 0.0;
}

//Процедура инициализации программно-аппаратных средств управления модулем (внешним) памяти стенда
bool InitMemory(){
  SPI.begin();                                            //Инициализация SPI для взаимодействия с модулем SD-карты
  if (!SD.begin(CS_SD_PIN)){                              //Инициализация модуля управления SD-картой
    Serial.printf("Ошибка: SD-модуль не обнаружен\n");    //Вывод диагностического сообщения, если SD-модуль не обнаружен
    return false;                                         //Выход из процедура с признаком ошибки инициализации SD-модуля
  } else {
    Serial.printf("SD-модуль обнаружен\n");               //Вывод диагностического сообщения при успешной инициализации SD-модуля
    uint8_t cardType = SD.cardType();                     //Запрос типа SD-карты
    if(cardType==CARD_NONE){                              //Анализ типа SD-карты
      Serial.println("Ошибка: карта памяти отсутствует"); //Вывод диагностического сообщения об отсутствие SD-карты в модуле flash-памяти
      return false;                                       //Выход из процедура с признаком ошибки инициализации SD-карты
    } else {
      //Вывод типа SD-карты
      Serial.printf("Карта памяти:\n\tтип - ");
      switch(cardType){
        case CARD_MMC:
          Serial.printf("MMC\n");
          break;
        case CARD_SD:
          Serial.printf("SDSC\n");
          break;
        case CARD_SDHC:
          Serial.printf("SDHC\n");
          break;
        default:
          Serial.print("неизвестен\n");
        }
      uint64_t cardSize = SD.cardSize() / (1024 * 1024);  //Расчет объема памяти SD-карты в мегабайтах
      Serial.printf("\tРазмер - %llu MB\n", cardSize);    //Вывод диагностического сообщения об объеме памяти SD-карты в мегабайтах
    }
  }
  return true;                                            //Выход из процедура с признаком успешной инициализации SD-карты
}

// Загрузка первого вектора для заданной симуляции
bool loadFirstVector() {
    char sql[200];
    snprintf(sql, sizeof(sql), 
            "SELECT Id, V, A, W, T FROM Vectors WHERE Simulation_id = %ld ORDER BY Id LIMIT 1", 
            simulationId);
    
    rc = sqlite3_prepare_v2(db, sql, -1, &res, NULL);
    if (rc != SQLITE_OK) {
      // Не смогли прочитать вектор с БД
      return false;
    }
    
    if (sqlite3_step(res) == SQLITE_ROW) {
        currentVectorId = sqlite3_column_int(res, 0);
        currentV = sqlite3_column_double(res, 1);
        currentA = sqlite3_column_double(res, 2);
        currentW = sqlite3_column_double(res, 3);
        currentT = sqlite3_column_double(res, 4);
        
        vectorStartTime = millis() / 1000.0;  // текущее время в секундах
        vectorElapsedTime = 0;
        
        Serial.printf("Загружен вектор %d: V=%.3f, A=%.3f, W=%.3f, T=%.3f\n", 
                      currentVectorId, currentV, currentA, currentW, currentT);
        
        sqlite3_finalize(res);
        return true;
    }
    
    sqlite3_finalize(res);
    return false;
}

// Загрузка следующего вектора по ID (без String)
bool loadNextVector() {
    char sql[160];
    snprintf(sql, sizeof(sql), 
             "SELECT Id, V, A, W, T FROM Vectors WHERE Id > %u AND Simulation_id = %ld ORDER BY Id LIMIT 1",
             currentVectorId, simulationId);
    
    rc = sqlite3_prepare_v2(db, sql, -1, &res, NULL);
    if (rc != SQLITE_OK) {
      // Не смогли прочитать вектор с БД
      return false;
    }
    
    if (sqlite3_step(res) == SQLITE_ROW) {
        previousVectorId = currentVectorId;
        
        currentVectorId = sqlite3_column_int(res, 0);
        currentV = sqlite3_column_double(res, 1);
        currentA = sqlite3_column_double(res, 2);
        currentW = sqlite3_column_double(res, 3);
        currentT = sqlite3_column_double(res, 4);
        
        vectorStartTime = millis() / 1000.0;
        vectorElapsedTime = 0;
        
        sqlite3_finalize(res);
        return true;
    }
    
    sqlite3_finalize(res);
    return false;
}

// Логирование
bool logMovement(uint32_t vectorId, float needW, float factW) {
    char sql[256];
    snprintf(sql, sizeof(sql), 
             "INSERT INTO Logs (Simulation_id, Vector_id, Need_w, Fact_w) VALUES (%ld, %u, %.6f, %.6f);",
             simulationId, vectorId, needW, factW);
    
    rc = sqlite3_exec(db, sql, NULL, NULL, &zErrMsg);
    if (rc != SQLITE_OK) {
        Serial.printf("Ошибка лога: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
        return false;
    }
    return true;
}

void processMovement() {
    if (!inMovement) return;
    
    float currentTime = millis() / 1000.0;
    vectorElapsedTime = currentTime - vectorStartTime;
    
    if (vectorElapsedTime >= currentT) {
        // Логируем текущий вектор перед его завершением
        if (currentVectorId != 0) {
            lastFactW = getCurrentWheelSpeed();  // Получаем реальную скорость
            logMovement(currentVectorId, lastNeedW, lastFactW);
        }

        if (!loadNextVector()) {
            stopMovement();
            return;
        }
        vectorStartTime = currentTime;
        vectorElapsedTime = 0;
        
        // Применяем новый вектор
        setWheelSpeeds(currentV, currentA, currentW);
    }
}

//Процедура выполнения SQL-запроса к базе данных стенда на SD-карте
int db_exec(sqlite3 *db, const char *sql) {
   long start = micros();                                                   //Время начала выполнения запроса
   int rc = sqlite3_exec(db, sql, NULL, NULL, &zErrMsg);                    //Обращение с запросом к драйверу SQLite
   if (rc != SQLITE_OK) {
        Serial.printf("Ошибка создания таблицы Logs: %d\n", rc);
    } else {
        Serial.println("Таблица Logs создана успешно");
    }
   sqlite3_free(zErrMsg);                                                   //Очистка сведений об ошибке в драйвере SQLite
   return rc;                                                               //Выдача кода-результата запроса вызывающей функции
}

//Процедура инициализации WiFi-сети
void Init_Wifi(){
  Serial.println("\nИнициация WiFi-модуля на ESP32");
  
  WiFi.begin(ssid, pass);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 200) { // ~10 секунд
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("Ошибка: статус подключения = %d\n", WiFi.status());
    Serial.println("Возможные причины:");
    Serial.println(" - Неверный пароль (особенно если есть ! @ # $ % ^ & *)");
    Serial.println(" - Скрытая сеть (надо добавить WiFi.setMinSecurity)");
    Serial.println(" - Роутер не в смешанном режиме b/g/n");
    return;
  }
  
  localIP = WiFi.localIP();
  Serial.print("\tЛокальный IP-адрес: ");
  Serial.println(localIP);
  Serial.printf("\tГрупповой IP-адрес: %s\n", groupIP.toString().c_str());
}

//Добавить новый вектор в БД
bool addVector(vector_t* vector){
  //Возвращаемое значение: false - возникновения ошибки в ходе выполнения процедуры; true - процедура выполнена успешно
  long start = micros();  //Время начала работы процедуры
  char sql[256];
  snprintf(sql, sizeof(sql), 
            "INSERT INTO Vectors (Simulation_id, V, A, W, T) VALUES (%u, %.6f, %.6f, %.6f, %.6f);",
            vector->Simulation_id, vector->V, vector->A, vector->W, vector->T);

  rc = sqlite3_prepare_v2(db, sql, strlen(sql), &res, NULL);  //Построение плана выполнения (компиляция) запроса с параметрами
  if (rc != SQLITE_OK) {  //Если возникла ошибка при компиляции запроса
    sqlite3_close(db);                                                        //Закрыть базу данных
    return false;                                                            //Завершить процедуру с кодом ошибки
  }
  //Построение списка параметров запроса в соответствие с номером поля в запросе INSERT и его типом в БД
  sqlite3_bind_int(res, 1, vector->Simulation_id);        //Помещение в список параметров запроса значения для поля PowerLED
  sqlite3_bind_double(res, 2, vector->V);          //Помещение в список параметров запроса значения для поля RegPeriod
  sqlite3_bind_double(res, 3, vector->A);         //Помещение в список параметров запроса значения для поля Exposition
  sqlite3_bind_double(res, 4, vector->W);               //Помещение в список параметров запроса значения для поля xPos
  sqlite3_bind_double(res, 5, vector->T);               //Помещение в список параметров запроса значения для поля yPos
  if (sqlite3_step(res) != SQLITE_DONE) {             //Выполнение параметризованного запроса и в случае ошибки
    sqlite3_close(db);                                                                          //Закрыть БД
    return false;                                                                              //Завершить процедуру с кодом ошибки
  }
  sqlite3_clear_bindings(res);  //Очистить список параметров в объекте параметрического запроса
  rc = sqlite3_reset(res);      //Сбросить объект параметрического запроса - подготовка к новому запросу
  if (rc != SQLITE_OK) {        //В случае ошибки предыдущей операции
    sqlite3_close(db);          //Закрыть БД
    return false;               //Завершить процедуру с кодом ошибки
  }
  sqlite3_finalize(res);        //Очистить память выделенную под объект управления параметрическим запросом (удалить план выполнения запроса)
  
  return true;     //Вернуть в качестве результата признак успешности добавления новой записи о последовательности в БД
}

// Прием UDP пакетов 
void parsePacket(char* data, int len) {
    if (len == 0) return;
    
    // Копируем данные во временный буфер, так как strtok его изменяет
    char buffer[len + 1];
    memcpy(buffer, data, len);
    buffer[len] = '\0';
    
    // Проверяем, команда ли это
    if (strncmp(buffer, "command", 7) == 0) {
        // Формат: "command;1;" или "command;0;"
        char* semicolon = strchr(buffer, ';');
        if (semicolon == NULL) return;
        
        char* cmdStart = semicolon + 1;
        int command = atoi(cmdStart);
        
        if (command == 1) {
            startMovement();
        } else if (command == 0) {
            stopMovement();
        }
        return;
    }
    
    // Иначе это пакет с векторами
    // Формат: ID;V1;A1;W1;T1;V2;A2;W2;T2;...
    
    char* ptr = buffer;
    char* end = buffer + len;
    
    // 1. Парсим ID симуляции (первое число до ';')
    long id = strtol(ptr, &ptr, 10);
    if (ptr == NULL || *ptr != ';') {
        Serial.println("Ошибка: неверный формат (ID)");
        return;
    }
    ptr++; // пропускаем ';'
    simulationId = id;
    
    // 2. Парсим все вектора до конца строки
    int vectorsCount = 0;
    
    while (ptr < end && *ptr != '\0') {
        // Парсим V
        char* next;
        float V = strtof(ptr, &next);
        if (next == ptr || *next != ';') break;
        ptr = next + 1;
        
        // Парсим A
        float A = strtof(ptr, &next);
        if (next == ptr || *next != ';') break;
        ptr = next + 1;
        
        // Парсим W
        float W = strtof(ptr, &next);
        if (next == ptr || *next != ';') break;
        ptr = next + 1;
        
        // Парсим T
        float T = strtof(ptr, &next);
        if (next == ptr) break;
        ptr = next;
        if (*ptr == ';') ptr++; // пропускаем ';' если есть
        
        // Сохраняем вектор
        actualVector.Simulation_id = simulationId;
        actualVector.V = V;
        actualVector.A = A;
        actualVector.W = W;
        actualVector.T = T;
        
        if (!addVector(&actualVector)) {
            Serial.println("Ошибка сохранения вектора");
            return;
        }
        vectorsCount++;
    }
    
    Serial.printf("Сохранено %d векторов для симуляции %ld\n", vectorsCount, simulationId);
}

// Начать движение по команде из CoppeliaSim
void startMovement() {
    if (inMovement) {
        Serial.println("Движение уже выполняется");
        return;
    }
    
    if (simulationId == 0) {
        Serial.println("Ошибка: нет сохранённой траектории (simulationId=0)");
        return;
    }
    
    currentVectorId = 0;
    previousVectorId = 0;
    lastNeedW = 0;
    lastFactW = 0;
    
    if (!loadFirstVector()) {
        Serial.println("Ошибка: не найдены вектора для симуляции");
        return;
    }
    
    setWheelSpeeds(currentV, currentA, currentW);
    Serial.printf("Старт: V=%.3f, A=%.3f, W=%.3f, T=%.3f\n", 
                  currentV, currentA, currentW, currentT);
    
    inMovement = true;
    lastMovementTime = millis();
    vectorStartTime = millis() / 1000.0;
    vectorElapsedTime = 0;
}

// Закончить движение по команде из CoppeliaSim
void stopMovement() {
    if (!inMovement) return;
    
    inMovement = false;
    
    // Останавливаем колеса
    stopAllWheels();
    
    // Логируем последний вектор, если есть
    if (currentVectorId != 0) {
        lastFactW = getCurrentWheelSpeed();
        logMovement(currentVectorId, lastNeedW, lastFactW);
    }
    
    // Сбрасываем указатели
    currentVectorId = 0;
    previousVectorId = 0;
    lastNeedW = 0;
    lastFactW = 0;
    
    Serial.println("Движение остановлено");
}

void setup() {
    Serial.begin(115200);

    // Инициализация памяти и БД
    if(InitMemory() == false) return;
    
    sqlite3_initialize();

    rc = sqlite3_open("/sd/offRoadDrone.db", &db);
    if (rc) {
        Serial.printf("Ошибка: невозможно открыть базу данных\n");
        return;
    } else {
        db_exec(db, sqlCreateVectors);
        db_exec(db, sqlCreateLogs);
        Serial.println("База данных готова к работе");
        Serial.flush();
    }

    // Проверка: сколько векторов в БД
    // char sql[100];
    // snprintf(sql, sizeof(sql), "SELECT COUNT(*) FROM Vectors;");
    // rc = sqlite3_prepare_v2(db, sql, -1, &res, NULL);
    // if (rc == SQLITE_OK && sqlite3_step(res) == SQLITE_ROW) {
    //     int count = sqlite3_column_int(res, 0);
    //     Serial.printf("В базе данных %d векторов\n", count);
    // }
    // sqlite3_finalize(res);

    Init_Wifi();

  // Подписываемся на мультикаст
  if (udp.listenMulticast(groupIP, portUDP, TCPIP_ADAPTER_IF_STA)) {
        udp.onPacket([](AsyncUDPPacket packet) {
        if (packet.isMulticast()) {
            // Не обрабатываем пакет сразу, а кладём в очередь
            if (packetQueue.size() < 20) {  // Защита от переполнения
                QueuedPacket p;
                p.len = packet.length();
                if (p.len < sizeof(p.data)) {
                    memcpy(p.data, packet.data(), p.len);
                    packetQueue.push(p);
                } else {
                    Serial.println("Packet too large for queue buffer!");
                }
            } else {
                Serial.println("Packet queue full, dropping packet!");
            }
        } else {
            Serial.println("Received non-multicast packet, ignoring.");
        }
    });
  } else {
      Serial.println("Failed to start UDP Multicast listener. Check your network configuration.");
  }

  // Запускаем веб сервер для логов
  if (MDNS.begin(String("controller") + String(CONTROLLER_ID))) {
      Serial.println("mDNS started: controller" + String(CONTROLLER_ID) + ".local");
      MDNS.addService("http", "tcp", 80);
  }
  server.on("/", handleRoot);
  server.on("/logs", handleLogs);
  server.on("/logs/csv", handleLogsCSV);
  server.on("/command", handleCommand);
  server.begin();
    
  Serial.println("Готов к приему траекторий");

  //LeftMotor.setEncoderISR(ISR_LeftEncA,ISR_LeftEncB);
  //Назначение энкодеру заднего правого колеса глобальных callback-функций
  //RightMotor.setEncoderISR(ISR_RightEncA,ISR_RightEncB);
  //Запуск таймера с периодом 1 мс для регулярного контроля скорости колес 
  timerPID.attach(0.01,timer_ISR);
  //LeftMotor.setOmega(1); 
  //RightMotor.setOmega(1);
}

void setup1() {

}

// Обрабатываем пакет от UDP, который в очереди
void processPacketQueue() {
    if (packetQueue.empty()) return;
    QueuedPacket p = packetQueue.front();
    packetQueue.pop();
    parsePacket(p.data, p.len);
}

void loop() {
    // Обрабатываем входящие UDP-пакеты из очереди
    processPacketQueue();
    
    // Обработка движения с фиксированной частотой
    if (inMovement) {
        uint32_t currentMs = millis();
        if (currentMs - lastMovementTime >= MOVEMENT_INTERVAL_MS) {
            lastMovementTime = currentMs;
            processMovement();
        }
    }

    // Слушаем обращения к веб серверу логов
    server.handleClient();
    
    delay(0.01);  // Отдаём процессорное время
}

void loop1() {

}

// Веб сайт для логов и управления роботом 

// Функция получения количества записей в логах
int getLogsCount() {
    char sql[100];
    snprintf(sql, sizeof(sql), "SELECT COUNT(*) FROM Logs WHERE Simulation_id = %ld;", simulationId);
    int count = 0;
    rc = sqlite3_prepare_v2(db, sql, -1, &res, NULL);
    if (rc == SQLITE_OK && sqlite3_step(res) == SQLITE_ROW) {
        count = sqlite3_column_int(res, 0);
    }
    sqlite3_finalize(res);
    return count;
}

// HTML главной страницы
void handleRoot() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset='UTF-8'>";
    html += "<title>Robot Controller " + String(CONTROLLER_ID) + "</title>";
    html += "<style>";
    html += "body{font-family:Arial;margin:20px}";
    html += "table{border-collapse:collapse;width:100%}";
    html += "th,td{border:1px solid #ddd;padding:8px;text-align:left}";
    html += "tr:nth-child(even){background-color:#f2f2f2}";
    html += ".success{color:green}.error{color:red}";
    html += "</style>";
    html += "</head><body>";
    
    html += "<h1>Robot Controller #" + String(CONTROLLER_ID) + "</h1>";
    html += "<p>Simulation ID: <strong>" + String(simulationId) + "</strong></p>";
    html += "<p>Logs: <strong>" + String(getLogsCount()) + "</strong> records</p>";
    
    // Кнопки управления
    html += "<button onclick='sendCommand(1)'>▶ START</button> ";
    html += "<button onclick='sendCommand(0)'>⏹ STOP</button>";
    html += "<button onclick='location.href=\"/logs\"'>📊 View Logs</button>";
    html += "<button onclick='location.href=\"/logs/csv\"'>📥 Export CSV</button>";
    
    // JavaScript для отправки команд
    html += "<script>";
    html += "function sendCommand(cmd){";
    html += "fetch('/command?cmd='+cmd).then(r=>r.text()).then(console.log);";
    html += "}</script>";
    
    html += "</body></html>";
    server.send(200, "text/html", html);
}

// Страница с логами
void handleLogs() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset='UTF-8'>";
    html += "<title>Logs - Controller " + String(CONTROLLER_ID) + "</title>";
    html += "<style>body{font-family:Arial;margin:20px}";
    html += "table{border-collapse:collapse;width:100%}";
    html += "th,td{border:1px solid #ddd;padding:8px;text-align:left}";
    html += "</style></head><body>";
    
    html += "<h1>Movement Logs (Controller #" + String(CONTROLLER_ID) + ")</h1>";
    html += "<a href='/'>← Back</a><br><br>";
    html += "<td><tr><th>ID</th><th>Vector ID</th><th>Need W</th><th>Fact W</th><th>Time</th></tr>";
    
    char sql[200];
    snprintf(sql, sizeof(sql), 
             "SELECT Id, Vector_id, Need_w, Fact_w, Created_at FROM Logs WHERE Simulation_id = %ld ORDER BY Id DESC LIMIT 500;",
             simulationId);
    
    rc = sqlite3_prepare_v2(db, sql, -1, &res, NULL);
    if (rc == SQLITE_OK) {
        while (sqlite3_step(res) == SQLITE_ROW) {
            html += "<tr>";
            html += "<td>" + String(sqlite3_column_int(res, 0)) + "</td>";
            html += "<td>" + String(sqlite3_column_int(res, 1)) + "</td>";
            html += "<td>" + String(sqlite3_column_double(res, 2)) + "</td>";
            html += "<td>" + String(sqlite3_column_double(res, 3)) + "</td>";
            html += "<td>" + String((const char*)sqlite3_column_text(res, 4)) + "</td>";
            html += "</tr>";
        }
        sqlite3_finalize(res);
    }
    
    html += "</table></body></html>";
    server.send(200, "text/html", html);
}

// Экспорт в CSV
void handleLogsCSV() {
    String csv = "Id,VectorId,NeedW,FactW,Time\n";
    
    char sql[200];
    snprintf(sql, sizeof(sql), 
             "SELECT Id, Vector_id, Need_w, Fact_w, Created_at FROM Logs WHERE Simulation_id = %ld ORDER BY Id;",
             simulationId);
    
    rc = sqlite3_prepare_v2(db, sql, -1, &res, NULL);
    if (rc == SQLITE_OK) {
        while (sqlite3_step(res) == SQLITE_ROW) {
            csv += String(sqlite3_column_int(res, 0)) + ",";
            csv += String(sqlite3_column_int(res, 1)) + ",";
            csv += String(sqlite3_column_double(res, 2)) + ",";
            csv += String(sqlite3_column_double(res, 3)) + ",";
            csv += String((const char*)sqlite3_column_text(res, 4)) + "\n";
        }
        sqlite3_finalize(res);
    }
    
    server.send(200, "text/csv", csv);
}

// API для команд
void handleCommand() {
    if (server.hasArg("cmd")) {
        int cmd = server.arg("cmd").toInt();
        if (cmd == 1) {
            startMovement();
            server.send(200, "text/plain", "START command sent");
        } else if (cmd == 0) {
            stopMovement();
            server.send(200, "text/plain", "STOP command sent");
        } else {
            server.send(400, "text/plain", "Unknown command");
        }
    } else {
        server.send(400, "text/plain", "Missing cmd parameter");
    }
}
