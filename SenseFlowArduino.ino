#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SdFat.h>
#include <RtcDS1302.h>
#include <ThreeWire.h>
#include <Adafruit_ADS1X15.h>
#include <Keypad.h>
#include <math.h>

// ---------------------- Константы и пины ----------------------
#define RST_PIN 8
#define DAT_PIN 7
#define CLK_PIN 6
ThreeWire myWire(DAT_PIN, CLK_PIN, RST_PIN);
RtcDS1302<ThreeWire> Rtc(myWire);

// Параметры Timer1 (120 Гц)
#define TIMER1_PRESCALER 64
#define TIMER1_OCR_VALUE ((16000000UL / (TIMER1_PRESCALER * 120)) - 1)

// ADS1115
Adafruit_ADS1115 ads;

// Клавиатура 4x4
const byte ROWS = 4, COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {9, 8, 7, 6};
byte colPins[COLS] = {5, 4, 3, 2};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// SD карта
SdFat sd;
File32 dataFile;
File32 ddataFile;

// ---------------------- Оптимизированные настройки ----------------------
static const uint8_t  rrBufferSize      = 4;
static const float    signalAlpha       = 0.10f;   // базовое EMA сигнала
static const float    signalWarmupAlpha = 0.3f;   // ускоренное EMA на старте
static const float    hrAlpha           = 0.08f;   // базовое EMA ЧСС
static const float    startupAlpha      = 0.22f;   // ускоренное EMA ЧСС первые 2 с
static const uint16_t warmupTime        = 3500;    // мс – сколько длится «разогрев»

uint32_t measurementStartTime = 0;                 // Время начала измерения
static const uint32_t minBeatInterval = 250000UL;  // 250 мс между пиками
static const uint8_t  rawWindowSize   = 6;
static const uint16_t chunkSize       = 32;
static uint8_t        sampleCounter   = 0;

// ---------------------- Флаги ----------------------
struct {
  bool acquisitionStarted   : 1;
  bool readyToLog           : 1;
  bool warmupDone   : 1;  // Новый флаг завершения инициализации
  volatile bool sampleNow   : 1;
  volatile bool bufferReadyToWrite : 1;
  volatile bool writingInProgress  : 1;
} flags = {0};

char  bufferA[chunkSize];
char  bufferB[chunkSize];
volatile uint16_t bufferAIndex = 0;
volatile uint16_t bufferBIndex = 0;
volatile bool     usingA       = true;

char  dBuffer[chunkSize];
volatile uint16_t dBufferIndex       = 0;
volatile bool     dBufferReadyToWrite = false;

uint32_t lastBufferFlush = 0;

// Таймеры
uint32_t logStartTime = 0;
short timer           = 0;
uint32_t lastKeyTime  = 0;

// ---------------------- Структура канала ----------------------
struct ChannelData {
  uint8_t  pin;
  uint16_t baseThreshold;

  volatile bool     peakDetected;
  volatile uint32_t lastPeakTime;

  uint32_t rrIntervals[rrBufferSize];
  uint8_t  rrIndex;
  uint8_t  rrCount;
  float    emaHR;

  int16_t  rawBuffer[rawWindowSize];
  uint8_t  rawIndex;
  int16_t  filterBuffer[5];
  uint8_t  filterIndex;
  int32_t  filterSum;
  bool     filterFilled;
  float    emaSignal;

  int16_t  maxSignal;
  int16_t  minSignal;
};

// ---------------------- Каналы ----------------------
ChannelData channels[2] = {
  {A6, 540, false, 0, {0}, 0, 0, 0.0f, {0}, 0, {0}, 0, 0, false, 0.0f, 0, 1023},
  {A2, 510, false, 0, {0}, 0, 0, 0.0f, {0}, 0, {0}, 0, 0, false, 0.0f, 0, 1023}
};

// ---------------------- Прототипы ----------------------
void setupTimer1();
ISR(TIMER1_COMPA_vect, ISR_NOBLOCK);
void handleSampling();
int16_t getUltraFilteredSignal(ChannelData &ch);
void insertionSort(int16_t* arr, uint8_t size);
void updateLCD();
void logValue(int16_t bpm1, int16_t bpm2, int16_t rVal);
void logDate();
int16_t calculateR();
float correctResistance(float rMeas_k);
void flushBuffer();
inline void resetChannelAmplitude(ChannelData &ch);

// ---------------------- SETUP ----------------------
void setup() {
  Serial.begin(115200);
  if(!sd.begin(10, SD_SCK_MHZ(18))) { }
  Rtc.Begin();
  if(!Rtc.GetIsRunning()) Rtc.SetIsRunning(true);
  if(!ads.begin()) { }
  ads.setGain(GAIN_TWOTHIRDS);

  lcd.init();
  lcd.backlight();
  lcd.print(F("Press '1'"));
  lcd.setCursor(0, 1);
  lcd.print(F("to Start"));

  setupTimer1();
}

// ---------------------- LOOP ----------------------
void loop() {
  uint32_t now = millis();

  if (flags.acquisitionStarted && !flags.warmupDone && 
      (now - measurementStartTime >= warmupTime)) {
    flags.warmupDone = true;
    logStartTime = now; // Сбрасываем время начала логирования
    // lcd.clear(); // Очищаем экран для перехода в основной режим
  }

  // Старт логирования после первой секунды работы
  if(!flags.readyToLog && flags.warmupDone && logStartTime && (now - logStartTime > 1000)) {
    flags.readyToLog = true;
    logDate();
    lcd.clear();
  }

  // Запись буферов на SD
  if(flags.bufferReadyToWrite || (now - lastBufferFlush >= 2000)) {
    flushBuffer();
  }

  // ------------------ Клавиатура ------------------
  char key = keypad.getKey();
  if(key && (now - lastKeyTime > 200)) {
    lastKeyTime = now;
    switch(key) {
      case '1':
        // --- старт ---
        flags.acquisitionStarted = true;
        flags.warmupDone = false; // Сбрасываем флаг инициализации
        flags.readyToLog = false;
        logStartTime = now;
        measurementStartTime = now;
        lcd.clear();
        timer = 0;

        // «Праймируем» emaSignal 10‑ю быстрыми замерами
        for(uint8_t c=0; c<2; c++) {
          long sum = 0;
          for(uint8_t s=0; s<10; s++) sum += analogRead(channels[c].pin);
          channels[c].emaSignal = sum / 10.0f;
          channels[c].emaHR     = 0.0f;
        }
        break;

      case '2':
        // --- стоп ---
        flags.acquisitionStarted = false;
        lcd.clear();
        lcd.print(F("Press '1'"));
        lcd.setCursor(0, 1);
        lcd.print(F("To start"));

        flushBuffer();
        if(usingA && bufferAIndex) {
          dataFile.write(bufferA, bufferAIndex);
          bufferAIndex = 0;
        } else if(bufferBIndex) {
          dataFile.write(bufferB, bufferBIndex);
          bufferBIndex = 0;
        }
        if(dataFile) {
          char endMsg[10];
          snprintf(endMsg, sizeof(endMsg), "%d сек\n", timer);
          dataFile.print(endMsg);
          dataFile.close();
        }
        if(ddataFile) {
          char endMsg[10];
          snprintf(endMsg, sizeof(endMsg), "%d сек\n", timer);
          ddataFile.print(endMsg);
          ddataFile.close();
        }
        break;
    }
  }

  // ------------------ Обработка семплов ------------------
  if(flags.acquisitionStarted && flags.sampleNow) {
    handleSampling();
    flags.sampleNow = false;
  }

  // ------------------ Обновление LCD + сброс амплитуды ------------------
  static uint32_t lastUpdate = 0;
  uint16_t resetInterval = (now - measurementStartTime < 5000) ? 250 : 1000;
  if(flags.acquisitionStarted && (now - lastUpdate >= resetInterval)) {
    for(uint8_t i = 0; i < 2; i++) resetChannelAmplitude(channels[i]);
    updateLCD();
    lastUpdate = now;
  }
}

// ---------------------- ТАЙМЕР ----------------------
void setupTimer1() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC, делитель 64
  OCR1A  = TIMER1_OCR_VALUE;
  TIMSK1 = (1 << OCIE1A);
  interrupts();
}

ISR(TIMER1_COMPA_vect, ISR_NOBLOCK) {
  flags.sampleNow = true;
}

// ---------------------- ОБРАБОТКА СЕМПЛОВ ----------------------
void handleSampling() {
  int16_t filteredVals[2];

  for (uint8_t i = 0; i < 2; i++) {
    ChannelData &ch = channels[i];

    // Сырой сигнал
    ch.rawBuffer[ch.rawIndex] = analogRead(ch.pin);
    ch.rawIndex = (ch.rawIndex + 1) % rawWindowSize;

    // Сверх‑фильтруем (медиана + EMA)
    filteredVals[i] = getUltraFilteredSignal(ch);

    // --- EMA сигнала с ускоренной альфой на старте ---
    float sigAlpha = (millis() - measurementStartTime < warmupTime) ? signalWarmupAlpha : signalAlpha;
    ch.emaSignal += sigAlpha * (filteredVals[i] - ch.emaSignal);

    // Обновляем амплитуду пика
    if (filteredVals[i] > ch.maxSignal) ch.maxSignal = filteredVals[i];
    if (filteredVals[i] < ch.minSignal) ch.minSignal = filteredVals[i];

    // ------------------ Детектор пика ------------------
    uint32_t t = micros();
    float delta       = fabsf(filteredVals[i] - ch.emaSignal);
    float dynamicBase = ch.minSignal + (ch.maxSignal - ch.minSignal) * 0.3f; // 30 % от диапазона
    float dynFactor   = (millis() - measurementStartTime < 3000) ? 0.32f : 0.52f;

    if (ch.emaSignal > dynamicBase &&
        delta > (ch.maxSignal - ch.minSignal) * dynFactor &&
        !ch.peakDetected &&
        (t - ch.lastPeakTime) > minBeatInterval) {

      ch.peakDetected = true;
      uint32_t rr = t - ch.lastPeakTime;
      ch.lastPeakTime = t;
      ch.rrIntervals[ch.rrIndex++] = rr;
      ch.rrIndex %= rrBufferSize;
      if (ch.rrCount < rrBufferSize) ch.rrCount++;

      if (rr > 0) {
        float bpm = 60000000.0f / rr;
        if (bpm >= 40.0f && bpm <= 180.0f) {
          bool  inStartup   = (millis() - measurementStartTime < 2000);
          float currentAlpha = inStartup ? startupAlpha : hrAlpha;
          ch.emaHR += currentAlpha * (bpm - ch.emaHR);
        }
      }
    } else if (ch.emaSignal < dynamicBase) {
      ch.peakDetected = false;
    }
  }

  // Запись отфильтрованных значений в отдельный файл
  if (++sampleCounter >= 6) {
    if (ddataFile) {
      char buf[12];
      int len = snprintf(buf, sizeof(buf), "%d %d\n", filteredVals[0], filteredVals[1]);
      ddataFile.write(buf, len);
    }
    sampleCounter = 0;
  }
}

// ---------------------- ФИЛЬТРАЦИЯ ----------------------
int16_t getUltraFilteredSignal(ChannelData &ch) {
  int16_t temp[rawWindowSize];
  memcpy(temp, ch.rawBuffer, sizeof(temp));
  insertionSort(temp, rawWindowSize);
  int16_t med = temp[rawWindowSize >> 1];

  static float prevVal = 0;
  float alpha = (millis() - measurementStartTime < warmupTime) ? 0.8f : 0.3f;
  float filtered = alpha * med + (1.0f - alpha) * prevVal;
  prevVal = filtered;
  return (int16_t)filtered;
}

// ---------------------- СОРТИРОВКА ВСТАВКАМИ ----------------------
void insertionSort(int16_t* arr, uint8_t size) {
  for(uint8_t i = 1; i < size; i++) {
    int16_t key = arr[i];
    int8_t j = i - 1;
    while(j >= 0 && arr[j] > key) {
      arr[j+1] = arr[j];
      j--;
    }
    arr[j+1] = key;
  }
}

// ---------------------- LCD ----------------------
void updateLCD() {
  if (!flags.acquisitionStarted) return;

  // Во время инициализации показываем сообщение
  if (!flags.warmupDone) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Initializing..."));
    return;
  }

  // Основной режим отображения
  if (!flags.readyToLog) return;

  int16_t bpm1 = (int16_t)(channels[0].emaHR + 0.5f);
  int16_t bpm2 = (int16_t)(channels[1].emaHR + 0.5f);
  int16_t rVal = calculateR();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(bpm1); lcd.print(" BPM");
  lcd.setCursor(0, 1);
  lcd.print(bpm2); lcd.print(" BPM");
  lcd.setCursor(12,0); 
  
  
  // Таймер теперь считает только после инициализации
  if (!flags.warmupDone) {
    lcd.print(0);
  } else {
    lcd.print(timer);
    timer++;
  }
  
  lcd.setCursor(12,1); lcd.print(rVal);

  // if (flags.acquisitionStarted) {
  //   logValue(bpm1, bpm2, rVal);
  //   Serial.print("BPM1:");
  //   Serial.print(bpm1);
  //   Serial.print(";BPM2:");
  //   Serial.print(bpm2);
  //   Serial.print(";R:");
  //   Serial.println(rVal);
  // }
}

void logValue(int16_t bpm1, int16_t bpm2, int16_t rVal) {
  char line[16];
  uint8_t len = snprintf(line, sizeof(line), "%d %d %d\n", bpm1, bpm2, rVal);
  char* tgt = usingA ? bufferA : bufferB;
  volatile uint16_t &idx = usingA ? bufferAIndex : bufferBIndex;

  if(idx + len < chunkSize) {
    memcpy(tgt + idx, line, len);
    idx += len;
    if(idx >= chunkSize - 16) flags.bufferReadyToWrite = true;
  }
}

// ---------------------- SD CARD ----------------------
void flushBuffer() {
  noInterrupts();
  bool localUsingA = usingA;
  uint16_t localAIndex = bufferAIndex;
  uint16_t localBIndex = bufferBIndex;
  uint16_t localDIndex = dBufferIndex;
  bool localDReady = dBufferReadyToWrite;
  interrupts();

  if(localUsingA && localAIndex) {
    dataFile.write(bufferA, localAIndex);
    bufferAIndex = 0;
  } else if(localBIndex) {
    dataFile.write(bufferB, localBIndex);
    bufferBIndex = 0;
  }

  if(localDIndex && localDReady) {
    ddataFile.write(dBuffer, localDIndex);
    dBufferIndex = 0;
    dBufferReadyToWrite = false;
  }

  flags.bufferReadyToWrite = false;
  lastBufferFlush = millis();
  usingA = !usingA; // переключаемся на другой буфер
}

// ---------------------- R‑измерение ----------------------
int16_t calculateR() {
  static int16_t rBuffer[3] = {0};
  static uint8_t index = 0;
  static bool filled = false;

  int16_t adcDiff = ads.readADC_Differential_0_1();
  float voltage = adcDiff * (6.144f / 32768.0f);
  float v = fabsf(voltage);
  if(v < 0.0001f || v >= 5.0f) return 0;

  float R_meas_k = (v * 320.0f) / (5.0f - v);
  if(R_meas_k < 0.1f) return 0;

  float R_corr_k = correctResistance(R_meas_k);
  int16_t currentVal = (int16_t)(R_corr_k + 0.5f);

  rBuffer[index] = currentVal;
  index = (index + 1) % 3;
  filled = filled || (index == 0);

  int16_t sum = 0;
  uint8_t count = filled ? 3 : index;
  for(uint8_t i = 0; i < count; i++) sum += rBuffer[i];
  return sum / count;
}

float correctResistance (float rMeas_k) {
  if (rMeas_k < 0.1f)           // всё, что ниже 100 Ом – шум
    return rMeas_k;

  float lnR = logf(rMeas_k);
  float lnCorr = 0.00950f*lnR*lnR*lnR
               - 0.09950f*lnR*lnR
               + 1.25500f*lnR
               - 0.12649f;
  float Rcorr = expf(lnCorr);

  if (Rcorr < 7)
    return Rcorr;

  float lnRc = logf(Rcorr);
  float scale = 0.01439f*lnRc*lnRc - 0.05102f*lnRc + 0.74220f;
  if (Rcorr > 655) {
    return Rcorr * scale * 1.78;
  } else if (Rcorr > 280 && Rcorr < 470) {
    return Rcorr * scale * 1.65;
  }
  return Rcorr * scale * 1.69;
}


inline void resetChannelAmplitude(ChannelData &ch) {
  ch.maxSignal = 0;
  ch.minSignal = 1023;
}

// ---------------------- Логирование даты ----------------------
void logDate() {
  RtcDateTime now = Rtc.GetDateTime();
  char filename[32];
  char dfilename[33];
  sprintf(filename,  "%02d%02d%02d_%02d%02d%02d.txt",
          now.Day(), now.Month(), now.Year() % 100,
          now.Hour(), now.Minute(), now.Second());
  sprintf(dfilename,"D%02d%02d%02d_%02d%02d%02d.txt",
          now.Day(), now.Month(), now.Year() % 100,
          now.Hour(), now.Minute(), now.Second());

  dataFile.open(filename,  O_RDWR | O_CREAT | O_AT_END);
  ddataFile.open(dfilename,O_RDWR | O_CREAT | O_AT_END);
}
