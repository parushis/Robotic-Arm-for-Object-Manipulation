#include <FastLED.h>
#include <Servo.h>


const static uint16_t DOC5[] = { 523 };
const static uint16_t DOC6[] = { 1047u };

const static uint8_t servoPins[5] = { 7, 6, 5, 4, 3};
const static uint8_t buzzerPin = 11;
const static uint8_t rgbPin = 13;
typedef enum {
  MODE_KNOB,
  MODE_APP,
  MODE_ACTIONGROUP,
  MODE_EXTENDED,
} UhandMode;

static CRGB rgbs[1];

static UhandMode g_mode = MODE_KNOB;                          /* mode, 0 -> panel, 1 -> app */
static uint8_t knob_angles[5] = { 90, 90, 90, 90, 90};   /* 旋钮产生的角度数值 */
static uint8_t servo_angles[5] = { 90, 90, 90, 90, 90};  /* 舵机实际控制的角度数值 */

static uint16_t tune_num = 0;
static uint32_t tune_beat = 10;
static uint16_t *tune;

Servo servos[5];

static void knob_update(void);   /* 旋钮读取更新 */
static void servo_control(void); /* 舵机控制 */
void tune_task(void);

void setup() {
  Serial.begin(9600);
  // 设置串行端口读取数据的超时时间
  Serial.setTimeout(500);
  pinMode(buzzerPin, OUTPUT);
  // 绑定舵机IO口
  for (int i = 0; i < 5; ++i) {
    servos[i].attach(servoPins[i]);
  }

  FastLED.addLeds<WS2812, rgbPin, GRB>(rgbs, 1);
  rgbs[0] = CRGB(0, 255, 0);
  FastLED.show();
  tone(buzzerPin, 1000);
  delay(100);
  noTone(buzzerPin);
}

void loop() {
  // 蜂鸣器鸣响任务
  tune_task();
  // 旋钮读取更新
  knob_update();
  // 舵机控制
  servo_control();
}

void knob_update(void) { /* 旋钮读取更新 */
  static uint32_t last_tick = 0;
  static float values[5];
  float angle = 0;
  if (millis() - last_tick < 10) {
    return;
  }
  last_tick = millis();
  values[0] = values[0] * 0.7 + analogRead(A0) * 0.3;
  values[1] = values[1] * 0.7 + analogRead(A1) * 0.3;
  values[2] = values[2] * 0.7 + analogRead(A2) * 0.3;
  values[3] = values[3] * 0.7 + analogRead(A3) * 0.3;
  values[4] = values[4] * 0.7 + analogRead(A4) * 0.3;

  for (int i = 0; i < 5; ++i) {
    angle = map(values[i], 0, 1023, 0, 180);
    angle = angle < 0 ? 0 : (angle > 180 ? 180 : angle);
    if (fabs(angle - knob_angles[i]) > 5 && g_mode != MODE_KNOB) { /* 当发现旋钮被旋转超过阈值则恢复旋钮控制模式(可能处于手机app控制模式) */
      g_mode = MODE_KNOB;
      rgbs[0].r = 0;
      rgbs[0].g = 255;
      rgbs[0].b = 0;
      FastLED.show();
    }
    if (g_mode == MODE_KNOB) {
      knob_angles[i] = angle;
      if (i == 5) {
      }
    }
  }
}

void servo_control(void) {
  static uint32_t last_tick = 0;
  if (millis() - last_tick < 25) {
    return;
  }
  last_tick = millis();
  Serial.print("{");
  for (int i = 0; i < 5; ++i) {
    servo_angles[i] = servo_angles[i] * 0.85 + knob_angles[i] * 0.15;
    servos[i].write(i == 0 || i == 5 ? 180 - servo_angles[i] : servo_angles[i]);
    
    Serial.print(servo_angles[i]);
    if(i<4){
      Serial.print(" ,");
    }
  }
  Serial.println("}");
}

void tune_task(void) {
  static uint32_t l_tune_beat = 0;
  static uint32_t last_tick = 0;
  // 若未到定时时间 且 响的次数跟上一次的一样
  if (millis() - last_tick < l_tune_beat && tune_beat == l_tune_beat) {
    return;
  }
  l_tune_beat = tune_beat;
  last_tick = millis();
  if (tune_num > 0) {
    tune_num -= 1;
    tone(buzzerPin, *tune++);
  } else {
    noTone(buzzerPin);
    tune_beat = 10;
    l_tune_beat = 10;
  }
}
