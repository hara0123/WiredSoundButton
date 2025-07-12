#include <Arduino.h>
#include <M5Stack.h>

#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>  // クラス"LGFX"を準備します

#include "DFRobotDFPlayerMini.h"

// DEVICE_MAXは接続されたデバイス数ではなくシステムの最大値
#define DEVICE_MAX 5

#define SW1_PIN G34
#define SW2_PIN G35
#define SW3_PIN G36
#define SW4_PIN G0
#define SW5_PIN G13

#define RELAY1_PIN G22
#define RELAY2_PIN G21
#define RELAY3_PIN G2
#define RELAY4_PIN G12
#define RELAY5_PIN G15

#define SOUND_BUSY_PIN G5

// スイッチ入力をチェックするタイミング
#define INPUT_CHECK_TIMING 10 // **[ms]
#define TIMER_RESET_COUNT 3600000UL


// 以下、グローバル変数
static LGFX lcd_;
static LGFX_Sprite canvas_(&lcd_);

DFRobotDFPlayerMini dfplayer_; // Serial2で接続、BUSYピンはG5に接続

uint8_t swPin_[DEVICE_MAX];
uint8_t relayPin_[DEVICE_MAX];

const int charBufferLen_ = 60;
char messageStr_[charBufferLen_];
char soundModuleStatusStr_[charBufferLen_];
char buttonStatusStr_[charBufferLen_];
char debugStr_[charBufferLen_];
uint32_t debugCount_;

// キュー
bool buttonCheckQueue_;
bool unitySendQueue_;

// タイマー
hw_timer_t* timer = nullptr;
uint32_t timerCount = 0;

// マイコンからUnityへ送るデータ
const int toUnityDataLen = 8; // 'S' + スイッチ5個 + 'E' + '\0'
char toUnityData[toUnityDataLen]; // スイッチ5個、例えばS11010Eが送られる

// Unityから送られるデータ
const int fromUnityDataLen = 50; // ひとまずダミー
char fromUnityData[fromUnityDataLen];

// 以下、プロトタイプ宣言
void IRAM_ATTR onTimer();

void SpeakerSelect(uint8_t n);
uint8_t ButtonRead();
void PlaySound(uint8_t folderNo, uint8_t fileNo, uint8_t vol);

void CommandDecode();

// キュー処理関数
void DoButtonCheckProcess();
void DoUnitySendProcess();

// 表示系関数
void DrawSoundModuleStatus(bool status);
void DrawButtonStatus(uint8_t swBit);
void DrawUnityData();
void DrawOtherInfo();
void DrawDebugData();


// 以下、関数定義
void setup() {
  // put your setup code here, to run once:
  pinMode(SW1_PIN, INPUT);
  pinMode(SW2_PIN, INPUT);
  pinMode(SW3_PIN, INPUT);
  pinMode(SW4_PIN, INPUT);
  pinMode(SW5_PIN, INPUT);

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  pinMode(RELAY5_PIN, OUTPUT);

  swPin_[0] = SW1_PIN;
  swPin_[1] = SW2_PIN;
  swPin_[2] = SW3_PIN;
  swPin_[3] = SW4_PIN;
  swPin_[4] = SW5_PIN;

  relayPin_[0] = RELAY1_PIN;
  relayPin_[1] = RELAY2_PIN;
  relayPin_[2] = RELAY3_PIN;
  relayPin_[3] = RELAY4_PIN;
  relayPin_[4] = RELAY5_PIN;

  // Unity用シリアル通信
  Serial.begin(115200);

  strncpy(toUnityData, "S11111E", sizeof(toUnityData));
  fromUnityData[0] = '\0';
  //strncpy(fromUnityData, "aaa", sizeof(fromUnityData)); // デバッグ中はダミーデータを用意
  debugStr_[0] = '\0';
  debugCount_ = 0;

  // lovyanGFX関連
  lcd_.init();
  lcd_.setRotation(1);
  lcd_.setBrightness(128);
  lcd_.setColorDepth(16);  // RGB565
  lcd_.setFont(&fonts::lgfxJapanGothic_20);

  // DFPlayerMini関連
  pinMode(SOUND_BUSY_PIN, INPUT);
  Serial2.begin(9600); // DFPlayerMiniのデフォルトボーレートは9600bps
  if (!dfplayer_.begin(Serial2))
  {
    DrawSoundModuleStatus(false);
  }
  else
  {
    DrawSoundModuleStatus(true);
  }

  buttonCheckQueue_ = false;
  unitySendQueue_ = false;

  timer = timerBegin(0, getApbFrequency() / 1000000, true); // 1usでカウントアップ
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);  // 1000us=1msごとにonTimer関数が呼ばれる
  timerAlarmEnable(timer);
}

void loop() {
  // put your main code here, to run repeatedly:
  // 先にUnityからの受信を処理
  if(Serial.available() > 0)
  {
    sprintf(debugStr_, "received, %d", debugCount_);
    debugCount_++;
    CommandDecode();
  }

  // ボタンの状態確認
  if(buttonCheckQueue_)
  {
    buttonCheckQueue_ = false;
    DoButtonCheckProcess();
  }

  if(unitySendQueue_)
  {
    unitySendQueue_ = false;
    DoUnitySendProcess();
  }

  DrawUnityData(); // 毎フレーム描画
  DrawOtherInfo();
  DrawDebugData();
}

void SpeakerSelect(uint8_t n)
{
  if(n >= DEVICE_MAX)
    return;

  for(uint8_t i = 0; i < DEVICE_MAX; i++ )
  {
    if(i != n)
    {
      digitalWrite(relayPin_[i], LOW);
    }
    else{
      digitalWrite(relayPin_[i], HIGH);
    }
  }
}

uint8_t ButtonRead()
{
  uint8_t swBit = 0;

  for(int i = DEVICE_MAX -1; i >= 0; i--)
  {
    swBit |= (digitalRead(swPin_[i])) << i;
  }

  return swBit;
}

void PlaySound(uint8_t folderNo, uint8_t fileNo, uint8_t vol)
{
  int soundBusyPin = digitalRead(SOUND_BUSY_PIN);
  if(soundBusyPin == LOW)
  {
    // 再生中、強制停止
    dfplayer_.stop();
  }
  dfplayer_.volume(vol);
  dfplayer_.playFolder(folderNo, fileNo);
}

void CommandDecode()
{
   char inputChar = Serial.read();
    // コマンドはbxddfffvvの9文字
    if(inputChar == 'b')
    {
      char tmp[3];
      char* ptr = fromUnityData;
      
      *ptr = 'b';
      ptr++;

      // 残り8字
      // ID、有線の場合はスピーカー番号
      tmp[0] = Serial.read();
      uint8_t id = tmp[0] - '0';
      *ptr = tmp[0];
      ptr++;

      // フォルダ番号、2桁など自力でデコードしてやる
      tmp[1] = Serial.read();
      tmp[0] = Serial.read();
      int folderNum = (tmp[1] - '0') * 10 + (tmp[0] - '0');
      *ptr = tmp[1];
      ptr++;
      *ptr = tmp[0];
      ptr++;
      
      // ファイル番号、3桁など自力でデコードしてやる
      tmp[2] = Serial.read();
      tmp[1] = Serial.read();
      tmp[0] = Serial.read();
      int fileNum = (tmp[2] - '0') * 100 + (tmp[1] - '0') * 10 + (tmp[0] - '0');
      *ptr = tmp[2];
      ptr++;
      *ptr = tmp[1];
      ptr++;
      *ptr = tmp[0];
      ptr++;

      // ボリューム、2桁など自力でデコードしてやる
      tmp[1] = Serial.read();
      tmp[0] = Serial.read();
      int vol = (tmp[1] - '0') * 10 + (tmp[0] - '0');
      *ptr = tmp[1];
      ptr++;
      *ptr = tmp[0];
      ptr++;

      if(strcmp(fromUnityData, "b12345678") == 0)
      {
        strncpy(messageStr_, "itti", sizeof(messageStr_));
        SpeakerSelect(0);
        PlaySound(1,1,15);
      }
      else if(strcmp(fromUnityData, "b99999999") == 0)
      {
        strncpy(messageStr_, "itti", sizeof(messageStr_));
        SpeakerSelect(1);
        PlaySound(1,1,15);
      }
      else
      {
        strncpy(messageStr_, "xxxxx", sizeof(messageStr_));
      }

      snprintf(messageStr_, sizeof(messageStr_), "%d %d %d %d", id, folderNum, fileNum, vol);
    }  
}

void IRAM_ATTR onTimer()
{
  if(timerCount % INPUT_CHECK_TIMING == 0)
  {
    buttonCheckQueue_ = true;
  }

  timerCount++;
  if(timerCount == TIMER_RESET_COUNT) // 60000で1分、3600000で1時間
  {
    timerCount = 0;
  }
}

// ボタンの確認
void DoButtonCheckProcess()
{
  uint8_t swBit = ButtonRead();

  DrawButtonStatus(swBit);
}

// Unityへ情報を送る
void DoUnitySendProcess()
{
  Serial.println(toUnityData);
}

void DrawSoundModuleStatus(bool status)
{
  lcd_.fillRect(0, 0, 20, 20, (uint8_t)0xE0);  // 赤で矩形の塗りを描画
  lcd_.fillRect(10, 10, 20, 20, (uint8_t)0x1C);  // 緑で矩形の塗りを描画

  if(status)
  {
    strncpy(soundModuleStatusStr_, "Sound Module Online.", sizeof(soundModuleStatusStr_));
  }
  else
  {
    strncpy(soundModuleStatusStr_, "Sound Module unable to begin.", sizeof(soundModuleStatusStr_));
  }
  lcd_.drawString(soundModuleStatusStr_, 40, 0);
}

void DrawButtonStatus(uint8_t swBit)
{
  const char label[] = "Button: ";
  strncpy(buttonStatusStr_, label, sizeof(buttonStatusStr_));

  char* ptr = buttonStatusStr_ + sizeof(label) - 1;
  for(int i = 0; i < DEVICE_MAX; i++)
  {
    *ptr = ((swBit >> i) & 0x1) + '0';
    ptr++;
  }
  *ptr = '\0';

  lcd_.drawString(buttonStatusStr_, 0, 50);
}

void DrawUnityData()
{
  lcd_.drawString(toUnityData, 0, 90);
  lcd_.drawString(fromUnityData, 0, 120);
}

void DrawOtherInfo()
{
  lcd_.drawString(messageStr_, 0, 160);
}

void DrawDebugData()
{
  lcd_.drawString(debugStr_, 0, 200);
}
