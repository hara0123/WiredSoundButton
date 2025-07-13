#include <Arduino.h>
#include <M5Stack.h>

#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

#include "DFRobotDFPlayerMini.h"

#include "KOGEI16_b.h"

// DEVICE_MAXは接続されたデバイス数ではなくシステムの最大値
#define DEVICE_MAX 5

#define SW1_PIN G13
#define SW2_PIN G0
#define SW3_PIN G36
#define SW4_PIN G35
#define SW5_PIN G34

#define RELAY1_PIN G22
#define RELAY2_PIN G21
#define RELAY3_PIN G2
#define RELAY4_PIN G12
#define RELAY5_PIN G15

#define SOUND_BUSY_PIN G5

// スイッチ入力をチェックするタイミング、**[ms]
#define INPUT_CHECK_TIMING 10
#define HEART_BEAT_COUNT 1000
#define TIMER_RESET_COUNT 3600000UL

// DFPlayerMiniでコマンドが連続するときに少し待たせる、**[ms]
#define DFPLAYER_COMMAND_WAIT_TIME 20


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
char decodeResultStr_[charBufferLen_];
char heartBeatStr_[charBufferLen_];
char debugStr_[charBufferLen_];
uint32_t heartBeat_ = 0;

uint16_t buttonStatus_; // 上位8ビットは1フレーム前のボタンの状態

// キュー
volatile bool buttonCheckQueue_;
volatile bool unitySendQueue_;

// タイマー
hw_timer_t* timer_ = nullptr;
volatile uint32_t timerCount_ = 0;

// マイコンからUnityへ送るデータ
const int toUnityDataLen_ = 8; // 'S' + スイッチ5個 + 'E' + '\0'
char toUnityData[toUnityDataLen_]; // スイッチ5個、例えばS11010Eが送られる

// Unityから送られるデータ
const int fromUnityDataLen_ = 50; // ひとまずダミー
char fromUnityData[fromUnityDataLen_];

// 背景色
const int bgColor_ = TFT_DARKGREEN;


// 以下、プロトタイプ宣言
void IRAM_ATTR onTimer();

void SpeakerSelect(uint8_t n);
uint8_t ButtonRead();
void PlaySound(uint8_t folderNo, uint8_t fileNo, uint8_t vol);

void CommandDecode();
bool CommandCheck(uint8_t spNo, uint8_t folderNo, uint8_t fileNo, uint8_t vol);

// キュー処理関数
void DoButtonCheckProcess();
void DoUnitySendProcess();

// 表示系関数
void DrawLOGO();
void DrawSoundModuleStatus(bool status);
void DrawButtonStatus(uint8_t swBit);
void DrawUnityData();
void DrawDecodeResult();
void DrawOtherInfo();
void DrawHeartBeat();
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
  decodeResultStr_[0] = '\0';
  heartBeatStr_[0] = '\0';
  debugStr_[0] = '\0';
  
  // lovyanGFX関連
  lcd_.init();
  lcd_.setRotation(1);
  lcd_.setBrightness(128);
  lcd_.setColorDepth(16);  // RGB565
  lcd_.drawLine(0, 0, 320, 0, TFT_YELLOW);
  lcd_.fillRect(0, 1, 320, 39, TFT_BLACK);
  lcd_.drawLine(0, 40, 320, 40, TFT_YELLOW);
  lcd_.fillRect(0, 41, 320, 165, bgColor_);
  lcd_.drawLine(0, 206, 320, 206, TFT_YELLOW);

  canvas_.createSprite(320, 165);
  canvas_.fillScreen(bgColor_);

  DrawLOGO();

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

  buttonStatus_ = 0x1F; // 下位8ビットのうち5ビットを1

  buttonCheckQueue_ = false;
  unitySendQueue_ = false;

  timer_ = timerBegin(0, getApbFrequency() / 1000000, true); // 1usでカウントアップ
  timerAttachInterrupt(timer_, &onTimer, true);
  timerAlarmWrite(timer_, 1000, true);  // 1000us=1msごとにonTimer関数が呼ばれる
  timerAlarmEnable(timer_);
}

void loop() {
  // put your main code here, to run repeatedly:
  // 先にUnityからの受信を処理
  //if(Serial.available() >= 9 && Serial.peek() == 'b')
  if(Serial.available() > 0)
  {
    CommandDecode();
  }

  // ボタンの状態確認内でDraw関数を呼ぶので、スプライトの消去はボタンの状態確認の前に行う
  canvas_.fillScreen(bgColor_);

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

  DrawUnityData();
  DrawDecodeResult();
  DrawOtherInfo();
  DrawHeartBeat(); // heartBeat_はOnTimer内で操作
  DrawDebugData();

  canvas_.pushSprite(0, 41);
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

  // ボタン4が上位ビット、ボタン0がLSB
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
    delay(DFPLAYER_COMMAND_WAIT_TIME);
  }
  dfplayer_.volume(vol);
  delay(DFPLAYER_COMMAND_WAIT_TIME);
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

      // フォルダ番号、2桁
      tmp[1] = Serial.read();
      tmp[0] = Serial.read();
      int folderNum = (tmp[1] - '0') * 10 + (tmp[0] - '0');
      *ptr = tmp[1];
      ptr++;
      *ptr = tmp[0];
      ptr++;
      
      // ファイル番号、3桁
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

      // ボリューム、2桁
      tmp[1] = Serial.read();
      tmp[0] = Serial.read();
      int vol = (tmp[1] - '0') * 10 + (tmp[0] - '0');
      *ptr = tmp[1];
      ptr++;
      *ptr = tmp[0];
      ptr++;

      snprintf(decodeResultStr_, sizeof(decodeResultStr_), " => sp:%d dir:%d file:%d vol:%d", id, folderNum, fileNum, vol);

      if(CommandCheck(id, folderNum, fileNum, vol))
      {
        SpeakerSelect(id); // idは0オリジン
        PlaySound(folderNum, fileNum, vol);
        debugStr_[0] = '\0';
      }
      else
      {
        strncpy(debugStr_, "unable to decode          ", sizeof(debugStr_));
      }

      // デバッグ用裏コマンド
      if(strcmp(fromUnityData, "b12345678") == 0)
      {
        strncpy(debugStr_, "b12345678                 ", sizeof(messageStr_));
        SpeakerSelect(0);
        PlaySound(1,1,15);
      }
      else if(strcmp(fromUnityData, "b99999999") == 0)
      {
        strncpy(debugStr_, "b99999999                 ", sizeof(messageStr_));
        esp_restart();
      }
    }  
}

bool CommandCheck(uint8_t spNo, uint8_t folderNo, uint8_t fileNo, uint8_t vol)
{
  // 引数はいずれも符号なしなので、負数はチェックしない
  if(spNo >= DEVICE_MAX)
  {
    return false;
  }

  if(folderNo > 99)
  {
    return false;
  }

  // fileNoはuint8_tの範囲なら再生可能なのでノーチェック

  if(vol > 30) // 実際はボリューム31は再生可能かもしれない
  {
    return false;
  }

  return true;
}

void IRAM_ATTR onTimer()
{
  if(timerCount_ % INPUT_CHECK_TIMING == 0)
  {
    buttonCheckQueue_ = true;
  }

  if(timerCount_ % HEART_BEAT_COUNT == 0)
  {
    heartBeat_++; // 処理が単純なのでloop内ではなくここで直接操作
  }

  timerCount_++;
  if(timerCount_ == TIMER_RESET_COUNT) // 60000で1分、3600000で1時間
  {
    timerCount_ = 0;
  }
}

// ボタンの確認
void DoButtonCheckProcess()
{
  uint8_t swBit = ButtonRead();

  bool changeFlag = false;

  // ボタン4が上位ビット、ボタン0がLSB
  // ボタンの押下を確認しつつUnityに送るデータを作成、作成はするが変化がなければ送信フラグは立てない → 立てることにした
  // Unityに送るデータはボタン0→4の並び
  for(int i = 0; i < DEVICE_MAX; i++)
  {
    if(buttonStatus_ >> i & 0x1 && (swBit >> i & 0x1) == 0)
    {
      // 押した瞬間
      changeFlag = true;
      toUnityData[1 + i] = '0'; // 冒頭の1+は'S'
    }
    else
    {
      toUnityData[1 + i] = '1'; // 冒頭の1+は'S'
    }
  }

  if(changeFlag)
  {
    // 変化あり
    strncpy(messageStr_, "button was pressed.", sizeof(messageStr_));
    // unitySendQueue_ = true;
  }
  else
  {
    messageStr_[0] = '\0';
  }

  unitySendQueue_ = true; // 変化がない場合でもUnityには送る
  DrawButtonStatus(swBit);

  buttonStatus_ <<= 8;
  buttonStatus_ |= swBit & 0x1F;
}

// Unityへ情報を送る
void DoUnitySendProcess()
{
  Serial.println(toUnityData);
}

void DrawLOGO()
{
  lcd_.setSwapBytes(true);
  lcd_.pushImage(0, 4, 48, 32, (uint16_t*)logoData);
}

void DrawSoundModuleStatus(bool status)
{
  lcd_.setFont(&fonts::Font4);
  lcd_.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd_.drawString("Sound Module", 56, 14);
  lcd_.setFont(&fonts::FreeSans18pt7b);
  if(status)
  {
    lcd_.drawString("OK", 240, 5);
  }
  else
  {
    lcd_.drawString("NG", 240, 5);
  }
}

void DrawButtonStatus(uint8_t swBit)
{
  const char label[] = "Button: ";
  strncpy(buttonStatusStr_, label, sizeof(buttonStatusStr_));

  int buttonPressedNo = -1;
  char* ptr = buttonStatusStr_ + sizeof(label) - 1;

  // 上位ビットから書く
  for(int i = DEVICE_MAX - 1; i >= 0; i--)
  {
    *ptr = (swBit >> i & 0x1) + '0';
    ptr++;
    if((swBit >> i & 0x1) == 0)
    {
      buttonPressedNo = i;
    }
  }
  *ptr = '\0';

  canvas_.setFont(&fonts::Font4);
  canvas_.setTextColor(TFT_WHITE, bgColor_);
  canvas_.drawString(buttonStatusStr_, 0, 5);

  if(buttonPressedNo != -1)
  {
    canvas_.setFont(&fonts::Font8);
    canvas_.setTextColor(TFT_WHITE, bgColor_);
    char c[2];
    c[0] = buttonPressedNo + '0' + 1; // 表示上のボタン番号は1オリジンにした
    c[1] = '\0';
    canvas_.drawString(c, 240, 5);
  }
}

void DrawUnityData()
{
  canvas_.setFont(&fonts::Font4);
  canvas_.setTextColor(TFT_WHITE, bgColor_);
  canvas_.drawString("SND:", 0, 35);
  canvas_.drawString(toUnityData, 64, 35);
  canvas_.drawString("RCV:", 0, 65);
  canvas_.drawString(fromUnityData, 64, 65);
}

void DrawDecodeResult()
{
  canvas_.drawString(decodeResultStr_, 0, 95);
}

void DrawOtherInfo()
{
  canvas_.drawString(messageStr_, 0, 125);
}

void DrawHeartBeat()
{
  canvas_.setFont(&fonts::Font0);
  canvas_.setTextColor(TFT_WHITE, bgColor_);
  sprintf(heartBeatStr_, "%lu", heartBeat_); // heartBeatStr_はOnTimerで値を操作していて文字列に変換するちょうどよいタイミングがなく、ここで変換
  canvas_.drawString(heartBeatStr_, 260, 155);
}

void DrawDebugData()
{
  lcd_.setFont(&fonts::FreeMonoOblique12pt7b);
  lcd_.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd_.drawString(debugStr_, 0, 212);
}
