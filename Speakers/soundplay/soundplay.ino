#include <SoftwareSerial.h> //调用软串口库
#define ARDUINO_RX 10       //should connect to TX of the Serial MP3 Player module
#define ARDUINO_TX 11       //connect to RX of the module

//MP3模块指令集
#define CMD_SEL_DEV 0X09     //指定播放设备
#define DEV_TF 0X02          //TF卡
#define CMD_NEXT_SONG 0X01   //下一曲
#define CMD_PREV_SONG 0X02   //上一曲
#define CMD_VOLUME_UP 0X04   //音量加
#define CMD_VOLUME_DOWN 0X05 //音量减
#define CMD_PLAY 0X0D        //播放
#define CMD_PAUSE 0X0E       //暂停

unsigned char playmode = 1; //定义一个播放变量用于存储播放状态
#define PLAY 1
#define PAUSE 0

static int8_t Send_buf[8] = {0};                 //定义缓冲区列队指令
SoftwareSerial mySerial(ARDUINO_RX, ARDUINO_TX); //创建软串口对象mySerial
#define LED_pin 13                               //LED灯为13号引脚

void serialEvent()
{
  while (Serial.available()) //检查当前串口缓冲区是否有内容
  {
    char inChar = (char)Serial.read(); //把串口读到的指令存放到inChar中

    switch (inChar)
    {
    case 'A': //next song
      sendCommand(CMD_NEXT_SONG, 0);
      BBlink(1);//每发出一个指令闪烁一次LED
      break;
    case 'B': //prev song
      sendCommand(CMD_PREV_SONG, 0);
      BBlink(1);
      break;
    case 'C': //volume up
      sendCommand(CMD_VOLUME_UP, 0);
      BBlink(1);
      break;
    case 'D': //volume down
      sendCommand(CMD_VOLUME_DOWN, 0);
      BBlink(1);
      break;
    case 'E': //play/pause
      if (playmode == PLAY)
      {
        playmode = PAUSE;
        sendCommand(CMD_PAUSE, 0); //暂停
      }
      else
      {
        playmode = PLAY;
        sendCommand(CMD_PLAY, 0); //播放
      }
      BBlink(1);
      break;

    default:
      BBlink(3); //如果识别到非法指令，就闪烁三次LED
      break;
    }
  }
}

void BBlink(int inc) //LED灯闪烁控制函数
{
  for (int c = 0; c < inc; c++)
  {
    digitalWrite(LED_pin, 1);
    delay(100);
    digitalWrite(LED_pin, 0);
    delay(100);
  }
}

void sendCommand(int8_t command, int16_t dat) //发送命令功能函数
{
  delay(20);
  Send_buf[0] = 0x7e;               //starting byte
  Send_buf[1] = 0xff;               //version
  Send_buf[2] = 0x06;               //the number of bytes of the command without starting byte and ending byte
  Send_buf[3] = command;            //
  Send_buf[4] = 0x00;               //0x00 = no feedback, 0x01 = feedback
  Send_buf[5] = (int8_t)(dat >> 8); //datah
  Send_buf[6] = (int8_t)(dat);      //datal
  Send_buf[7] = 0xef;               //ending byte
  for (uint8_t i = 0; i < 8; i++)   //
  {
    mySerial.write(Send_buf[i]);
  }
}

void setup()
{
  Serial.begin(9600);               //初始化串口
  mySerial.begin(9600);             //初始化软串口
  sendCommand(CMD_SEL_DEV, DEV_TF); //select the TF card
  pinMode(LED_pin, 1);              //LED引脚为输出模式
  digitalWrite(LED_pin, 0);         //熄灭LED灯
}

void loop()
{
}
