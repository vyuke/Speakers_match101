/*
 * This example demonstrates using the pattern matching engine (CuriePME)
 * to classify streams of accelerometer data from CurieIMU.
 *
 * First, the sketch will prompt you to draw some letters in the air (just
 * imagine you are writing on an invisible whiteboard, using your board as the
 * pen), and the IMU data from these motions is used as training data for the
 * PME. Once training is finished, you can keep drawing letters and the PME
 * will try to guess which letter you are drawing.
 *
 * This example requires a button to be connected to digital pin 4
 * https://www.arduino.cc/en/Tutorial/Button
 *
 * NOTE: For best results, draw big letters, at least 1-2 feet tall.
 *
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See license notice at end of file.
 */

#include "CurieIMU.h" //调用curie的IMU库
#include "CuriePME.h" //调用模式匹配引擎库

#define LED_pin 13 //定义LED灯的引脚为13号
/*  This controls how many times a letter must be drawn during training.
 *  Any higher than 4, and you may not have enough neurons for all 26 letters
 *  of the alphabet. Lower than 4 means less work for you to train a letter,
 *  but the PME may have a harder time classifying that letter. */
const unsigned int trainingReps = 4; //定义动作的学习次数

/* Increase this to 'A-Z' if you like-- it just takes a lot longer to train */
const unsigned char trainingStart = 'A'; //第一个动作用A表示
const unsigned char trainingEnd = 'E';   //第五个动作用E表示

/* The input pin used to signal when a letter is being drawn- you'll
 * need to make sure a button is attached to this pin */
const unsigned int buttonPin = 4; //定义识别按钮接到4号引脚上

/* Sample rate for accelerometer */
const unsigned int sampleRateHZ = 200; //加速度采样率

/* No. of bytes that one neuron can hold */
const unsigned int vectorNumBytes = 128; //一个神经元可以容纳的字节数

/* Number of processed samples (1 sample == accel x, y, z)
 * that can fit inside a neuron */
const unsigned int samplesPerVector = (vectorNumBytes / 3); //给每个轴的数据分配1/3的空间

/* This value is used to convert ASCII characters A-Z
 * into decimal values 1-26, and back again. */
const unsigned int upperStart = 0x40; //ASCII码字符转换参数

const unsigned int sensorBufSize = 2048; //缓存空间
const int IMULow = -32768;               //划分IMU左右极限
const int IMUHigh = 32767;

void setup()
{
    Serial.begin(9600);  //101串口波特率设置为9600
    Serial1.begin(9600); //101串口1波特率设置为9600，与蓝牙模块相连
    while (!Serial)      //等待串口启动
        ;

    pinMode(buttonPin, INPUT); //初始化按钮引脚为输入模式
    pinMode(LED_pin, 1);       //初始化LED灯的引脚为输出模式

    /* Start the IMU (Intertial Measurement Unit), enable the accelerometer */
    CurieIMU.begin(ACCEL); //初始化IMU

    /* Start the PME (Pattern Matching Engine) */
    CuriePME.begin(); //初始化模式匹配引擎

    CurieIMU.setAccelerometerRate(sampleRateHZ); //设置加速度采样率
    CurieIMU.setAccelerometerRange(2);           //设置加速度范围

    trainLetters(); //开始学习动作的函数
    Serial.println("Training complete. Now, draw some letters (remember to ");
    Serial.println("hold the button) and see if the PME can classify them.");
}

void loop()
{
    byte vector[vectorNumBytes]; //向量数组，用于存放动作数据
    unsigned int category;       //匹配到的值
    char letter;                 //代表动作的字母的变量

    /* Record IMU data while button is being held, and
     * convert it to a suitable vector */
    readVectorFromIMU(vector); //读取动作加速度数据

    /* Use the PME to classify the vector, i.e. return a category
     * from 1-26, representing a letter from A-Z */
    category = CuriePME.classify(vector, vectorNumBytes); //匹配动作加速度数据

    if (category == CuriePME.noMatch) //如果没有匹配到
    {
        Serial.println("Don't recognise that one-- try again."); //用串口说不认识，再一次。
        Serial1.print('X');                                      //发送识别错误信息给音箱
    }
    else
    {
        letter = category + upperStart; //否则就输出这个匹配到的值
        //Serial.println(letter);
        switch (letter)
        {
        case 'A': //如果是A，就代表下一曲，并且用串口1将这个值用蓝牙发送出去，下面以此类推
            Serial.println("Next song");
            Serial1.print('A');
            break;
        case 'B':
            Serial.println("Prev song");
            Serial1.print('B');
            break;
        case 'C':
            Serial.println("Volume_up");
            Serial1.print('C');
            break;
        case 'D':
            Serial.println("Volume_down");
            Serial1.print('D');
            break;
        case 'E':
            Serial.println("Play/Psuse");
            Serial1.print('E');
            break;
        }
    }
}

/* Simple "moving average" filter, removes low noise and other small
 * anomalies, with the effect of smoothing out the data stream. */
byte getAverageSample(byte samples[], unsigned int num, unsigned int pos,
                      unsigned int step) //这整个函数用滑动平均法滤波，除去底频噪声，处理动作数据，可略过
{
    unsigned int ret;
    unsigned int size = step * 2;

    if (pos < (step * 3) || pos > (num * 3) - (step * 3))
    {
        ret = samples[pos];
    }
    else
    {
        ret = 0;
        pos -= (step * 3);
        for (unsigned int i = 0; i < size; ++i)
        {
            ret += samples[pos - (3 * i)];
        }

        ret /= size;
    }

    return (byte)ret;
}

/* We need to compress the stream of raw accelerometer data into 128 bytes, so
 * it will fit into a neuron, while preserving as much of the original pattern
 * as possible. Assuming there will typically be 1-2 seconds worth of
 * accelerometer data at 200Hz, we will need to throw away over 90% of it to
 * meet that goal!
 *
 * This is done in 2 ways:
 *
 * 1. Each sample consists of 3 signed 16-bit values (one each for X, Y and Z).
 *    Map each 16 bit value to a range of 0-255 and pack it into a byte,
 *    cutting sample size in half.
 *
 * 2. Undersample. If we are sampling at 200Hz and the button is held for 1.2
 *    seconds, then we'll have ~240 samples. Since we know now that each
 *    sample, once compressed, will occupy 3 of our neuron's 128 bytes
 *    (see #1), then we know we can only fit 42 of those 240 samples into a
 *    single neuron (128 / 3 = 42.666). So if we take (for example) every 5th
 *    sample until we have 42, then we should cover most of the sample window
 *    and have some semblance of the original pattern. */
void undersample(byte samples[], int numSamples, byte vector[]) //对数据进行压缩取样，可略过
{
    unsigned int vi = 0;
    unsigned int si = 0;
    unsigned int step = numSamples / samplesPerVector;
    unsigned int remainder = numSamples - (step * samplesPerVector);

    /* Centre sample window */
    samples += (remainder / 2) * 3;
    for (unsigned int i = 0; i < samplesPerVector; ++i)
    {
        for (unsigned int j = 0; j < 3; ++j)
        {
            vector[vi + j] = getAverageSample(samples, numSamples, si + j, step);
        }

        si += (step * 3);
        vi += 3;
    }
}

void readVectorFromIMU(byte vector[]) //从IMU中读取动作向量
{
    byte accel[sensorBufSize]; //加速度值
    int raw[3];

    unsigned int samples = 0;
    unsigned int i = 0;

    /* Wait until button is pressed */
    while (digitalRead(buttonPin) == LOW) //等待按钮按下
        digitalWrite(LED_pin, 0);         //熄灭LED
    ;

    /* While button is being held... */
    while (digitalRead(buttonPin) == HIGH) //按下按钮后
    {
        digitalWrite(LED_pin, 1); //点亮LED
        if (CurieIMU.dataReady()) //如果IMU准备就绪后
        {

            CurieIMU.readAccelerometer(raw[0], raw[1], raw[2]); //读取加速度数据

            /* Map raw values to 0-255 */
            accel[i] = (byte)map(raw[0], IMULow, IMUHigh, 0, 255); //三轴的数据分别存入加速度数组中
            accel[i + 1] = (byte)map(raw[1], IMULow, IMUHigh, 0, 255);
            accel[i + 2] = (byte)map(raw[2], IMULow, IMUHigh, 0, 255);

            i += 3;
            ++samples;

            /* If there's not enough room left in the buffers
            * for the next read, then we're done */
            if (i + 3 > sensorBufSize)
            {
                break;
            }
        }
    }

    undersample(accel, samples, vector);
}

void trainLetter(char letter, unsigned int repeat) //训练函数
{
    unsigned int i = 0;

    while (i < repeat)
    {
        byte vector[vectorNumBytes]; //动作向量

        if (i)
            Serial.println("And again..."); //再一次

        readVectorFromIMU(vector);
        CuriePME.learn(vector, vectorNumBytes, letter - upperStart); //神经网络开始学习获取到的动作数据

        Serial.println("Got it!"); //学习好后继续下一步
        delay(1000);               //缓冲一秒
        ++i;
    }
}

void trainLetters() //整个函数用于向串口传递当前的状态，给用户指示需要做的动作
{
    for (char i = trainingStart; i <= trainingEnd; ++i)
    {
        Serial.print("Hold down the button and doing action '");
        //Serial.print(String(i) + "' in the air. Release the button as soon ");
        //////
        switch (i)
        {
        case 'A':
            Serial.print("Next song'"); //做下一曲的动作
            break;
        case 'B':
            Serial.print("Prev song'"); //做上一曲的动作
            break;
        case 'C':
            Serial.print("Volume_up'"); //做音量加的动作
            break;
        case 'D':
            Serial.print("Volume_down'"); //做音量减的动作
            break;
        case 'E':
            Serial.print("Play/Psuse'"); //做播放暂停的动作
            break;
        }
        //////
        Serial.println(" as you are done.");
        trainLetter(i, trainingReps);
        Serial.println("OK, finished with this letter.");
        delay(2000); //加油哦，相信你！
    }
}

/*
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */
