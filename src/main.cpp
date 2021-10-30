#include <Arduino.h>
#include <Wire.h>

// Values returned by 'process'
// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

#define R_START 0x0

// Use the full-step state table (emits a code at 00 only)
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6

const unsigned char ttable[7][4] = {
    // R_START
    {R_START, R_CW_BEGIN, R_CCW_BEGIN, R_START},
    // R_CW_FINAL
    {R_CW_NEXT, R_START, R_CW_FINAL, R_START | DIR_CW},
    // R_CW_BEGIN
    {R_CW_NEXT, R_CW_BEGIN, R_START, R_START},
    // R_CW_NEXT
    {R_CW_NEXT, R_CW_BEGIN, R_CW_FINAL, R_START},
    // R_CCW_BEGIN
    {R_CCW_NEXT, R_START, R_CCW_BEGIN, R_START},
    // R_CCW_FINAL
    {R_CCW_NEXT, R_CCW_FINAL, R_START, R_START | DIR_CCW},
    // R_CCW_NEXT
    {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};

#define qh1 15 //d12
#define qh2 14 //d11
#define sh 12  // d9
#define clk 13 // d10

#define IRQ_PIN_INPUT 16 //D13
#define IRQ_PIN_CTRL 1

#define I2C_ADD 0x00

unsigned int rowPins[] = {22, 21, 20, 19};
unsigned int colPins[] = {11, 10, 9, 8, 7, 6, 5, 2};

//inputstates

//00 n 10 down 01 up
volatile byte enc1 = B00000000;
volatile byte enc2 = B00000000;

volatile byte encBtn = B00000000;

volatile byte ctrl1 = B00000000;
volatile byte ctrl2 = B00000000;

volatile byte matrix[] = {B00000000, B00000000, B00000000, B00000000};
//1-32 (pulldown)
volatile byte matrixTemp[] = {B00000000, B00000000, B00000000, B00000000};

bool update_matrix = false;
bool update_ctrl = false;

// 0 input, 1 ctrl
unsigned int nextSend = 0;

unsigned char encStates[6];
// set which Data is requested

void setOutput(int byteCount)
{
  nextSend = Wire.read();
}

//send Data when requested
void inputRead()
{
  if (nextSend == 0u)
  {

    //pull bring IRQ in idle position
    digitalWrite(IRQ_PIN_INPUT, HIGH);
    for (int i = 0; i < 4; i++)
    {
      Wire.write(matrix[i]);
    }
    update_matrix = false;
  }
  else
  {
    digitalWrite(IRQ_PIN_CTRL, HIGH);
    Wire.write(enc1);
    Wire.write(enc2);
    Wire.write(encBtn);
    Wire.write(ctrl1);
    Wire.write(ctrl2);
    //put encstep to a neutral state
    enc1 = B00000000;
    enc2 = B00000000;
    encBtn = B00000000;
    ctrl1 = B00000000;
    ctrl2 = B00000000;

    update_ctrl = false;
  }

  //send data
}

void checkRegs()
{

  // Load data to ShiftRegister
  digitalWrite(sh, HIGH);
  digitalWrite(sh, LOW);

  int stateA = 0;
  int stateB = 0;
  int indexEnc = 0;

  // check encoder pins (two per cycle)
  for (int i = 0; i < 12; i += 2)
  {

    // Pulse the Clock (rising edge shifts the next bit).
    digitalWrite(clk, HIGH);
    digitalWrite(clk, LOW);

    stateA = digitalRead(qh1);

    // Pulse the Clock (rising edge shifts the next bit).
    digitalWrite(clk, HIGH);
    digitalWrite(clk, LOW);

    stateB = digitalRead(qh1);

    //determin pinstate
    byte pinState = (stateA << 1) | stateB;
    indexEnc = i / 2;
    // Determine new state from the pins and state table.
    encStates[indexEnc] = ttable[encStates[indexEnc] & 0xf][pinState];
    Serial.println(encStates[indexEnc] + " , index: " + indexEnc);
    switch (encStates[indexEnc])
    {
    case DIR_CW:
      if (i < 8)
      {
        bitWrite(enc1, indexEnc, 1);
        bitWrite(enc1, indexEnc + 1, 0);
      }
      bitWrite(enc2, indexEnc, 1);
      bitWrite(enc2, indexEnc + 1, 0);
      update_ctrl = true;
      break;
    case DIR_CCW:
      if (i < 8)
      {
        bitWrite(enc1, indexEnc, 0);
        bitWrite(enc1, indexEnc + 1, 1);
      }
      bitWrite(enc2, indexEnc, 0);
      bitWrite(enc2, indexEnc + 1, 1);
      update_ctrl = true;
      break;
    default:
      break;
    }
  }
  // check remaining serial data
  for (int i = 0; i < 4; i++)
  {
    // Pulse the Clock (rising edge shifts the next bit).
    digitalWrite(clk, HIGH);
    digitalWrite(clk, LOW);

    int data = digitalRead(qh1);

    if (bitRead(encBtn, i) != data)
    {
      bitWrite(encBtn, i, data);
      update_ctrl = true;
    }
  }
  // read ctrl buttons
}

void setup()
{
  Serial.begin(9600);
  // put your setup code here, to run once:
  Wire.begin(I2C_ADD);
  Wire.onRequest(inputRead);
  Wire.onReceive(setOutput);
  // Assign variables.

  // Set pins to input.
  pinMode(qh1, INPUT);
  pinMode(qh2, INPUT);

  pinMode(sh, OUTPUT);
  pinMode(clk, OUTPUT);

  //pull up IRQ
  digitalWrite(IRQ_PIN_INPUT, HIGH);
  //pull up IRQ
  digitalWrite(IRQ_PIN_INPUT, HIGH);

  Serial.println("initialized successfully!");
}

void loop()
{
  //check matrix
  for (int r = 0; r < 4; r++)
  {
    for (int c = 0; c < 8; c++)
    {
      if (digitalRead(rowPins[r]) == 1 && digitalRead(colPins[c]) == 1)
      {
        //sets bit at c to 1 if button is pressed
        matrixTemp[r] |= 1u << c;
      }
      else
      {
        matrixTemp[r] |= 0u << c;
      }
    }
    //compare whole byte to avoid notification while bouncing
    if (matrix[r] != matrixTemp[r])
    {
      matrix[r] = matrixTemp[r];
      update_matrix = true;
    }
  }

  checkRegs();

  if (update_matrix)
  {
    if (millis() % 5000 == 0)
    {
      digitalWrite(IRQ_PIN_INPUT, HIGH);
    }
    Serial.println("matrix IRQ active (low)");
    //pull up IRQ
    digitalWrite(IRQ_PIN_INPUT, LOW);
  }
  if (update_ctrl)
  {
    if (millis() % 5000 == 0)
    {
      digitalWrite(IRQ_PIN_INPUT, HIGH);
    }
    Serial.println("ctrl IRQ active (low)");
    //pull up IRQ
    digitalWrite(IRQ_PIN_CTRL, LOW);
  }
}
