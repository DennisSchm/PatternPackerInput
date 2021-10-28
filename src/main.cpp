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

#define IRQ_PIN_INPUT 16
#define IRQ_PIN_CTRL 1

unsigned int rowPins[] = {22, 21, 20, 19};
unsigned int colPins[] = {11, 10, 9, 8, 7, 6, 5, 2};

//inputstates

//00 n 10 down 01 up
// enc 1 - 4
volatile byte shift1_1 = 0;
// enc 5-6 & enc btn 1-4
volatile byte shift1_2 = 0;

volatile byte shift2_1 = 0;
volatile byte shift2_2 = 0;

//1-8
volatile byte matrix[] = {0, 0, 0, 0};
//1-32 (pulldown)
volatile byte matrixTemp[] = {0, 0, 0, 0};

bool update_inp = false;
bool update_ctrl = false;

// 0 input, 1 ctrl
unsigned int nextSend = 0;

unsigned char encStates[6];

void setup()
{
  // put your setup code here, to run once:
  Wire.begin(0x01);
  Wire.onRequest(inputRead);
  Wire.onReceive(setOutput);
  // Assign variables.

  // Set pins to input.
  pinMode(qh1, INPUT);
  pinMode(qh2, INPUT);

  pinMode(sh, OUTPUT);
  pinMode(clk, OUTPUT);
}

void loop()
{
  for (int r = 0; r < 4; r++)
  {
    byte row = 0x0;
    for (int c = 0; c < 8; c++)
    {
      if (digitalRead(rowPins[r]) == 1 && digitalRead(colPins[c]) == 1)
        //sets bit at c to 1 if button is pressed
        matrixTemp[r] |= 1u << c;
      else
        matrixTemp[r] |= 0u << c;
    }
    //compare whole byte to avoid notification while bouncing
    if (matrix[r] != matrixTemp[r])
    {
      matrix[r] = matrixTemp[r];
      update_inp = true;
    }
  }

  checkRegs();

  if (update_inp)
    //pull up IRQ
    digitalWrite(24, HIGH);
  if (update_ctrl)
    //pull up IRQ
    digitalWrite(23, HIGH);
}
void setOutput(int byteCount)
{
  nextSend = Wire.read();
}

void inputRead()
{
  if (nextSend == 0u)
  {

    //pull down IRQ
    digitalWrite(IRQ_PIN_INPUT, HIGH);
    for (int i = 0; i < 4; i++)
    {
      Wire.write(matrix[i]);
    }
    bool update_inp = false;
  }
  else
  {
    digitalWrite(IRQ_PIN_CTRL, LOW);
    Wire.write(shift1_1);
    Wire.write(shift1_2);
    Wire.write(shift2_1);
    Wire.write(shift2_2);
    //put encstep to a neutral state
    shift1_1 = 0;
    byte shiftmask = ((1 << 4) - 1) << 4;
    shift1_2 = (shift1_2 & ~shiftmask);
    bool update_ctrl = false;
  }

  //send data
}

byte checkRegs()
{

  // Load data to ShiftRegister
  digitalWrite(sh, HIGH);
  digitalWrite(sh, LOW);

  int stateA = 0;
  int stateB = 0;
  int index = 0;

  // check encoder pins (two per cycle)
  for (int i = 0; i < 12; i++)
  {

    // Pulse the Clock (rising edge shifts the next bit).
    digitalWrite(clk, HIGH);
    digitalWrite(clk, LOW);
    stateA = digitalRead(qh1);

    // Pulse the Clock (rising edge shifts the next bit).
    digitalWrite(clk, HIGH);
    digitalWrite(clk, LOW);
    stateB = digitalRead(qh1);

    i++;
    index = i / 2;
    byte state = (stateA << 1) | stateB;
    encStates[i / 2] = state;
    if (state == DIR_CW || state == DIR_CCW)
    {
      if (i <= 8)
      {
        const int shiftBy = i - 1;
        // mask two digits to read current state
        byte shiftmask = ((1 << 2) - 1) << shiftBy;
        byte stateCurr = (shift1_1 >> shiftBy);
        // check if state has changed since last time
        if (stateCurr != (state >> 4))
        {
          //invert mask & shift in new state
          shift1_1 = (shift1_1 & ~shiftmask) | (state << shiftBy);
          // signal teensy to pull update
          update_inp = true;
        }
      }
      else
      {
        const int shiftBy = i - 9;
        // mask two digits to read current state
        byte shiftmask = ((1 << 2) - 1) << shiftBy;
        byte stateCurr = (shift1_2 >> shiftBy);

        if (stateCurr != (state >> 4))
        {
          //invert mask & shift in new state
          shift1_2 = (shift1_2 & ~shiftmask) | (state << shiftBy);
          // signal teensy to pull update
          update_inp = true;
        }
      }
    }
  }
}