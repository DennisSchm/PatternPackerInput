#include <Arduino.h>
#include <Wire.h>

const int IRQ_CTRL = 11;
const int IRQ_MATRIX = 12;
// const int IRQ_ENC = 13;

const int SDA_I2C = A4;
const int SCL_I2C = A5;

const int I2C_ADD = 80;

// Pin to 74HC595
const int dataOutPin = 2;
const int clockOutPin = 3;
const int latchOutPin = 4;

// Pin to 74HC165
const int dataInPin = 5;
const int clockInPin = 6;
const int latchInPin = 7;

// indicates if matrixstate or ctrl states gets send on request
uint8_t nextOutput = B00000000;

// No complete step yet.
const uint8_t DIR_NONE = B00000000;
// Clockwise step.
const uint8_t DIR_CW = B00010000;
// Anti-clockwise step.
const uint8_t DIR_CCW = B00100000;

const uint8_t R_START = B00000000;
// Use the full-step state table (emits a code at 00 only)
const uint8_t R_CW_FINAL = B00000001;
const uint8_t R_CW_BEGIN = B00000010;
const uint8_t R_CW_NEXT = B00000011;
const uint8_t R_CCW_BEGIN = B00000100;
const uint8_t R_CCW_FINAL = B00000101;
const uint8_t R_CCW_NEXT = B00000110;

const uint8_t ttable[7][4] = {
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

volatile char matrixBtns[]{0, 0, 0, 0};
volatile uint8_t matrixBtnsSent[]{0, 0, 0, 0};
volatile bool matrixBtnsDiffDetected{false};
unsigned long matrixBtnsDiffDetectedAt{0};

volatile uint16_t ctrlBtns{0};
volatile uint16_t ctrlBtnsSent{0};
volatile bool ctrlBtnsDiffDetected{false};
unsigned long ctrlBtnsDiffDetectedAt{0};

volatile uint16_t encBtns{0};
volatile uint16_t encBtnsSent{0};
volatile bool encBtnsDiffDetected{false};
unsigned long encBtnsDiffDetectedAt{0};

volatile int8_t enc[9]{};
volatile bool encsDiffDetected{false};
volatile uint32_t encStates{0};

// statemachine makes debounce obsolete
volatile uint8_t encTransStates[9] = {R_START, R_START, R_START, R_START, R_START, R_START, R_START, R_START, R_START};

bool isDifferentToSentMatrixBtns()
{
  for (int i{0}; i < 4; i++)
    if (matrixBtns[i] != matrixBtnsSent[i])
      return true;
  return false;
}

/*
ctrl and encoder
*/
void updateCtrlBtn(bool state, int num)
{
  if (bitRead(ctrlBtns, num) != state)
  {
    bitWrite(ctrlBtns, num, state);
    Serial.print("Ctrl Btn ");
    Serial.print(num, DEC);
    Serial.print(" updated: ");
    Serial.print(state);
    Serial.print(" | ");
  }
}
void updateEncBtn(bool state, int num)
{

  if (bitRead(encBtns, num) != state)
  {
    bitWrite(encBtns, num, state);
    Serial.print("Enc Btn ");
    Serial.print(num, DEC);
    Serial.print(" updated: ");
    Serial.print(state);
    Serial.print(" | ");
  }
}
// enc 1-4 --> encState1, 4-8 --> encState2, 9 encState3
void updateEnc(int a, int b, int num)
{

  byte newState = (a << 1) | b;
  // Determine new state from bitA and B and state table.
  encTransStates[num] = ttable[encTransStates[num] & B00001111][newState];
  byte maskedState = encTransStates[num] & B00110000;

  if (maskedState == DIR_CW)
  {
    enc[num]++;
    Serial.print("Enc ");
    Serial.print(num, DEC);
    Serial.println(" incremented");
  }
  if (maskedState == DIR_CCW)
  {
    enc[num]--;
    Serial.print("Enc ");
    Serial.print(num, DEC);
    Serial.println(" decremented");
  }
}
void setActiveRow(byte row)
{
  digitalWrite(clockOutPin, LOW);
  digitalWrite(latchOutPin, HIGH);                  // shift serial data to storrage/output
  shiftOut(dataOutPin, clockOutPin, LSBFIRST, row); // write that byte
  digitalWrite(latchOutPin, LOW);                   // set latch to low so we can write an entire byte at once
}
// to save battery reduces times this function gets called
void readShift()
{
  byte active = B01000000;
  // repeat for every row of matrix
  for (int iRow = 0; iRow < 4; iRow++)
  {
    digitalWrite(clockInPin, HIGH);
    digitalWrite(latchInPin, LOW);
    digitalWrite(latchInPin, HIGH);

    char reg = shiftIn(dataInPin, clockInPin, LSBFIRST);
    updateCtrlBtn(bitRead(reg, 0), 2);
    updateCtrlBtn(bitRead(reg, 1), 3);
    updateEncBtn(bitRead(reg, 2), 3);
    updateEncBtn(bitRead(reg, 3), 2);
    updateEnc(bitRead(reg, 4), bitRead(reg, 5), 3);
    updateEnc(bitRead(reg, 6), bitRead(reg, 7), 2);

    reg = shiftIn(dataInPin, clockInPin, LSBFIRST);
    updateCtrlBtn(bitRead(reg, 0), 1);
    updateEncBtn(bitRead(reg, 1), 1);
    updateCtrlBtn(bitRead(reg, 2), 0);
    updateEncBtn(bitRead(reg, 3), 0);
    updateEnc(bitRead(reg, 5), bitRead(reg, 4), 0);
    updateEnc(bitRead(reg, 7), bitRead(reg, 6), 1);

    reg = shiftIn(dataInPin, clockInPin, LSBFIRST);
    updateEnc(bitRead(reg, 1), bitRead(reg, 0), 5);
    updateEnc(bitRead(reg, 2), bitRead(reg, 3), 4);
    updateCtrlBtn(bitRead(reg, 4), 5);
    updateCtrlBtn(bitRead(reg, 5), 4);
    updateEncBtn(bitRead(reg, 6), 4);
    updateEncBtn(bitRead(reg, 7), 5);

    reg = shiftIn(dataInPin, clockInPin, LSBFIRST);
    updateCtrlBtn(bitRead(reg, 0), 6);
    updateCtrlBtn(bitRead(reg, 1), 7);
    updateCtrlBtn(bitRead(reg, 2), 8);
    updateEncBtn(bitRead(reg, 3), 6);
    updateCtrlBtn(bitRead(reg, 4), 10);
    updateCtrlBtn(bitRead(reg, 5), 11);
    updateEnc(bitRead(reg, 7), bitRead(reg, 6), 6);

    reg = shiftIn(dataInPin, clockInPin, LSBFIRST);
    updateCtrlBtn(bitRead(reg, 0), 12);
    updateCtrlBtn(bitRead(reg, 1), 9);
    updateEncBtn(bitRead(reg, 2), 7);
    updateEncBtn(bitRead(reg, 3), 8);
    updateEnc(bitRead(reg, 5), bitRead(reg, 4), 7);
    updateEnc(bitRead(reg, 7), bitRead(reg, 6), 8);

    if (iRow == 3)
      active = B10000000;
    setActiveRow(active);
    active = active >> 1;
    reg = shiftIn(dataInPin, clockInPin, LSBFIRST);
    // set rows to negative to prevent bleeding
    setActiveRow(B00000000);
    if (reg != 0)
    {
      char regCopy = reg;
      bitWrite(reg, 0, bitRead(regCopy, 3));
      bitWrite(reg, 1, bitRead(regCopy, 2));
      bitWrite(reg, 2, bitRead(regCopy, 1));
      bitWrite(reg, 3, bitRead(regCopy, 0));

      char regDummy = reg;
      for (int i{0}; i < 8; i++) // reverse Bit because of wiring
        bitWrite(reg, i, bitRead(regDummy, 7 - i));
    }

    if (reg != matrixBtns[iRow])
    {
      Serial.println("matrix updated");
      for (int c{0}; c < 8; c++)
      {
        int newBit = bitRead(reg, c);               // check the state of that bit
        int prevBit = bitRead(matrixBtns[iRow], c); // check the previous state of that bit
        if (newBit != prevBit)
        {
          Serial.print(iRow);
          Serial.print(",");
          Serial.print(c);
          Serial.print(" | ");
        }
      }
      matrixBtns[iRow] = reg;
    }
  }
}

void sendCtrl()
{
  Serial.println("ctrl requested");
  // sende, header
  char header{};
  bitWrite(header, 0, ctrlBtnsDiffDetected);
  bitWrite(header, 1, encBtnsDiffDetected);
  bitWrite(header, 2, encsDiffDetected);
  Wire.write(header);
  if (ctrlBtnsDiffDetected)
  {
    ctrlBtnsDiffDetected = false;
    uint8_t lsb = ctrlBtns;
    uint8_t msb = ctrlBtns >> 8;
    Wire.write(lsb);
    Wire.write(msb);
    ctrlBtnsSent = ctrlBtns;
  }
  if (encBtnsDiffDetected)
  {
    encBtnsDiffDetected = false;
    uint8_t msb = encBtns >> 8;
    uint8_t lsb = encBtns;
    Wire.write(lsb);
    Wire.write(msb);
    encBtnsSent = encBtns;
  }
  if (encsDiffDetected)
  {
    encsDiffDetected = false;
    uint8_t encHeaderA{};
    uint8_t encHeaderB{};
    for (int i{0}; i < 8; i++)
      if (enc[i] != 0)
        bitWrite(encHeaderA, i, 1);
    if (enc[8] != 0)
      bitWrite(encHeaderB, 0, 1);

    Wire.write(encHeaderA);
    Wire.write(encHeaderB);

    for (int i{0}; i < 9; i++)
      if (enc[i] != 0)
        Wire.write(enc[i]);
    for (int i{0}; i < 9; i++)
      enc[i] = 0;
  }
  digitalWrite(IRQ_CTRL, HIGH);
}
void sendMatrix()
{
  char header{0};
  for (int r{0}; r < 4; r++)
    if (matrixBtns[r] != matrixBtnsSent[r])
      bitWrite(header, r, 1);
  Wire.write(header);

  for (int r{0}; r < 4; r++)
    if (matrixBtns[r] != matrixBtnsSent[r])
    {
      Serial.println(matrixBtns[r], BIN);
      Wire.write(matrixBtns[r]);
    }

  for (int r{0}; r < 4; r++)
    matrixBtnsSent[r] = matrixBtns[r];
  digitalWrite(IRQ_MATRIX, HIGH);
}
void setOutput(int _)
{
  byte requestedOutput = Wire.read();
  // matrix on 0, ctrl on 1, enc on 2
  nextOutput = requestedOutput;
}
void sendInputData()
{
  if (nextOutput == B00000000)
  {
    sendMatrix();
  }
  else
  {
    sendCtrl();
  }
}

void setup()
{
  // set pins to output because they are addressed in the main loop
  pinMode(latchOutPin, OUTPUT);
  pinMode(dataOutPin, OUTPUT);
  pinMode(clockOutPin, OUTPUT);

  pinMode(latchInPin, OUTPUT);
  pinMode(clockInPin, OUTPUT);
  pinMode(dataInPin, INPUT);

  pinMode(IRQ_CTRL, OUTPUT);
  pinMode(IRQ_MATRIX, OUTPUT);
  // pinMode(IRQ_ENC, OUTPUT);

  digitalWrite(IRQ_CTRL, HIGH);
  digitalWrite(IRQ_MATRIX, HIGH);
  // digitalWrite(IRQ_ENC, HIGH);

  Serial.begin(9600);
  Serial.println("reset");
  Wire.begin(I2C_ADD);
  Wire.onReceive(setOutput);
  Wire.onRequest(sendInputData);

  setActiveRow(B00000000); // make sure shift register starts at all HIGH
}

void loop()
{
  readShift();
  if (ctrlBtnsSent != ctrlBtns)
  {
    if (!ctrlBtnsDiffDetected)
    {
      ctrlBtnsDiffDetected = true;
      ctrlBtnsDiffDetectedAt = millis();
    }
    else
    {
      if (ctrlBtnsDiffDetectedAt <= millis() - 50)
      {
        digitalWrite(IRQ_CTRL, HIGH);
        digitalWrite(IRQ_CTRL, LOW);
        Serial.print("ctrl low, ");
      }
    }
  }
  else
  {
    ctrlBtnsDiffDetected = false;
  }

  if (encBtnsSent != encBtns)
  {
    if (!encBtnsDiffDetected)
    {
      encBtnsDiffDetected = true;
      encBtnsDiffDetectedAt = millis();
    }
    else
    {
      if (encBtnsDiffDetectedAt <= millis() - 50)
      {
        digitalWrite(IRQ_CTRL, HIGH);
        digitalWrite(IRQ_CTRL, LOW);
        Serial.print("ctrl low, ");
      }
    }
  }
  else
  {
    encBtnsDiffDetected = false;
  }

  if (isDifferentToSentMatrixBtns())
  {
    if (!matrixBtnsDiffDetected)
    {
      matrixBtnsDiffDetected = true;
      matrixBtnsDiffDetectedAt = millis();
    }
    else
    {
      if (matrixBtnsDiffDetectedAt <= millis() - 50)
      {
        digitalWrite(IRQ_MATRIX, HIGH);
        digitalWrite(IRQ_MATRIX, LOW);
        Serial.print("matrix low, ");
      }
    }
  }
  else
  {
    matrixBtnsDiffDetected = false;
  }

  bool encChanged{false};
  for (int i{0}; i < 9; i++)
  {
    if (enc[i] != 0)
    {
      encChanged = true;
      encsDiffDetected = true;
      digitalWrite(IRQ_CTRL, HIGH);
      digitalWrite(IRQ_CTRL, LOW);
      Serial.print("ctrl low");
    }
  }
  encsDiffDetected = encChanged;
}
