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

volatile uint8_t regsMatrix[]{};
volatile uint8_t regsCtrl[]{};

volatile uint8_t matrix[4]{};
volatile uint8_t matrixPrev[4]{};
volatile uint8_t matrixSent[4]{};
volatile uint8_t ctrl[2]{};
volatile uint8_t ctrlPrev[2]{};
volatile uint8_t ctrlSent[2]{}; // 13 elements
volatile uint8_t encBtn[2];     // 9 elements
volatile uint8_t encBtnPrev[9];
volatile uint8_t encBtnSent[9];
volatile int enc[9]{};
// statemachine no need for debounce
volatile uint8_t encTransitionStates[9] = {R_START, R_START, R_START, R_START, R_START, R_START, R_START, R_START, R_START};

volatile unsigned long matrix_lastChangeAt{0};
volatile unsigned long ctrlData_lastChangeAt{0};

volatile uint8_t transmissionDataMatrix[5];
volatile int cBytesTransmissionDataMatrix{};
volatile uint8_t transmissionDataCtrl[13];
volatile int cBytesTransmissionDataCtrl{};
bool dataSentCtrl{false};
bool dataSentMatrix{false};

void setDataSentCtrl()
{
  for (int i{0}; i < 2; i++)
    ctrlSent[i] = ctrl[i];
  for (int i{0}; i < 2; i++)
    encBtnSent[i] = encBtn[i];
  for (int i{0}; i < 9; i++)
    enc[i] = 0;
}

void setDataSentMatrix()
{
  for (int i{0}; i < 4; i++)
    matrixSent[i] = matrix[i];
}

void prepTransmissionDataMatrix()
{

  int iDataBlock{0};
  for (int i{0}; i < 4; i++)
    if (matrix[i] != matrixSent[i])
      bitWrite(transmissionDataMatrix[iDataBlock], i, 1);
  iDataBlock++;
  for (int i{0}; i < 4; i++)
  {
    if (matrix[i] != matrixSent[i])
    {
      transmissionDataMatrix[iDataBlock] = matrix[i];
      iDataBlock++;
    }
  }
  cBytesTransmissionDataMatrix = iDataBlock;
}
void prepTransmissionDataCtrl()
{
  // clear prev transmissionData
  for (int i{0}; i < 13; i++)
    transmissionDataCtrl[i] = 0;
  // first two bytes hold information which ctrl data has changed
  bool ctrlBtnUpdates{ctrlSent[0] != ctrl[0] || ctrlSent[1] != ctrl[1]};
  bool encBtnUpdates{encBtn[0] != encBtnSent[0] || encBtn[1] != encBtnSent[1]};
  bool encUpdates{false};
  for (int i{0}; i < 9; i++)
    if (enc[i] != 0)
      encUpdates = true;
  bool ctrlBtnUpdates0To7{ctrlSent[0] != ctrl[0]};

  bool ctrlBtnUpdates8To12{ctrlSent[1] != ctrl[1]};
  bool encBtnUpdates0To8{encBtn[0] != encBtnSent[0]};
  bool encBtnUpdates9{encBtn[1] != encBtnSent[1]};
  bool encUpdate0{enc[0] != 0};

  int iDataBlock{0};
  bitWrite(transmissionDataCtrl[iDataBlock], 0, ctrlBtnUpdates);
  bitWrite(transmissionDataCtrl[iDataBlock], 1, encBtnUpdates);
  bitWrite(transmissionDataCtrl[iDataBlock], 2, encUpdates);
  bitWrite(transmissionDataCtrl[iDataBlock], 3, ctrlBtnUpdates0To7);
  bitWrite(transmissionDataCtrl[iDataBlock], 4, ctrlBtnUpdates8To12);
  bitWrite(transmissionDataCtrl[iDataBlock], 5, encBtnUpdates0To8);
  bitWrite(transmissionDataCtrl[iDataBlock], 6, encBtnUpdates9);
  bitWrite(transmissionDataCtrl[iDataBlock], 7, encUpdate0);
  iDataBlock++;
  if (encUpdates)
  {
    for (int i{1}; i < 9; i++)
    {
      if (enc[i] != 0)
      {
        bitWrite(transmissionDataCtrl[iDataBlock], i - 1, true);
      }
    }
    iDataBlock++;
  }

  if (ctrlBtnUpdates)
  {
    if (ctrlBtnUpdates0To7)
    {
      transmissionDataCtrl[iDataBlock] = ctrl[0];
      iDataBlock++;
    }
    if (ctrlBtnUpdates8To12)
    {
      transmissionDataCtrl[iDataBlock] = ctrl[1];
      iDataBlock++;
    }
  }
  if (encBtnUpdates)
  {
    if (encBtnUpdates0To8)
    {
      transmissionDataCtrl[iDataBlock] = encBtn[0];
      iDataBlock++;
    }
    if (encBtnUpdates9)
    {
      transmissionDataCtrl[iDataBlock] = encBtn[1];
      iDataBlock++;
    }
  }
  if (encUpdates)
  {
    for (int i{0}; i < 9; i++)
    {
      if (enc[i] != 0)
      {
        transmissionDataCtrl[iDataBlock] = (int8_t)enc[i];
        iDataBlock++;
      }
    }
  }
  cBytesTransmissionDataCtrl = iDataBlock;
}

void sendDataCtrl()
{
  // set all enc to 0
}

void setActiveRow(byte row)
{
  digitalWrite(clockOutPin, LOW);
  digitalWrite(latchOutPin, HIGH);                  // close latch to shift byte out paralel
  shiftOut(dataOutPin, clockOutPin, LSBFIRST, row); // write that byte
  digitalWrite(latchOutPin, LOW);                   // shift out
}
void readRegs()
{
  byte activeRow = B10000000;
  for (int i{0}; i < 4; i++)
  {
    setActiveRow(activeRow);

    // discard first 3 cycles of ctrl data
    // TODO design PCB so that matrix register is adressable seperate to avoid discarding
    if (i < 3)
      for (int j{}; j < 5; j++)
        shiftIn(dataInPin, clockInPin, LSBFIRST);
    else
      for (int j{}; j < 5; j++)
        regsCtrl[j] = shiftIn(dataInPin, clockInPin, LSBFIRST);

    regsMatrix[i] = shiftIn(dataInPin, clockInPin, LSBFIRST);
    setActiveRow(B00000000);
    activeRow = activeRow >> 1;
    continue;
  }
}
void handleEnc(int a, int b, int num)
{
  byte newState = (a << 1) | b;
  int index = num;
  // Determine new state from bitA and B and state table.
  encTransitionStates[index] = ttable[encTransitionStates[index] & B00001111][newState];
  byte maskedState = encTransitionStates[index] & B00110000;
  switch (maskedState)
  {
  case DIR_CW:
    enc[index]++;
    break;

  case DIR_CCW:
    enc[index]--;
    break;
  default:
    break;
  }
}

void regToData()
{
  static int encAB[9][2];
  encAB[0][0] = bitRead(regsCtrl[1], 5);
  encAB[0][1] = bitRead(regsCtrl[1], 4);
  encAB[1][0] = bitRead(regsCtrl[1], 7);
  encAB[1][1] = bitRead(regsCtrl[1], 6);
  encAB[2][0] = bitRead(regsCtrl[0], 6);
  encAB[2][1] = bitRead(regsCtrl[0], 7);
  encAB[3][0] = bitRead(regsCtrl[0], 4);
  encAB[3][1] = bitRead(regsCtrl[0], 5);
  encAB[4][0] = bitRead(regsCtrl[2], 2);
  encAB[4][1] = bitRead(regsCtrl[2], 3);
  encAB[5][0] = bitRead(regsCtrl[2], 1);
  encAB[5][1] = bitRead(regsCtrl[2], 0);
  encAB[6][0] = bitRead(regsCtrl[3], 7);
  encAB[6][1] = bitRead(regsCtrl[3], 6);
  encAB[7][0] = bitRead(regsCtrl[4], 5);
  encAB[7][1] = bitRead(regsCtrl[4], 4);
  encAB[8][0] = bitRead(regsCtrl[4], 7);
  encAB[8][1] = bitRead(regsCtrl[4], 6);
  for (int i{0}; i < 9; i++)
    handleEnc(encAB[i][0], encAB[i][1], 0);

  bitWrite(ctrl[0], 0, bitRead(regsCtrl[1], 2));
  bitWrite(ctrl[0], 1, bitRead(regsCtrl[1], 0));
  bitWrite(ctrl[0], 2, bitRead(regsCtrl[0], 0));
  bitWrite(ctrl[0], 3, bitRead(regsCtrl[0], 1));
  bitWrite(ctrl[0], 4, bitRead(regsCtrl[2], 5));
  bitWrite(ctrl[0], 5, bitRead(regsCtrl[2], 4));
  bitWrite(ctrl[0], 6, bitRead(regsCtrl[3], 0));
  bitWrite(ctrl[0], 7, bitRead(regsCtrl[3], 1));

  bitWrite(ctrl[1], 8, bitRead(regsCtrl[3], 2));
  bitWrite(ctrl[1], 9, bitRead(regsCtrl[4], 1));
  bitWrite(ctrl[1], 10, bitRead(regsCtrl[3], 4));
  bitWrite(ctrl[1], 11, bitRead(regsCtrl[3], 5));
  bitWrite(ctrl[1], 12, bitRead(regsCtrl[4], 0));

  encBtn[0] = bitRead(regsCtrl[1], 3);
  encBtn[1] = bitRead(regsCtrl[1], 1);
  encBtn[2] = bitRead(regsCtrl[0], 3);
  encBtn[3] = bitRead(regsCtrl[0], 2);
  encBtn[4] = bitRead(regsCtrl[2], 6);
  encBtn[5] = bitRead(regsCtrl[2], 7);
  encBtn[6] = bitRead(regsCtrl[3], 3);
  encBtn[7] = bitRead(regsCtrl[4], 2);
  encBtn[8] = bitRead(regsCtrl[4], 3);

  for (int i{0}; i < 4; i++)
  {
    matrix[i] = regsMatrix[i];
    // reverse first 4 bits (because of wiring)
    bitWrite(matrix[i], 0, bitRead(regsMatrix[i], 3));
    bitWrite(matrix[i], 1, bitRead(regsMatrix[i], 2));
    bitWrite(matrix[i], 2, bitRead(regsMatrix[i], 1));
    bitWrite(matrix[i], 3, bitRead(regsMatrix[i], 0));
  }
  // correct row order (because of wiring)
  uint8_t row4 = matrix[0];
  matrix[0] = matrix[1];
  matrix[1] = matrix[2];
  matrix[2] = matrix[3];
  matrix[3] = row4;
}
void currentToPrevData()
{
  for (int i{0}; i < 4; i++)
    matrixPrev[i] = matrix[i];
  for (int i{0}; i < 2; i++)
    ctrlPrev[i] = ctrl[i];
  for (int i{0}; i < 2; i++)
    encBtnPrev[i] = encBtn[i];
}
void updateLastChanged()
{
  for (int i{0}; i < 2; i++)
    if (ctrlPrev[i] != ctrl[i])
      ctrlData_lastChangeAt = millis();
  for (int i{0}; i < 2; i++)
    if (encBtnPrev[i] != encBtn[i])
      ctrlData_lastChangeAt = millis();
  for (int i{0}; i < 4; i++)
    if (matrixPrev[i] != matrix[i])
      matrix_lastChangeAt = millis();
}

bool hasFreshDataMatrix()
{
  for (int i{0}; i < 4; i++)
  {
    if (matrixSent[i] != matrix[i])
      return true;
  }
  return false;
}
bool hasFreshDataCtrl()
{
  for (int i{0}; i < 13; i++)
    if (ctrlSent[i] != ctrl[i])
      return true;
  for (int i{0}; i < 9; i++)
    if (encBtnSent[i] != encBtn[i])
      return true;
  for (int i{0}; i < 9; i++)
    if (enc[i] != 0)
      return true;
  return false;
}

void sendCtrl()
{
  for (int i{0}; i < cBytesTransmissionDataCtrl; i++)
  {
    Wire.write(transmissionDataCtrl[i]);
  }
  dataSentCtrl = true;
  digitalWrite(IRQ_CTRL, HIGH);
}
void sendMatrix()
{
  for (int i = 0; i < cBytesTransmissionDataMatrix; i++)
  {
    Wire.write(transmissionDataMatrix[i]);
  }
  dataSentMatrix = true;
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
  if (dataSentCtrl)
  {
    setDataSentCtrl();
    dataSentCtrl = false;
  }
  if (dataSentMatrix)
  {
    setDataSentMatrix();
    dataSentMatrix = false;
  }
  currentToPrevData();
  readRegs();
  regToData();
  updateLastChanged();
  if (matrix_lastChangeAt <= (millis() - 50))
  {
    if (hasFreshDataMatrix())
    {
      prepTransmissionDataMatrix();
      digitalWrite(IRQ_MATRIX, LOW);
    }
  }
  else
    digitalWrite(IRQ_MATRIX, HIGH);

  if (ctrlData_lastChangeAt <= (millis() - 50)) // changes to Enc bypass this check
                                                // Statemachine handles false positives of enc
  {
    if (hasFreshDataCtrl())
    {
      prepTransmissionDataCtrl();
      digitalWrite(IRQ_CTRL, LOW);
    }
  }
  else
    digitalWrite(IRQ_CTRL, HIGH);
}
