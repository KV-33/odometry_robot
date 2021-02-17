#define BAUDRATE 115200
#define MOTOR_COUNT 2
#define ENCODER_COUNT 2
#define ENC_A 0
#define ENC_B 1
#define LEFT 0
#define RIGHT 1

#define ENC_LEFT_A_PIN PA9
#define ENC_LEFT_B_PIN PA10
#define ENC_RIGHT_A_PIN PA9
#define ENC_RIGHT_B_PIN PA10

long enc_counter[MOTOR_COUNT] = {0,0};
bool sign[MOTOR_COUNT][ENCODER_COUNT];

void setup() {
  Serial.begin(BAUDRATE);
  
  pinMode(ENC_LEFT_A_PIN, INPUT);
  pinMode(ENC_LEFT_B_PIN, INPUT);
  pinMode(ENC_RIGHT_A_PIN, INPUT);
  pinMode(ENC_RIGHT_B_PIN, INPUT);

  sign[LEFT][ENC_A] = digitalRead(ENC_LEFT_A_PIN);
  sign[LEFT][ENC_B] = digitalRead(ENC_LEFT_B_PIN);
  sign[RIGHT][ENC_A] = digitalRead(ENC_RIGHT_A_PIN);
  sign[RIGHT][ENC_B] = digitalRead(ENC_RIGHT_B_PIN);

  attachInterrupt(ENC_LEFT_A_PIN, left_enc_A, CHANGE);
  attachInterrupt(ENC_LEFT_B_PIN, left_enc_B, CHANGE);
  attachInterrupt(ENC_RIGHT_A_PIN, right_enc_A, CHANGE);
  attachInterrupt(ENC_RIGHT_B_PIN, right_enc_B, CHANGE);
}

void left_enc_A()
{
  sign[LEFT][ENC_A] = digitalRead(ENC_LEFT_A_PIN);
  if (sign[LEFT][ENC_B] == sign[LEFT][ENC_A])
  {
    enc_counter[LEFT]++;
  }
  else
  {
    enc_counter[LEFT]--;
  }
}

void left_enc_B()
{
  sign[LEFT][ENC_B] = digitalRead(ENC_LEFT_B_PIN);
  if (sign[LEFT][ENC_B] != sign[LEFT][ENC_A])
  {
    enc_counter[LEFT]++;
  }
  else
  {
    enc_counter[LEFT]--;
  }
}

void right_enc_A()
{
  sign[RIGHT][ENC_A] = digitalRead(ENC_RIGHT_A_PIN);
  if (sign[RIGHT][ENC_B] == sign[RIGHT][ENC_A])
  {
    enc_counter[RIGHT]++;
  }
  else
  {
    enc_counter[RIGHT]--;
  }
}

void right_enc_B()
{
  sign[RIGHT][ENC_B] = digitalRead(ENC_RIGHT_B_PIN);
  if (sign[RIGHT][ENC_B] != sign[RIGHT][ENC_A])
  {
    enc_counter[RIGHT]++;
  }
  else
  {
    enc_counter[RIGHT]--;
  }
}

void loop() 
{
  Serial.println(enc_counter[RIGHT]);
}
