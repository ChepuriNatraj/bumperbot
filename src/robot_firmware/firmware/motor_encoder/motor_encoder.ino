// ---------------- Pin definitions ----------------
#define LEFT_PWM 6
#define LEFT_DIR 7
#define RIGHT_PWM 4
#define RIGHT_DIR 5

#define LEFT_ENC_A 3
#define LEFT_ENC_B 19
#define RIGHT_ENC_A 2
#define RIGHT_ENC_B 18

// ---------------- Tunables ----------------
const unsigned long CMD_TIMEOUT_MS = 250;   // stop if no valid line for this long
const unsigned long PUBLISH_PERIOD_MS = 20; // encoder publish rate

const float WHEEL_RADIUS = 0.0585;
const float CPR = 46367.0;
     

volatile long left_ticks = 0;
volatile long right_ticks = 0;

long lastLeftTicks = 0;
long lastRightTicks = 0;

float targetVelL = 0.0;
float targetVelR = 0.0;

float actualVelL = 0.0;
float actualVelR = 0.0;

float Kp = 180.0;
float Ki = 40.0;
float Kd = 0.5;

float integralL = 0;
float integralR = 0;

float prevErrorL = 0;
float prevErrorR = 0;

float pwmL = 0;
float pwmR = 0;

String inputString = "";
unsigned long lastPublish = 0;
unsigned long lastCommand = 0;

void leftISR()
{
  if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B))
    left_ticks++;
  else
    left_ticks--;
}

void rightISR()
{
  if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B))
    right_ticks--;
  else
    right_ticks++;
}

void setMotor(int pwmPin, int dirPin, int pwm)
{
  pwm = constrain(pwm,-100,100);
  if (pwm >= 0)
  {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, constrain(abs(pwm),0,100));
  }
  else
  {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, constrain(abs(pwm),0,100));
  }
}

void stopMotors()
{
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
  digitalWrite(LEFT_DIR, LOW);
  digitalWrite(RIGHT_DIR, LOW);
}

// Returns true if the line was a valid, recognized command
bool processCommand(String cmd)
{
  cmd.trim();

  if (cmd.length() == 0)
    return false;

  if (cmd == "STOP")
  {
    targetVelL = 0;
    targetVelR = 0;

    integralL = 0;
    integralR = 0;

    prevErrorL = 0;
    prevErrorR = 0;

    pwmL = 0;
    pwmR = 0;

    stopMotors();
    Serial.println("EMERGENCY_STOP");
    return true;
  }

  int comma = cmd.indexOf(',');
  if (comma < 0)
    return false; // malformed - do NOT reset watchdog on garbage

  String leftStr = cmd.substring(0, comma);
  String rightStr = cmd.substring(comma + 1);

  // Basic sanity check: both parts should look numeric (optional '-' then digits)
  if (leftStr.length() == 0 || rightStr.length() == 0)
    return false;

  targetVelL = leftStr.toFloat();
  targetVelR = rightStr.toFloat();

  

  return true;
}
float runPID(
    float target,
    float actual,
    float &integral,
    float &previous,
    float pwm,
    float dt)
{
    float error = target - actual;

    integral += error * dt;
    integral = constrain(integral, -0.5, 0.5);

    float derivative = (error - previous) / dt;

    previous = error;

    float output =
    Kp * error +
    Ki * integral +
    Kd * derivative;

    pwm = output;

    return constrain(pwm,-100,100);
}


void setup()
{
  Serial.begin(115200);

  left_ticks = 0;
  right_ticks = 0;
  

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightISR, CHANGE);
  noInterrupts();
  lastLeftTicks = left_ticks;
  lastRightTicks = right_ticks;
  interrupts();

  stopMotors();
  lastCommand = millis();
  lastPublish = millis();
}

void loop()
{
  // ---- Read all available serial input, line by line ----
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n')
    {
      if (processCommand(inputString))
        lastCommand = millis(); // only reset watchdog on a VALID command
      inputString = "";
    }
    else if (c != '\r')
    {
      inputString += c;
    }
  }

  // ---- Watchdog: no valid command recently => stop instinctively ----
  if (millis() - lastCommand > CMD_TIMEOUT_MS)
  {
    targetVelL = 0;
    targetVelR = 0;

    integralL = 0;
    integralR = 0;

    prevErrorL = 0;
    prevErrorR = 0;

    pwmL = 0;
    pwmR = 0;

    stopMotors();
    // NOTE: do not reset lastCommand here - otherwise if the serial link
    // is dead, this branch would only fire once every CMD_TIMEOUT_MS
    // instead of continuously holding the motors stopped, which is fine
    // either way since stopMotors() is idempotent, but we don't want to
    // rely on it - so lastCommand is intentionally left alone.
  }

  // ---- Publish encoder ticks at a fixed rate ----
  if (millis() - lastPublish > PUBLISH_PERIOD_MS)
  {
    noInterrupts();
    long lt = left_ticks;
    long rt = right_ticks;
    interrupts();
    long dL = lt - lastLeftTicks;
    long dR = rt - lastRightTicks;

    lastLeftTicks = lt;
    lastRightTicks = rt;

    float distancePerTick =
    2.0 * PI * WHEEL_RADIUS / CPR;

    unsigned long now = millis();
    float dt = (now - lastPublish) / 1000.0f;

    if (dt <= 0.0f)
        return;

    actualVelL = dL * distancePerTick / dt;
    actualVelR = dR * distancePerTick / dt;
  
    if(abs(targetVelL)<0.005){
        targetVelL=0;
        integralL=0;
        prevErrorL=0;
    }

    if(abs(targetVelR)<0.005){
        targetVelR=0;
        integralR=0;
        prevErrorR=0;
    }
    pwmL = runPID(
        targetVelL,
        actualVelL,
        integralL,
        prevErrorL,
        pwmL,
        dt);

    pwmR = runPID(
        targetVelR,
        actualVelR,
        integralR,
        prevErrorR,
        pwmR,
        dt);
    
    setMotor(
          LEFT_PWM,
          LEFT_DIR,
          int(pwmL));

    setMotor(
          RIGHT_PWM,
          RIGHT_DIR,
          int(pwmR));

    

    Serial.print("E,");
    Serial.print(lt);
    Serial.print(",");
    Serial.println(rt);
    
    lastPublish = now ;
  }
}