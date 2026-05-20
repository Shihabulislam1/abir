// --- Left Motor (BTS7960) ---
#define RPWM_L 5  // Forward PWM (Must be a PWM pin)
#define LPWM_L 6  // Reverse PWM (Must be a PWM pin)
#define R_EN_L 2  // Right Enable
#define L_EN_L 3  // Left Enable

// --- Right Motor (BTS7960) ---
#define RPWM_R 10  // Forward PWM (Must be a PWM pin)
#define LPWM_R 11 // Reverse PWM (Must be a PWM pin)
#define R_EN_R 12 // Right Enable
#define L_EN_R 13 // Left Enable

String inputString = "";
boolean stringComplete = false;

void setup() {
  Serial.begin(115200);
  
  // Set all motor control pins as outputs
  pinMode(RPWM_L, OUTPUT);
  pinMode(LPWM_L, OUTPUT);
  pinMode(R_EN_L, OUTPUT);
  pinMode(L_EN_L, OUTPUT);
  
  pinMode(RPWM_R, OUTPUT);
  pinMode(LPWM_R, OUTPUT);
  pinMode(R_EN_R, OUTPUT);
  pinMode(L_EN_R, OUTPUT);

  // Enable the motor drivers (Turn them ON)
  digitalWrite(R_EN_L, HIGH);
  digitalWrite(L_EN_L, HIGH);
  digitalWrite(R_EN_R, HIGH);
  digitalWrite(L_EN_R, HIGH);
  
  inputString.reserve(50);
}

void loop() {
  // If the Python node sent a complete string ending with '\n'
  if (stringComplete) {
    parseCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

// Interrupt triggered when serial data arrives from the Raspberry Pi
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void parseCommand(String command) {
  // Verify format: <L:xxx,R:yyy>
  if (command.startsWith("<L:") && command.indexOf(",R:") > 0 && command.endsWith(">\n")) {
    
    int commaIndex = command.indexOf(',');
    int rightStartIndex = command.indexOf(",R:") + 3;
    int endIndex = command.indexOf('>');

    // Extract numbers
    int leftPWM = command.substring(3, commaIndex).toInt();
    int rightPWM = command.substring(rightStartIndex, endIndex).toInt();

    driveMotors(leftPWM, rightPWM);
  }
}

void driveMotors(int leftSpeed, int rightSpeed) {
  // --- Left Motor Control ---
  if (leftSpeed >= 0) {
    // Drive Forward: Apply PWM to RPWM, 0 to LPWM
    analogWrite(LPWM_L, 0); 
    analogWrite(RPWM_L, constrain(leftSpeed, 0, 255));
  } else {
    // Drive Reverse: Apply PWM to LPWM, 0 to RPWM
    analogWrite(RPWM_L, 0);
    analogWrite(LPWM_L, constrain(-leftSpeed, 0, 255)); // Convert negative to positive
  }

  // --- Right Motor Control ---
  if (rightSpeed >= 0) {
    // Drive Forward: Apply PWM to RPWM, 0 to LPWM
    analogWrite(LPWM_R, 0);
    analogWrite(RPWM_R, constrain(rightSpeed, 0, 255));
  } else {
    // Drive Reverse: Apply PWM to LPWM, 0 to RPWM
    analogWrite(RPWM_R, 0);
    analogWrite(LPWM_R, constrain(-rightSpeed, 0, 255)); // Convert negative to positive
  }
}