const int motor_directionPin2 = 9;
const int motor_directionPin1 = 10;
const int motor_currentPin = A0;

const int Ith = int(1024.0*0.4/5);
const int timeout = 3000;

const int PACKET_LENGTH = 3;
char incoming_data[PACKET_LENGTH] = {0,0,0};
int n = 0;
int t = 0;
int I = 0;
int tflag = 0;

void setup() {
  pinMode(motor_directionPin1, OUTPUT);
  pinMode(motor_directionPin2, OUTPUT);
  pinMode(motor_currentPin, INPUT);

  digitalWrite(motor_directionPin1,LOW);
  digitalWrite(motor_directionPin2,LOW);
  
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("ser_ee");
}

void loop() {
  while (Serial.available() > 0) {
    char msg = Serial.read();
    Serial.println(msg);
    if(n < PACKET_LENGTH) {
      incoming_data[n] = msg;
      n++;
    } 
    if (msg == '\n') {
      n = 0;
      Serial.flush();
      if (incoming_data[0] == 'G') {
        // grab command
        Serial.println(incoming_data[1]);
        if(incoming_data[1] == '0') {
          digitalWrite(motor_directionPin1,LOW);
          digitalWrite(motor_directionPin2,LOW);
          t = 0;
          tflag = 0;
        }
        if(incoming_data[1] == '1') {
          Serial.println("Grabbing... press r to release");
          digitalWrite(motor_directionPin1,HIGH);
          digitalWrite(motor_directionPin2,LOW);
          t = 0;
          tflag = 1;
        }
        // release command
        if(incoming_data[1] == '2') {
          Serial.println("Releasing... press g to grab");
          digitalWrite(motor_directionPin1,LOW);
          digitalWrite(motor_directionPin2,HIGH);
          t = 0;
          tflag = 1;
        }
        // short release command
        if(incoming_data[1] == '3') {
          Serial.println("Releasing... press g to grab");
          digitalWrite(motor_directionPin1,LOW);
          digitalWrite(motor_directionPin2,HIGH);
          t = 1500;
          tflag = 1;
        }
        delay(500);
        Serial.print("done\r\n");
      }
      if (incoming_data[0] == 'R') {
        Serial.println("no data");
      }
    }
  }
  I = analogRead(motor_currentPin);
  //Serial.println(tflag);
  if(I>Ith || t>timeout){
    Serial.println(I);
    Serial.println(tflag);
    Serial.println(t);
    digitalWrite(motor_directionPin1,LOW);
    digitalWrite(motor_directionPin2,LOW);
    tflag = 0;
    t = 0;
    Serial.print("done\r\n");
  }
  if(tflag != 0){
    //Serial.println(I);
    //Serial.println(t);
    t++;
  }
  delay(5);
}
