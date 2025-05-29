# LineFollower
Το έργο μας έχει ως στόχο την δημιουργία ενός line follower robot για το μαθημα Ενσωματωμένα Συστήματα, το οποίο θα φέρει εις περας τις 3 πιστες, καθως και να τις εκτελεσει στον βελτιστο χρονο. Συγκεκριμένα η πρωτη πιστα ειναι να ακολουθησει μια γραμμη. Η δευτερη πιστα να κανει μια μικρη πισα με μερικες στροφες και να σταματησει στο τελος της. η τελευταια πιστα ειναι μια συνεχης πιστα στην οποια πρεπει να κανει 3 γυρους συνεχομενους και να τους κανει στον λιγοτερο δυνατο χρονο.

## Τι θα χρειαστούμε για να φτιαξουμε το ρομποτ:
  1) 1x ESP32  
  2) 1x PCB board     
  4) 2x Step down 
  6) 1x 8 IR sensor array 
  7) 1x 12v battery
  8) 2x 12v 1000rpm dc motors
  9) 1x DRV8833 motor driver
  10) 2x rubber wheels
  11) 1x steel ball wheel
  12) Μερικα καλωδια για τις συνδεσεις
  13) 1x Switch για να δινουμε ρευμα στο ρομποτ

# Εισαγωγή βιβλιοθήκης
    #include <QTRSensors.h>

# Δηλώσεις μεταβλητών αισθητήρων
    QTRSensors qtr;
    const uint8_t SensorCount = 8;
    uint16_t sensorValues[SensorCount];
    float position = 0;
    float positionSum = 0;
    int sensorDetectCount = 0;
    int lastSensorDetect = -1;
    
    const float sensorPositions[SensorCount] = {-1.0, -0.714, -0.429, -0.143, 0.143, 0.429, 0.714, 1.0};

# Ορισμός pins για μοτέρ
    const int RIGHT_MOTOR_IN1 = 2;
    const int RIGHT_MOTOR_IN2 = 4;
    const int LEFT_MOTOR_IN3  = 16;
    const int LEFT_MOTOR_IN4  = 17;

# Μεταβλητές PID ελέγχου
    // PID variables
    float Kp = 280;   
    float Ki = 0;
    float Kd = 20;

    float error = 0;
    float lastError = 0;
    float integral = 0;

# Μεταβλητή-flag για να σταματάει οταν βλέπει γραμμή
    bool foundLine=false;

# Αρχικοποίηση των pins για αισθητήρες και μοτέρ
    void setup() {
      // QTR configuration
      qtr.setTypeAnalog();
      qtr.setSensorPins((const uint8_t[]){34, 35, 32, 33, 25, 26, 27, 14}, SensorCount);
      qtr.setEmitterPin(2);
    
      // Motor pins
      pinMode(RIGHT_MOTOR_IN1, OUTPUT);
      pinMode(RIGHT_MOTOR_IN2, OUTPUT);
      pinMode(LEFT_MOTOR_IN3, OUTPUT);
      pinMode(LEFT_MOTOR_IN4, OUTPUT);
    
      Serial.begin(115200);
    }
# Αναγνωση τιμών αισθητήρων
    void loop() {
      qtr.read(sensorValues);
      Serial.print("Sensors: ");
      sensorDetectCount = 0;
      positionSum = 0;
    
      for (uint8_t i = 0; i < SensorCount; i++) {
        int val = sensorValues[i];
        int threshold = 4000;

    if (val > threshold) {
      Serial.print("#");
      positionSum += sensorPositions[i];
      sensorDetectCount += 1;
      lastSensorDetect = i;
    } else {
      Serial.print("_");
    }
    }
# Υπολογισμός της θεσης των ασιθητήρων απο -1 μεχρι 1
      if (sensorDetectCount > 0) {
        position = positionSum / sensorDetectCount;
      } else {
        position = sensorPositions[lastSensorDetect]; // ή ενα fallback
      }

# Prints για debugging
      Serial.print(" | Position: ");
      Serial.print(position);
      Serial.print(" | sensorDetectCount: ");
      Serial.print(sensorDetectCount);
      Serial.print(" | lastSensorDetect: ");
      Serial.println(lastSensorDetect);
# Υπολογισμός error, integral και derivative για την διορθωση που κανει το ρομποτ    
      // ----- PID Control -----
      error = position; // αν το ρομπότ στρίβει λάθος, βάλε: error = -position;
      integral += error;
      float derivative = error - lastError;
      float correction = Kp * error + Ki * integral + Kd * derivative;
      lastError = error;
    
    
# Αναθεση τιμων στα μοτερ    
      // Ταχύτητα βάσης
      int baseSpeed = 80;
      int leftSpeed = baseSpeed + correction;
      int rightSpeed = baseSpeed - correction;
# Περιορισμός τιμών που μπορούν να πάρουν τα μοτέρ    
      // Περιορισμός PWM
      leftSpeed = constrain(leftSpeed, 0, 100);
      rightSpeed = constrain(rightSpeed, 0, 100);
# Αμα εντοπιζουν 6+ αισθητηρες γραμμή σταματάει    
      //σταματαει οταν βλεπει γραμμη καθετη
      if (sensorDetectCount>5){
        leftSpeed=0;
        rightSpeed=0;
        foundLine=true;
      }
# Flag για να σταματανε τα μοτερ οταν βλεπει γραμμή    
      if (foundLine){
        leftSpeed=0;
        rightSpeed=0;
      }
 # Ανάθεση τιμών στα μοτέρ     
      // Εφαρμογή στα μοτέρ
      analogWrite(RIGHT_MOTOR_IN1, rightSpeed);
      analogWrite(RIGHT_MOTOR_IN2, 0);
    
      analogWrite(LEFT_MOTOR_IN3, leftSpeed);
      analogWrite(LEFT_MOTOR_IN4, 0);
    }
