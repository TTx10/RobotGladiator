// Định nghĩa chân động cơ 
#define in1 2 
#define in2 3 
#define enA 9 
#define in3 4 
#define in4 5 
#define enB 10 

// Định nghĩa chân cảm biến hồng ngoại (5 cái) 
#define sensor1 A0 
#define sensor2 A1 
#define sensor3 A2 
#define sensor4 A3 
#define sensor5 A4 

// Định nghĩa chân cho mạch Bluetooth HM-10
#define bluetoothRX 7 // Chân TX của HM-10 kết nối với RX của Arduino
#define bluetoothTX 8 // Chân RX của HM-10 kết nối với TX của Arduino

// Định nghĩa chân cho tay gắp
#define servoPin 6 // Chân cho servo tay gắp

// Định nghĩa chân cho tia laser
#define laserPin 11 // Chân cho tia laser

// Biến PID 
float Kp = 1.0;  // Hệ số Proportional 
float Ki = 0.0;  // Hệ số Integral 
float Kd = 0.5;  // Hệ số Derivative 

float previous_error = 0; 
float integral = 0; 
unsigned long lost_line_time = 0;  // Thời gian mất line 
const unsigned long TIMEOUT = 2000; // Thời gian giới hạn mất line 

// Khởi tạo biến cho tay gắp
#include <Servo.h>
Servo grabber; // Đối tượng servo cho tay gắp

// Biến điều khiển bắn banh
bool isShooting = false; // Trạng thái động cơ bắn đang quay
unsigned long lastButtonPress = 0; // Thời gian bấm nút trước đó
const unsigned long shootDuration = 500; // Thời gian động cơ quay để bắn (ms)

void setup() { 
  // Thiết lập chân động cơ là đầu ra 
  pinMode(in1, OUTPUT); 
  pinMode(in2, OUTPUT); 
  pinMode(enA, OUTPUT); 
  pinMode(in3, OUTPUT); 
  pinMode(in4, OUTPUT); 
  pinMode(enB, OUTPUT); 

  // Thiết lập chân cảm biến là đầu vào 
  pinMode(sensor1, INPUT); 
  pinMode(sensor2, INPUT); 
  pinMode(sensor3, INPUT); 
  pinMode(sensor4, INPUT); 
  pinMode(sensor5, INPUT); 

  // Thiết lập chân Bluetooth
  Serial.begin(9600); // Khởi động Serial để debug

  // Thiết lập chân laser
  pinMode(laserPin, OUTPUT); 
  digitalWrite(laserPin, LOW); // Tắt tia laser ban đầu

  // Khởi tạo servo cho tay gắp
  grabber.attach(servoPin);
  grabber.write(0); // Đặt tay gắp ở vị trí ban đầu (mở)

} 

void loop() { 
  // Đọc giá trị cảm biến (5 cảm biến hồng ngoại đơn) 
  int s1 = digitalRead(sensor1); 
  int s2 = digitalRead(sensor2); 
  int s3 = digitalRead(sensor3); 
  int s4 = digitalRead(sensor4); 
  int s5 = digitalRead(sensor5); 

  // Kiểm tra xem robot có tìm thấy line không 
  bool lineDetected = s1 || s2 || s3 || s4 || s5; 

  // Kiểm tra dữ liệu từ Bluetooth
  if (Serial.available()) { 
    char command = Serial.read(); // Đọc lệnh từ Bluetooth
    
    if (command == 'F') { // Tiến
      moveMotors(150, 150);
    } else if (command == 'B') { // Lùi
      moveBackward();
    } else if (command == 'L') { // Rẽ trái
      moveMotors(100, 150); // Tùy chỉnh tốc độ
    } else if (command == 'R') { // Rẽ phải
      moveMotors(150, 100); // Tùy chỉnh tốc độ
    } else if (command == 'S') { // Dừng
      moveMotors(0, 0);
    } else if (command == 'P' && !isShooting) { // Bắn
      shootBall(); // Gọi hàm bắn banh
      isShooting = true; // Đánh dấu là đang bắn
      delay(shootDuration); // Đợi trong thời gian bắn
      stopShooting(); // Dừng bắn
      isShooting = false; // Đánh dấu là không còn bắn nữa
    } else if (command == 'G') { // Kích hoạt tay gắp
      grabber.write(90); // Gắp
      delay(500); // Giữ tay gắp trong 500ms
      grabber.write(0); // Mở tay gắp
    } else if (command == 'L') { // Bật/Tắt tia laser
      digitalWrite(laserPin, !digitalRead(laserPin)); // Chuyển trạng thái
    }
  }

  // Nếu không phát hiện line 
  if (!lineDetected) { 
    if (millis() - lost_line_time > TIMEOUT) { 
      // Nếu đã không phát hiện line trong thời gian timeout, robot lùi lại 
      Serial.println("Mất line, lùi lại..."); 
      moveBackward(); 
      delay(500); // Lùi lại trong 500ms 
      lost_line_time = millis(); // Cập nhật thời gian mất line 
    } else { 
      // Robot vẫn đang tìm line 
      Serial.println("Đang tìm line..."); 
      searchForLine(); // Tìm kiếm line 
    } 
  } else { 
    // Tính toán vị trí vạch (giá trị trung bình trọng số) 
    float error = (s1 * -2) + (s2 * -1) + (s3 * 0) + (s4 * 1) + (s5 * 2); 

    // Tính toán giá trị PID 
    integral += error; 
    float derivative = error - previous_error; 
    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative); 

    // Điều chỉnh tốc độ động cơ dựa trên giá trị PID 
    int base_speed = 150;  // Tốc độ cơ bản của robot 
    int left_speed = base_speed + correction; 
    int right_speed = base_speed - correction; 

    // Giới hạn giá trị tốc độ từ 0 đến 255 
    left_speed = constrain(left_speed, 0, 255); 
    right_speed = constrain(right_speed, 0, 255); 

    // Điều khiển động cơ 
    moveMotors(left_speed, right_speed); 

    // Cập nhật lỗi trước đó cho PID 
    previous_error = error; 

    // Hiển thị giá trị để debug 
    Serial.print("Error: "); 
    Serial.print(error); 
    Serial.print(" Correction: "); 
    Serial.print(correction); 
    Serial.print(" Left Speed: "); 
    Serial.print(left_speed); 
    Serial.print(" Right Speed: "); 
    Serial.println(right_speed); 

    delay(10);  // Giảm tải cho vi xử lý 
  } 
}

// Hàm điều khiển động cơ
void moveMotors(int leftSpeed, int rightSpeed) { 
  if (leftSpeed > 0) { 
    digitalWrite(in1, HIGH); 
    digitalWrite(in2, LOW); 
  } else { 
    digitalWrite(in1, LOW); 
    digitalWrite(in2, HIGH); 
    leftSpeed = -leftSpeed; 
  } 

  if (rightSpeed > 0) { 
    digitalWrite(in3, HIGH); 
    digitalWrite(in4, LOW); 
  } else { 
    digitalWrite(in3, LOW); 
    digitalWrite(in4, HIGH); 
    rightSpeed = -rightSpeed; 
  } 

  analogWrite(enA, leftSpeed); 
  analogWrite(enB, rightSpeed); 
} 

// Hàm lùi lại 
void moveBackward() { 
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH); 
  digitalWrite(in3, LOW); 
  digitalWrite(in4, HIGH); 
  analogWrite(enA, 100); // Tốc độ lùi 
  analogWrite(enB, 100); // Tốc độ lùi 
} 

// Hàm tìm kiếm line 
void searchForLine() { 
  // Quay trái để tìm line 
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH); 
  digitalWrite(in3, HIGH); 
  digitalWrite(in4, LOW); 
  analogWrite(enA, 100); // Tốc độ quay 
  analogWrite(enB, 100); // Tốc độ quay 
} 

// Hàm bắn banh
void shootBall() {
  Serial.println("Bắn banh!");
  // Thực hiện lệnh bắn ở đây (nếu cần)
}

// Hàm dừng bắn
void stopShooting() {
  // Thực hiện lệnh dừng bắn ở đây (nếu cần)
  Serial.println("Dừng bắn!");
}
