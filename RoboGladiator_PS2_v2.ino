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

// Định nghĩa chân PS2
#define PS2_SEL 10 // Chân cho nút chọn PS2
#define PS2_BTN_FIRE 11 // Chân cho nút bắn
#define PS2_BTN_TURBO 12 // Chân cho nút turbo

// Định nghĩa chân cho tay gắp
#define servoPin 6 // Chân cho servo tay gắp

// Định nghĩa chân cho tia laser
#define laserPin 7 // Chân cho tia laser

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
bool shootingEnabled = false; // Trạng thái bắn banh
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

  // Thiết lập chân PS2
  pinMode(PS2_SEL, INPUT); 
  pinMode(PS2_BTN_FIRE, INPUT); 
  pinMode(PS2_BTN_TURBO, INPUT); 

  // Thiết lập chân laser
  pinMode(laserPin, OUTPUT); 
  digitalWrite(laserPin, LOW); // Tắt tia laser ban đầu

  // Khởi tạo servo cho tay gắp
  grabber.attach(servoPin);
  grabber.write(0); // Đặt tay gắp ở vị trí ban đầu (mở)

  // Khởi động Serial để debug 
  Serial.begin(9600); 
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

  // Kiểm tra trạng thái tay cầm PS2
  bool fireButtonPressed = digitalRead(PS2_BTN_FIRE);
  
  if (!lineDetected) { 
    // Nếu không phát hiện line 
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
  
  // Kiểm tra nút bắn
  if (fireButtonPressed && (millis() - lastButtonPress > 200)) { // Kiểm tra để tránh bounce
    shootingEnabled = !shootingEnabled; // Chuyển đổi trạng thái bắn banh
    lastButtonPress = millis(); // Cập nhật thời gian bấm nút
  }

  // Thực hiện bắn banh
  if (shootingEnabled && !isShooting) {
    shootBall(); // Gọi hàm bắn banh
    isShooting = true; // Đánh dấu là đang bắn
    delay(shootDuration); // Đợi trong thời gian bắn
    stopShooting(); // Dừng bắn
    isShooting = false; // Đánh dấu là không còn bắn nữa
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
  analogWrite(enA, 255); // Bắn banh
  analogWrite(enB, 255); // Bắn banh
  Serial.println("Bắn banh!");
}

// Hàm dừng bắn banh
void stopShooting() {
  analogWrite(enA, 0); // Dừng động cơ bắn banh
  analogWrite(enB, 0); // Dừng động cơ bắn banh
  Serial.println("Dừng bắn!");
}
