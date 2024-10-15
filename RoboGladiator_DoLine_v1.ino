// Định nghĩa chân động cơ
#define in1 2
#define in2 3
#define enA 9
#define in3 4
#define in4 5
#define enB 10

// Định nghĩa chân cảm biến hồng ngoại
#define sensor1 A0
#define sensor2 A1
#define sensor3 A2
#define sensor4 A3
#define sensor5 A4

// Biến PID
float Kp = 1.0;  // Hệ số Proportional
float Ki = 0.0;  // Hệ số Integral
float Kd = 0.5;  // Hệ số Derivative

float previous_error = 0;
float integral = 0;

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

  // Khởi động Serial để debug
  Serial.begin(9600);
}

void loop() {
  // Đọc giá trị cảm biến
  int s1 = digitalRead(sensor1);
  int s2 = digitalRead(sensor2);
  int s3 = digitalRead(sensor3);
  int s4 = digitalRead(sensor4);
  int s5 = digitalRead(sensor5);

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
