#include <stdio.h>
#include <Wire.h>                    // Thư viện I2c của Arduino, do PCA9685 sử dụng chuẩn giao tiếp I2c nên thư viện này bắt buộc phải khai báo 
#include <Adafruit_PWMServoDriver.h> // Thư viện PCA9685
#include <PS2X_lib.h>                // Thư viện PS2X để giao tiếp với controller PS2
#include <Adafruit_Sensor.h>        // thư viện Adafruit_sensor
#include <Adafruit_TCS34725.h>      // Thư viện cảm biến màu TCS34725

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // Tạo đối tượng PCA9685 để điều khiển servo qua giao tiếp I2c
PS2X ps2x;      // Tạo đối tượng PS2X để giao tiếp với controller PS2

/* Sử dụng cảm với thời gian cụ thể và Gain cụ thể
 * integration time (Thời gian lấy mẫu màu của cảm biến): 2.4ms, 24ms, 50ms, 101ms, 154ms, 600ms
 * Gain (Độ lợi kiểm soát độ nhạy màu của biến): 1x, 4x, 16x, 60x
 */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_4X);

// Khai báo các chân kết nối với controller PS2
#define PS2_DAT 12   // Chân dữ liệu của controller PS2
#define PS2_CMD 13   // Chân lệnh của controller PS2
#define PS2_SEL 15   // Chân chọn của controller PS2
#define PS2_CLK 14   // Chân đồng hồ của controller PS2 

// Các giá trị PWM cho động cơ
#define MIN_PWM 0      // Giá trị PWM tối thiểu của động cơ
#define MAX_PWM 4096   // Giá trị PWM tối đa của động cơ

// Định nghĩa các kênh servo trên PCA9685
#define Servo_180_1 2  // Kênh điều khiển Servo_180_1
#define Servo_180_2 3  // Kênh điều khiển Servo_180_2
#define Servo_360_1 4  // Kênh điều khiển servo_360_1
#define Servo_360_2 5  // Kênh điều khiển servo_360_2

// Các giá trị PWM cho servo
#define MIN_SERVO 205   // Giá trị PWM tối thiểu của servo 
#define MAX_SERVO 410   // Giá trị PWM tối đa của servo 

unsigned long timedelay = 0;  // gán biến thời gian trễ để đá servo_180_1 lọc bóng bằng 0
 
void setup() 
{
  Serial.begin(115200);  // Khởi tạo giao tiếp Serial với tốc độ 115200
  initMotors();          // chương trình con các thiết lập ban đầu
  setupPS2controller(); // khởi tạo controller PS2
  //setupColorSensor();   // khởi tạo cảm biến màu
  Serial.println("Done setup!");  // in thông báo khởi động thành công
}

void loop() 
{
  ps2x.read_gamepad(false, false);      // gọi hàm để đọc tay điều khiển
  chuyendong();   // chương trình con điều khiển cho robot di chuyển
  thubong();    // chương trình con điều khiển robot thu thập bóng trắng, đen
  locbong();    // chương trình con lọc bóng
  banbongtrang();    // chương trình con điều khiển bắn bóng trắng
  trutbongden();    // chương trình con điều khiển trút bóng đen
}

void initMotors() 
{
  Wire.begin();                         // khởi tạo kết nối và tham gia vào mạng I2C
  pwm.begin();                          // Khởi tạo PCA9685
  pwm.setOscillatorFrequency(27000000); // Cài đặt tần số giao động 
  pwm.setPWMFreq(50);                   // cài đặt tần số PWM là 50 HZ
  Wire.setClock(400000);                // cài đặt tốc độ giao tiếp i2c ở tốc độ cao nhất(400 Mhz)
  setPWMMotors(0, 0, 0, 0);             // các động cơ đứng yên, robot không chuyển động 
}

void setupPS2controller() 
{
  if (ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true)) {    // khởi động tay cầm ps2
    ps2x.readType();
    Serial.println("PS2 đã hoạt động."); // In thông báo nếu controller được khởi tạo thành công
  } else {
    Serial.println("PS2 không kết nối, hãy kiểm tra lại kết nối."); // In thông báo lỗi nếu không khởi tạo được controller
  }    
}

void setupColorSensor() {
  if (tcs.begin()) {                         // khởi động cảm biến màu
    Serial.println("Cảm biến đã hoạt động"); // In thông báo nếu cảm biến được khởi tạo thành công
  } else {
    Serial.println("Cảm biến không kết nối, hãy kiểm tra lại kết nối."); // In thông báo lỗi nếu không khởi tạo được cảm biến
  }
}

bool chuyendong() 
{
  int bien = 1600;      //gán giá trị cho bien
  float bienn = 1600.0; //gán giá trị cho bienn
  //gán giá trị cho trục x, y của joystick trái:
  int nJoyX = ps2x.Analog(PSS_LX);  
  int nJoyY = ps2x.Analog(PSS_LY);  
  
  //đổi từ giá trị tọa độ mặc định sang giá trị tọa độ mới cho joystick trái:
  nJoyX = map(nJoyX, 0, 255, -bien, bien);
  nJoyY = map(nJoyY, 0, 255, bien, -bien);
 
  int nMotMixL;  // khai báo biến điều khiển tốc độ động cơ trái
  int nMotMixR;  // khai báo biến điều khiển tốc độ động cơ phải
  
  float fPivYLimit = bienn; // gán giá trị cho fPivYLimit

  float nMotPremixL;    // Khai báo biến phụ tốc độ động cơ trái
  float nMotPremixR;    // Khai báo biến phụ tốc độ động cơ phải
  
  // Khai báo biến
  int nPivSpeed;      
  float fPivScale;
     
  if (nJoyY >= 0) {
    // Tiến về phía trước, gán giá trị cho biến phụ tốc độ động cơ trái và động cơ phải để khi chuyển hướng thì tốc độ 2 bánh xe khác nhau
    nMotPremixL = (nJoyX >= 0) ? bienn : (bienn + nJoyX);  
    nMotPremixR = (nJoyX >= 0) ? (bienn - nJoyX) : bienn;  
  } 
  else {
    // lùi về phía sau,  gán giá trị cho biến phụ tốc độ động cơ trái và động cơ phải để khi chuyển hướng thì tốc độ 2 bánh xe khác nhau
    nMotPremixL = (nJoyX >= 0) ? (bienn - nJoyX) : bienn;  
    nMotPremixR = (nJoyX >= 0) ? bienn : (bienn + nJoyX);  
  }

  //Gán giá trị cho biến phụ tốc độ động cơ trái và phải phụ thuộc vào tọa độ trục Y của joystick trái (tránh robot quay tròn hay giật cục tại chỗ)
  nMotPremixL = nMotPremixL * nJoyY / bienn;           
  nMotPremixR = nMotPremixR * nJoyY / bienn;   

  nPivSpeed = nJoyX;  // gán giá trị cho nPivSpeed
  fPivScale = (abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyY) / fPivYLimit);  // gán giá trị cho fPivScale

  nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * (nPivSpeed);   // gán giá trị cho biến điều khiển tốc độ động cơ trái
  nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);  // gán giá trị cho biến điều khiển tốc độ động cơ phải
  
  int c1 = 0, c2 = 0, c3 = 0, c4 = 0; // gán giá trị cho c1, c2, c3, c4

  if (nMotMixR > 50) {    // nếu nMotMixR > 50 thì gán c3 = nMotMixR 
    c3 = nMotMixR;
  } 
  else if (nMotMixR < -50) {    // hoặc nếu nMotMixR < -50 thì gán c4 = giá trị tuyệt đối của nMotMixR    
    c4 = abs(nMotMixR);
  }

  if (nMotMixL > 50) {        // nếu nMotMixL > 50 thì gán c1 = nMotMixL        
    c1 = nMotMixL;
  } 
  else if (nMotMixL < -50) {    // hoặc nếu nMotMixL < -50 thì gán c2 = giá trị tuyệt đối của nMotMixL  
    c2 = abs(nMotMixL);
  }

  setPWMMotors(c1, c2, c3, c4);  // gọi chương trình con điều khiển động cơ  
  delay(50);  // chờ 50 ms 
  return 1;   // trả về giá trị true
}

void setPWMMotors(int c1, int c2, int c3, int c4)  // chương trình con điều khiển động cơ 
{
  // cài đặt độ rộng xung của kênh 8, 9, 10, 11 phụ thuộc theo c1, c2, c3, c4 để điều khiển động cơ
  pwm.setPWM(8, c1, MAX_PWM - c1);
  pwm.setPWM(9, c2, MAX_PWM - c2);
  pwm.setPWM(10, c3, MAX_PWM - c3);
  pwm.setPWM(11, c4, MAX_PWM - c4);
}
  
void thubong()
{
  if (ps2x.Button(PSB_PAD_UP)) {
    // ấn nút PSB_PAD_UP để điều khiển động cơ thu bóng quay tốc độ 50% = 4096/2, chiều quay thuận
    pwm.setPWM(12, 0, 2048); // chân số 12 set chiều dương là PWM 50%
    pwm.setPWM(13, 0, 0);    // chân số 13 set chiều âm 
  } 
   
  if (ps2x.Button(PSB_PAD_DOWN)) {
    // ấn nút PSB_PAD_DOWN để điều khiển động cơ thu bóng ngừng quay
    pwm.setPWM(12, 0, 0);
    pwm.setPWM(13, 0, 0);
  }
}

void locbong() {
  uint16_t r, g, b, c;    
  tcs.getRawData(&r, &g, &b, &c); // cảm biến đọc các giá trị màu
  if (r<50 && g<50 && b<50) {     // nếu thu được màu đen
    int a = 0;                    // gán biến a = 0 của vòng lặp while
    timedelay = millis();         // Ghi lại thời điểm bắt đầu chờ
    while (a == 0) {              // bắt đầu vòng lặp
      if (millis() - timedelay >= 300) {    // nếu thời gian trễ lớn hơn hoặc bằng 300ms
        pwm.setPWM(Servo_180_1, 0, MAX_SERVO);  // Quay Servo_180_1 sang trái 90 độ
        unsigned long time2 = millis();   // gán biến time2 để ghi lại thời điểm bắt đầu
        int b = 0;                // gán biến b = 0 cho vòng lặp
        while (b == 0) {          // bắt đầu vòng lặp
          if (millis() - time2 >= 100) {     // nếu thời gian trễ lớn hơn hoặc bằng 100ms
          a = 1;                  // gán a = 1 để thoát khỏi vòng lặp
          b = 1;                  // gán b = 1 để thoát khỏi vòng lặp
          }
        }
      }
    } 
  }  
  else 
  {
    pwm.setPWM(Servo_180_1, 0, MIN_SERVO); // giữ servo_180_1 đứng im nếu không nhận màu đen
  } 
}

void banbongtrang() 
{
  if (ps2x.ButtonPressed(PSB_SQUARE)){        // nếu ấn nút hình vuông
    pwm.setPWM(12, 0, 0);                     // ngắt động cơ thu bóng tập trung năng lượng để bắn bóng
    pwm.setPWM(13, 0, 0);                     // ngắt động cơ thu bóng tập trung năng lượng để bắn bóng
    pwm.setPWM(Servo_360_1, 0, MIN_SERVO);    // Quay Servo_360_1 sang trái 90 độ đóng công tắc hành trình điều khiển động cơ bắn bóng
    delay(500);                               // chờ 300ms
    pwm.setPWM(Servo_360_1, 0, 0);            // cho servo_360_1 ngừng quay        
  } 
  if (ps2x.ButtonReleased(PSB_SQUARE)){      // Nếu nhả không ấn nút hình vuông
    pwm.setPWM(Servo_360_1, 0, MAX_SERVO);    // Quay servo_360_1 trở lại vị trí ban đầu nhả mở công tắc hành trình
    delay(500);                              // chờ 300ms
    pwm.setPWM(Servo_360_1, 0, 0);          // cho servo_360_1 ngừng quay
    pwm.setPWM(12, 0, 2048);                   // cho động cơ thu bóng quay
    pwm.setPWM(13, 0, 0);                     // cho động cơ thu bóng quay
  }

  if (ps2x.ButtonPressed(PSB_R1)) {           // nếu nút R1 được ấn
    pwm.setPWM(Servo_360_2, 0, MIN_SERVO);    // Quay Servo_360_2 sang trái 90 độ mở cửa nạp bắn bóng
    delay(500);                               // chờ 300ms
    pwm.setPWM(Servo_360_2, 0, 0);            // cho servo_360_2 ngừng quay
  } 
  if (ps2x.ButtonReleased(PSB_R1)) {          // nếu nhả nút ấn R1
    pwm.setPWM(Servo_360_2, 0, MAX_SERVO);    // Quay Servo_360_2 trở lại vị trí ban đầu nhả mở công tắc hành trình
    delay(500);                              // chờ 300ms
    pwm.setPWM(Servo_360_2, 0, 0);          // cho servo_360_2 ngừng quay
  }
}

void trutbongden() {
  if (ps2x.ButtonPressed(PSB_TRIANGLE)) {          // nếu ấn nút tam giác
    pwm.setPWM(Servo_180_2, 0, MIN_SERVO); // Quay Servo_180_2 sang trái 90 độ
  }
  if (ps2x.ButtonReleased(PSB_TRIANGLE)) {        // nếu ấn nút tam giác được thả ra
    pwm.setPWM(Servo_180_2, 0, MAX_SERVO);  // Quay Servo_180_2 trở lại vị trí ban đầu
  }
}
