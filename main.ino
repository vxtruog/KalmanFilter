#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <AFMotor.h>
#include <BasicLinearAlgebra.h>

// Định nghĩa kích thước của ma trận và vector cho bộ lọc Kalman
#define MATRIX_SIZE 2
#define STATE_SIZE 2
using namespace BLA;

//        Sơ đồ vị trí của bốn bánh xe
//          M2                  M1
//       /////////           /////////
//       //  2  //           //  1  //
//       /////////           /////////
//          M3                  M4
//       /////////           /////////
//       //  3  //           //  4  //
//       /////////           /////////

// Khai báo các đối tượng động cơ
AF_DCMotor motor1(1); // Motor 1 kết nối với chân M1 của module L293D
AF_DCMotor motor2(2); // Motor 2 kết nối với chân M2 của module L293D
AF_DCMotor motor3(3); // Motor 3 kết nối với chân M3 của module L293D
AF_DCMotor motor4(4); // Motor 4 kết nối với chân M4 của module L293D
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void sg90_init(){
  // Chân 10 kết nối với động cơ servo, sử dụng Timer/Counter 2 tại chân này
  DDRB |= (1 << DDB4);                                    //đặt chân 10 là output
  // Xoá dữ liệu cũ của Timer/Counter 2
  TCCR2A = 0;
  TCCR2B = 0;
  // Cấu hình Timer/Counter 2
  TCCR2A |= (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);  //Xoá khi so sánh khớp, set khi lên Top, Fast PWM Top = 0xFF
  TCCR2B |= (1 << CS22) | (1 << CS21);                    // Prescaler = 256, T ~ 16us (1 lần đếm)
  OCR2A = 100;                                            // Giá trị đếm tương ứng với góc 0 độ
  TCNT2 = 0;                                              // Khởi động Timer/Counter 2
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void hcsr04_init(){
  //Thiết lập chân tín hiệu cảm biến HC-SR04
  DDRA |= (1 << DDA1);          // Thiết lập chân Trig 23 của cảm biến là output
  DDRA &= ~(1 << DDA3);         // Thiết lập chân Echo 25 của cảm biến là input
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile unsigned int count = 0;
void encoder_init(){
  // Khởi tạo ngắt INT0, tương ứng với chân PD0 (chân số 21)
  DDRD &= ~(1<<PD0);
  PIND |= (1<<PIND0);
  // Cấu hình INT0
  EICRA |= (1<<ISC01)|(0<<ISC00);   //thực thi ngắt khi giá trị từ 1 xuống 0 (Falling)
  EIMSK |= (1<<INT0);               //cho phép ngắt INT0
}
//chương trình ngắt ngoài INT0 để tăng biến đếm
ISR (INT0_vect){
  count++;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile bool timerFlag = false;
void time_init(){
  // Thiết lập Timer/Counter 5 tại chân 46
  // Xoá dữ liệu cũ của Timer/Counter 5
  TCCR5A = 0;
  TCCR5B = 0;
  // Cấu hình Timer/Counter 5
  TCCR5B |= (1 << WGM52);               // Chế độ CTC (Clear Timer on Compare Match)
  TCCR5B |= (1 << CS52) | (1 << CS50);  // Bật chế độ prescaler 1024
  OCR5A = 1562;                         // Giá trị OCR5A tương ứng với thời gian 0.1 giây (prescaler 1024)
  TIMSK5 |= (1 << OCIE5A);              // Cho phép ngắt khi đạt đến giá trị của OCR5A
}
//chương trình ngắt Timer/counter5
ISR(TIMER5_COMPA_vect) {
  timerFlag = true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const uint8_t temperature = 28;
unsigned long speed_sound = (331300 + 606*temperature);  // Tốc độ âm thanh trong không khí ở nhiệt độ cố định, đơn vị mm/s
unsigned long duration = 0;                              // Thời gian cả đi cả về của sóng âm thanh
unsigned long distance = 0;                              // Khoảng cách cả đi cả về của sóng âm thanh
float MEA = 0;                                           // Giá trị đo của cảm biến
//Chương trình đọc giá trị khoảng cách của cảm biến siêu âm HC-SR04
float read_hcsr04(){
  // Phát sóng siêu âm và đo khoảng cách
  PORTA &= ~(1 << PA1);
  _delay_us(2);
  PORTA |= (1 << PA1);
  _delay_us(10);
  PORTA &= ~(1 << PA1);
  duration = pulseIn(25, HIGH, 100000);
  if(duration == 0){
    duration = 280;
  }
  distance = speed_sound*duration;            //mm, khoảng cách cả đi cả về theo đơn vị thời gian (us)
  MEA = float(distance)/20000000;             //cm, khoảng cách từ cảm biến đến vật phát hiện theo đơn vị thời gian (s)
  return MEA;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float speed_car = 0;
float speed_car_pre = 0;
float accelerate_car = 0;
float distance_hcsr04 = 0;
float distance_car = 0;
bool runFlag = false;
void obstacles_detected(){
  //đọc giá trị vận tốc từ module encoder và khoảng cách từ cảm biến siêu âm sau mỗi 0.1 giây
  distance_hcsr04 = read_hcsr04();
  if(timerFlag){
    speed_car = float(count)*1.037*10;  //đơn vị tốc độ cm/s
    accelerate_car = (speed_car - speed_car_pre)*10;
    distance_car = Kalman(distance_hcsr04, speed_car, accelerate_car, 3);
    //đặt lại count và flag
    count = 0;
    speed_car_pre = speed_car;
    timerFlag = false;
  }
  if(distance_car < 30){
    stop_car();
    speed_car_pre = 0;
    runFlag = false;
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float data[3];            //lưu trữ dữ liệu khoảng cách của ba hướng động cơ servo quay
bool turnFlag = false;
void data_collection(){
  OCR2A = 159;            // tương ứng với góc -90 độ của động cơ servo sg90
  _delay_ms(600);
  data[0] = read_hcsr04();
  OCR2A = 38;             // tương ứng với góc 90 độ của động cơ servo sg90
  _delay_ms(600);
  data[2] = read_hcsr04();
  OCR2A = 100;             // tương ứng với góc 0 độ của động cơ servo sg90
  _delay_ms(200);
  data[1] = read_hcsr04();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void vehicle_behavior(float left, float front, float right){
  if(front > 32){
    runFlag = true;
    run_forward();        // Đi thẳng
  }
  else if(((left > 30) && (left < 200)) && (left>right)){
    turnFlag = true;
    turn_left(496);       // Rẽ trái
    stop_car();
    delay(300);
    runFlag = false;
    turnFlag = false;
  }
  else if(((right > 30) && (right < 200)) && (right>left)){
    turnFlag = true;
    turn_right(496);      // Rẽ phải
    stop_car();
    delay(300);
    runFlag = false;
    turnFlag = false;
  }
  else{
    turnFlag = true;
    turn_left(936);       // Quay ngược lại 180 độ
    stop_car();
    delay(300);
    runFlag = false;
    turnFlag = false;
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Khởi tạo các ma trận và vector cho bộ lọc Kalman
Matrix<MATRIX_SIZE, MATRIX_SIZE> A;       // Ma trận chuyển trạng thái
Matrix<MATRIX_SIZE, MATRIX_SIZE> B;       // Ma trận điều khiển
Matrix<MATRIX_SIZE, MATRIX_SIZE> H;       // Ma trận quan sát
Matrix<MATRIX_SIZE, MATRIX_SIZE> R;       // Ma trận hiệp phương sai đo lường
Matrix<MATRIX_SIZE, MATRIX_SIZE> P;       // Ma trận hiệp phương sai dự đoán
Matrix<MATRIX_SIZE, MATRIX_SIZE> I;       // Ma trận đơn vị
Matrix<MATRIX_SIZE, MATRIX_SIZE> K;       // Ma trận Kalman Gain
Matrix<MATRIX_SIZE, 1> x_hat;             // Ma trận dự đoán trạng thái
Matrix<MATRIX_SIZE, 1> X;                 // Ma trận ước lượng trạng thái thực => OUTPUT
Matrix<MATRIX_SIZE, 1> Y;                 // Ma trận đo lường
Matrix<MATRIX_SIZE, 1> U;                 // Ma trận điều khiển

// Chương trình lọc Kalman
float Kalman(float ultrasonic_distance, float encoder_speed, float accelerate, uint8_t n){
  // Khởi tạo giá trị các ma trận và vector cho bộ lọc Kalman
  A = {1, -0.1, 0, 1};
  B = {-0.005, 0, 0, 0.1};
  H = {1, 0, 0, 1};
  R = {0.068, 0, 0, 0.706};
  P = {0.08, 0, 0, 0.36};
  I = {1, 0, 0, 1};
  // Khởi tạo trạng thái dự đoán
  x_hat = {ultrasonic_distance-6, encoder_speed};
  U = {accelerate, accelerate};
  // Lọc n lớp
  while(n--){
    x_hat = A*x_hat + B*U;
    // Cập nhật Ma trận hiệp phương sai dự đoán
    P = A*P*(~A);
    P(0, 1) = 0;
    P(1, 0) = 0;
    // Tính ma trận Kalman Gain
    K = P*(~H)*Inverse(H*P*(~H)+R);
    // Khởi tạo trạng thái đo lường
    Y = {ultrasonic_distance, encoder_speed};
    // Cập nhật trạng thái ước tính và phương sai ước tính
    x_hat = x_hat + K*(Y-H*x_hat);
    P = (I-K*H)*P;
  }
  return x_hat(0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
  hcsr04_init();        //Khởi tạo cảm biến siêu âm HC-SR04
  sg90_init();          //Khởi tạo động cơ servo sg90, đưa vị trí động cơ servo về hướng thẳng
  time_init();          //Khởi tạo thời gian đọc encoder
  encoder_init();       //Khởi tạo ngắt ngoài đọc giá trị encoder
  sei();                //bật ngắt toàn cục
  
}

void loop() {
  if((!runFlag) && (!turnFlag)){
    data_collection();    // Thu thập dữ liệu
    vehicle_behavior(data[0], data[1], data[2]);      // Hành xử của xe
  }
  else{
    obstacles_detected();                             // Thực hiện phát hiện vật cản khi đi thẳng
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t pwm = 150;                     //Thiết lập xung PWM cho động cơ
uint8_t pwm_offset = 20;               //Thiết lập xung PWM bù vào độ lệch một bên bánh xe khi động cơ đi thẳng
uint8_t pwm_turn = 80;                 //Thiết lập xung PWM thêm vào một bên bánh xe và giảm bê bên ngược lại để rẽ trái/phải

void set_speed_forward(){
  motor1.setSpeed(pwm+pwm_offset);                 // Đặt tốc độ đi thẳng cho động cơ 1
  motor2.setSpeed(pwm);                            // Đặt tốc độ đi thẳng cho động cơ 2
  motor3.setSpeed(pwm);                            // Đặt tốc độ đi thẳng cho động cơ 3
  motor4.setSpeed(pwm+pwm_offset);                 // Đặt tốc độ đi thẳng cho động cơ 4
}

void set_speed_turn(){
  motor1.setSpeed(pwm+pwm_turn+pwm_offset);        // Đặt tốc độ rẽ cho động cơ 1
  motor2.setSpeed(pwm+pwm_turn);                   // Đặt tốc độ rẽ cho động cơ 2
  motor3.setSpeed(pwm+pwm_turn);                   // Đặt tốc độ rẽ cho động cơ 3
  motor4.setSpeed(pwm+pwm_turn+pwm_offset);        // Đặt tốc độ rẽ cho động cơ 4
}

void run_forward(){
  set_speed_forward();
  motor1.run(FORWARD);                  // Chạy động cơ 1 về phía trước
  motor2.run(FORWARD);                  // Chạy động cơ 2 về phía trước
  motor3.run(FORWARD);                  // Chạy động cơ 3 về phía trước
  motor4.run(FORWARD);                  // Chạy động cơ 4 về phía trước
}

void turn_left(unsigned int time){
  set_speed_turn();
  motor1.run(FORWARD);                  // Chạy động cơ 1 về phía trước
  motor2.run(BACKWARD);                 // Chạy động cơ 2 về phía sau
  motor3.run(BACKWARD);                 // Chạy động cơ 3 về phía sau
  motor4.run(FORWARD);                  // Chạy động cơ 4 về phía trước
  delay(time);                          // Chờ đến khi xe rẽ hoặc quay xong
}

void turn_right(unsigned int time){
  set_speed_turn();
  motor1.run(BACKWARD);                 // Chạy động cơ 1 về phía trước
  motor2.run(FORWARD);                  // Chạy động cơ 2 về phía sau
  motor3.run(FORWARD);                  // Chạy động cơ 3 về phía sau
  motor4.run(BACKWARD);                 // Chạy động cơ 4 về phía trước
  delay(time);                          // Chờ đến khi xe rẽ hoặc quay xong
}

void stop_car(){
  motor1.run(RELEASE);                  // Dừng động cơ 1
  motor2.run(RELEASE);                  // Dừng động cơ 2
  motor3.run(RELEASE);                  // Dừng động cơ 3
  motor4.run(RELEASE);                  // Dừng động cơ 4
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
