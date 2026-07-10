#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#include <geometry_msgs/msg/twist.h>

namespace {

constexpr uint8_t MOTOR_I2C_ADDR = 0x34;
constexpr uint8_t MOTOR_TYPE_ADDR = 0x14;
constexpr uint8_t MOTOR_ENCODER_POLARITY_ADDR = 0x15;
constexpr uint8_t MOTOR_FIXED_SPEED_ADDR = 0x33;

constexpr uint8_t MOTOR_TYPE_WITHOUT_ENCODER = 0;
constexpr uint8_t MOTOR_TYPE_TT = 1;
constexpr uint8_t MOTOR_TYPE_N20 = 2;
constexpr uint8_t MOTOR_TYPE_JGB = 3;

constexpr uint8_t MOTOR_TYPE = MOTOR_TYPE_JGB;
constexpr uint8_t MOTOR_ENCODER_POLARITY = 0;

constexpr float LINEAR_TO_MOTOR_SPEED = 100.0f;
constexpr float ANGULAR_TO_MOTOR_SPEED = 100.0f;
constexpr int8_t MAX_MOTOR_SPEED = 100;
constexpr uint32_t CMD_VEL_TIMEOUT_MS = 500;
constexpr uint32_t MOTOR_UPDATE_PERIOD_MS = 50;

// Tracked chassis wiring: M1 is one track, M3 is the other.
// Flip either sign here if that track turns backward during testing.
constexpr int8_t M1_FORWARD_SIGN = -1;
constexpr int8_t M3_FORWARD_SIGN = 1;

uint32_t last_motor_update_ms = 0;
float target_linear_x = 0.0f;
float target_angular_z = 0.0f;
uint32_t last_cmd_vel_ms = 0;
int cmd_vel_count = 0;

int8_t clamp_motor_speed(float speed)
{
  if (speed > MAX_MOTOR_SPEED) {
    return MAX_MOTOR_SPEED;
  }
  if (speed < -MAX_MOTOR_SPEED) {
    return -MAX_MOTOR_SPEED;
  }
  return static_cast<int8_t>(speed);
}

void wire_write_data_array(uint8_t register_address, const uint8_t * data, uint8_t length)
{
  Wire.beginTransmission(MOTOR_I2C_ADDR);
  Wire.write(register_address);
  for (uint8_t i = 0; i < length; i++) {
    Wire.write(data[i]);
  }
  Wire.endTransmission();
}

void setup_motor_driver()
{
  Wire.begin();
  Wire.setClock(100000);
  delay(200);

  uint8_t motor_type = MOTOR_TYPE;
  uint8_t encoder_polarity = MOTOR_ENCODER_POLARITY;

  wire_write_data_array(MOTOR_TYPE_ADDR, &motor_type, 1);
  delay(5);
  wire_write_data_array(MOTOR_ENCODER_POLARITY_ADDR, &encoder_polarity, 1);
  delay(5);
}

void write_motor_speeds(int8_t motor_1, int8_t motor_2, int8_t motor_3, int8_t motor_4)
{
  uint8_t speeds[4] = {
    static_cast<uint8_t>(motor_1),
    static_cast<uint8_t>(motor_2),
    static_cast<uint8_t>(motor_3),
    static_cast<uint8_t>(motor_4),
  };

  wire_write_data_array(MOTOR_FIXED_SPEED_ADDR, speeds, 4);
}

void stop_motors()
{
  write_motor_speeds(0, 0, 0, 0);
}

void update_motor_driver()
{
  const uint32_t now_ms = millis();
  if (now_ms - last_motor_update_ms < MOTOR_UPDATE_PERIOD_MS) {
    return;
  }
  last_motor_update_ms = now_ms;

  if (now_ms - last_cmd_vel_ms > CMD_VEL_TIMEOUT_MS) {
    stop_motors();
    return;
  }

  const float forward = target_linear_x * LINEAR_TO_MOTOR_SPEED;
  const float turn = target_angular_z * ANGULAR_TO_MOTOR_SPEED;

  const int8_t left_speed = clamp_motor_speed(forward - turn);
  const int8_t right_speed = clamp_motor_speed(forward + turn);

  write_motor_speeds(
    M1_FORWARD_SIGN * left_speed,
    0,
    M3_FORWARD_SIGN * right_speed,
    0);
}

}  // namespace

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_publisher_t publisher;
rcl_timer_t timer;
rclc_executor_t executor;

std_msgs__msg__Int32 msg;

rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

void cmd_vel_callback(const void * msg_in)
{
  const geometry_msgs__msg__Twist * msg =
    (const geometry_msgs__msg__Twist *)msg_in;

  target_linear_x = msg->linear.x;
  target_angular_z = msg->angular.z;
  last_cmd_vel_ms = millis();
  cmd_vel_count++;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;

  if (timer != NULL) {
    rcl_ret_t publish_ret = rcl_publish(&publisher, &msg, NULL);
    (void) publish_ret;
    msg.data++;
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  setup_motor_driver();
  stop_motors();

  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(
    &node,
    "teensy_base",
    "",
    &support
  );

  rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );

  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "teensy/heartbeat"
  );

  rclc_timer_init_default2(
    &timer,
    &support,
    RCL_MS_TO_NS(1000),
    timer_callback,
    true
  );

  rclc_executor_init(
    &executor,
    &support.context,
    2,
    &allocator
  );

  rclc_executor_add_subscription(
    &executor,
    &cmd_vel_subscriber,
    &cmd_vel_msg,
    &cmd_vel_callback,
    ON_NEW_DATA
  );
  
  rclc_executor_add_timer(&executor, &timer);

  msg.data = 0;
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  update_motor_driver();
  delay(10);
}
