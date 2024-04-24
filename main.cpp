#include "mbed.h"

// pes board pin map
#include "pm2_drivers/PESBoardPinMap.h"

// drivers
#include "pm2_drivers/DebounceIn.h"
#include "pm2_drivers/IMU.h"
#include "pm2_drivers/Servo.h"
#include "pm2_drivers/IIR_Filter.h"

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once
// while loop gets executed every main_task_period_ms milliseconds, this is a
// simple approach to repeatedly execute main
const int main_task_period_ms = 20; // define main task period time in ms e.g. 20 ms, there for
                                    // the main task will run 50 times per second

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(USER_BUTTON); // create DebounceIn object to evaluate the user button
                                     // falling and rising edge
void toggle_do_execute_main_fcn();   // custom function which is getting executed when user
                                     // button gets pressed, definition below
//float iir_filter(float u_k);         // function for implementing low pass filter 

// main runs as an own thread
int main()
{
    // attach button fall function address to user button object, button has a pull-up resistor
    user_button.fall(&toggle_do_execute_main_fcn);

    Timer main_task_timer;              // create Timer object which we use to run the main task
                                        // every main_task_period_ms
    Timer timer;

    // led on nucleo board
    DigitalOut user_led(USER_LED);

    IIR_Filter lp_filter(1.0f/(2.0f * M_PI * 20.0f), main_task_period_ms, 1.0f);
    // servo
    Servo servo_roll(PB_D0);
    Servo servo_pitch(PB_D1);

    // minimal pulse width and maximal pulse width obtained from the servo calibration process
    // nodelcraft RS2 MG/BB
    float servo_ang_min = 0.03f; // carefull, these values might differ from servo to servo
    float servo_ang_max = 0.13f;

    // servo.setNormalisedPulseWidth: before calibration (0,1) -> (min pwm, max pwm)
    // servo.setNormalisedPulseWidth: after calibration (0,1) -> (servo_D0_ang_min, servo_D0_ang_max)
    servo_roll.calibratePulseMinMax(servo_ang_min, servo_ang_max);
    servo_pitch.calibratePulseMinMax(servo_ang_min, servo_ang_max);

    // angle limits of the servos
    float angle_range_min = -M_PI/2.0f;
    float angle_range_max = M_PI/2.0f;    

    // angle to pulse width coefficients
    float a = 1.0f / M_PI;
    float b = 0.5f;

    // pulse width
    float roll_servo_width;
    float roll_servo_width_f;
    float pitch_servo_width;
    float pitch_servo_width_f;

    // IMU
    ImuData imu_data;
    IMU imu(PB_IMU_SDA, PB_IMU_SCL);    
    Eigen::Vector2f rp;
    
    // start timer
    main_task_timer.start();
    timer.start();

    // this loop will run forever
    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {
            
            if (!servo_roll.isEnabled())
                servo_roll.enable();
            if (!servo_pitch.isEnabled())
                servo_pitch.enable();

            imu_data = imu.getImuData();

            // roll
            rp(0) = atan2f(imu_data.quat.x() + imu_data.quat.z(), imu_data.quat.w() - imu_data.quat.y()) - atan2f(imu_data.quat.z() - imu_data.quat.x(), imu_data.quat.y() + imu_data.quat.w());
            // pitch
            rp(1) = acosf((imu_data.quat.w() - imu_data.quat.y()) * (imu_data.quat.w() - imu_data.quat.y()) + (imu_data.quat.x() + imu_data.quat.z()) * (imu_data.quat.x() + imu_data.quat.z()) - 1.0f) - M_PI / 2.0f;

            roll_servo_width = -a * rp(0) + b;
            pitch_servo_width = a * rp(1) + b;

            roll_servo_width_f = /*lp_filter.filter*/(lp_filter.filter(roll_servo_width));

            pitch_servo_width_f = lp_filter.filter(pitch_servo_width); 

            if (rp(0) < M_PI/2.0f && rp(0) > -M_PI/2.0f) {
                servo_roll.setNormalisedPulseWidth(roll_servo_width_f);
            }
            if (rp(0) < M_PI/2.0f && rp(0) > -M_PI/2.0f) {
                servo_pitch.setNormalisedPulseWidth(pitch_servo_width_f);    
            }

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;
                servo_roll.setNormalisedPulseWidth(0.5f);
                servo_pitch.setNormalisedPulseWidth(0.5f);
            }
        }

        // toggling the user led
        user_led = !user_led;

        float time_ms = std::chrono::duration_cast<std::chrono::microseconds>(timer.elapsed_time()).count() * 1.0e-3f;

        //printf("Angles: roll %f, pitch %f \n", rp(0) * 180 / M_PI, rp(1) * 180 / M_PI);
        printf("%f, %f, \n", roll_servo_width, roll_servo_width_f);
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}


// float iir_filter(float u_k) 
// {
//     static float u_k_1;
//     static float y_k;
//     static float y_k_1;
//     static const float tau = 1.0f / (M_PI);
//     static const float Ts = main_task_period_ms;

//     static float a1 = (Ts + 2.0f * tau);
//     static float a0 = (Ts - 2.0f *tau) / a1;
//     static float b0 = Ts/a1;
//     static float b1 = b0;

//     y_k = b1 * u_k + b0 * u_k_1 - a0 * y_k_1;

//     u_k_1 = u_k;
//     y_k_1 = y_k;

//     return y_k;
// }
