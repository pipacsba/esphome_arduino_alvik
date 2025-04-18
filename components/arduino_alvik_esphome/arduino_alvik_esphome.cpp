
//#include "Arduino_Alvik.h"
#include "arduino_alvik_esphome.h"

#include "esphome/core/entity_base.h"
#include "esphome/core/application.h"
#include "esphome/components/number/number.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <utility>

namespace esphome {
namespace alvik {

    static const char *const TAG = "arduinoalvik";

    static uint32_t millis()
    {
      return esp_timer_get_time() / 1000;
    }

    void AlvikComponent::setup() {
        this->set_cycle(0);
        this->orientation_correction_enabled = false;

        //----------------Alvik HW variables
        last_ack = NO_ACK;
        waiting_ack = NO_ACK;
        
        fw_version[0] = 0;
        fw_version[1] = 0;
        fw_version[2] = 0;

        lib_version[0] = 1;
        lib_version[1] = 1;
        lib_version[2] = 0;
        
        led_state = 0;
        
        line_sensors[0] = 0;
        line_sensors[1] = 0;
        line_sensors[2] = 0;
        
        color_sensor[0] = 0;
        color_sensor[1] = 0;
        color_sensor[2] = 0;
        
        servo_positions[0] = 90;
        servo_positions[1] = 90;
        
        orientation[0] = 0.0;
        orientation[1] = 0.0;
        orientation[2] = 0.0;
        yaw_est = 0.0;
        
        move_bits = 0;
        
        imu[0] = 0.0;
        imu[1] = 0.0;
        imu[2] = 0.0;
        imu[3] = 0.0;
        imu[4] = 0.0;
        imu[5] = 0.0;
        
        distances[0] = 0.0;
        distances[1] = 0.0;
        distances[2] = 0.0;
        distances[3] = 0.0;
        distances[4] = 0.0;
        distances[5] = 0.0;
        distances[6] = 0.0;
        distances_updated = 0;
        
        touch = 0;
        touch_bits = 0;
        
        joints_velocity[0] = 0.0;
        joints_velocity[1] = 0.0;
      
        joints_position[0] = 0.0;
        joints_position[1] = 0.0;
       
        robot_velocity[0] = 0.0;
        robot_velocity[1] = 0.0;
        
        robot_pose[0] = 0.0;
        robot_pose[1] = 0.0;
        robot_pose[2] = 0.0;
        
        battery = 0.0;
        battery_soc = 0.0;
        battery_is_charging = false;

        this->set_stm32_fw_compatible(false);
        this->stm32_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLDOWN);
        this->i2c_switch1_pin_->pin_mode(gpio::FLAG_INPUT);
        this->i2c_switch2_pin_->pin_mode(gpio::FLAG_INPUT);
        
        this->nano_pin_->pin_mode(gpio::FLAG_OUTPUT);
        this->reset_pin_->pin_mode(gpio::FLAG_OUTPUT);
        this->red_led_pin_->pin_mode(gpio::FLAG_OUTPUT);
        this->green_led_pin_->pin_mode(gpio::FLAG_OUTPUT);
        this->blue_led_pin_->pin_mode(gpio::FLAG_OUTPUT);

        //----------------ACTION_MAZE_SOLVER
        maze_solver_start_         = false;
        maze_solved_               = false;
        line_detection_threshold_  = 300;
        maze_solution_             = "";
        maze_solution_saved_       = "";
        maze_crawling_speed_       = 0; //RPM
        maze_crawling_speed_max_   = 30;
        this->maze_crawling_speed_number_->publish_state(maze_crawling_speed_);
        intersection_dir_          = INTERSECTION_NONE;
        maze_crawling_state_       = CRAWLING_STRAIGHT;
        maze_saved_cycle_counter_  = 0;
        maze_intersection_counter_ = 0;
        maze_left_turn_confidence  = 0;
        maze_right_turn_confidence = 0;
        maze_dead_end_confidence   = 0;
        maze_solution_trust_confidence_ = 1;
        maze_straight_continue_confidence_inverze_ = 1;
        maze_turn_started_confidence = 0;
        maze_turn_start_yaw_ = 0;

        line_follower_p_  = 20;
        line_follower_d_  = 2;
        line_follower_i_  = 0.1;
        this->linefollower_p_number_->publish_state(line_follower_p_);
        this->linefollower_i_number_->publish_state(line_follower_i_);
        this->linefollower_d_number_->publish_state(line_follower_d_);
        line_follower_centoid_previous_ = 0.0;
        line_follower_centoid_integral_ = 0.0;

        //----------------ACTION_CONSTANT_DIRECTION
        direction_control_start_ = false;
        constant_direction_tolerance_angle_ = 5;
        constant_direction_gain_ = 3;
        constant_direction_target_angle_ = 0;
        this->constant_direction_gain_number_->publish_state(constant_direction_gain_);
        this->constant_direction_target_number_->publish_state(constant_direction_target_angle_);


        //----------------ACTION_PERFORM_COMMAND_LIST; ACTION_COLLECT_COMMAND_LIST
        this->forward_distance_->publish_state(150);
        this->set_forward_move_distance(150);
        this->turn_degree_number_->publish_state(90);
        this->set_turn_degree(90);


        //----------------ACTION_FOLLOW
        follow_start_ = false;
        follow_target_       = 150;
        follow_tolerance_    =  20;
        follow_Kp_           =   1;
        follow_K_horizontal_ =   5;
        centoid_tolerance_   = 0.5;
        this->follow_distance_number_->publish_state(follow_target_);
        this->follow_tolerance_number_->publish_state(follow_tolerance_);
        this->follow_gain_horizontal_number_->publish_state(follow_K_horizontal_);
        this->follow_gain_front_number_->publish_state(follow_Kp_);

        //--------------Set up Alvik
        this->nano_pin_->digital_write(false);
        this->red_led_pin_->digital_write(true);
        this->green_led_pin_->digital_write(true);
        this->blue_led_pin_->digital_write(true);
        
        this->flush();
        while (this->available()){
            this->read();
        }

        this->reset_pin_->digital_write(false);
        
        this->set_alvik_state(ALVIK_STARTUP);
        this->set_alvik_action(ACTION_NOT_SET);
        
        this->last_command_time_ = 0;
        this->last_sensor_time_  = 0;
        this->sensor_group_ = 0;
        this->received_messages_count_ = 0;
        this->last_command_received_time_ = 0;
        this->alvik_command_list_.clear();

        //--------------LSM303DLHC (magnetic compass) measurement
        if (this->compass_sensor_  != nullptr)
       {
            this->compass_sensor_->write_byte(M_REG_M, 0x00);
            this->compass_sensor_->write_byte(CRA_REG_M, 0x0C);  //0x08 3Hz; 0x0C 7.5Hz
            this->compass_sensor_->write_byte(CRB_REG_M, 0x10);  // 1.3Gauss range
            this->compass_sensor_->write_byte(M_REG_M, 0x00);    // continous measuremeent mode
            this->compass_x_min = 51.36;     //55;
            this->compass_x_max = 59.18;     //55;
            this->compass_y_min = -73.82;    //-50;
            this->compass_y_max = -26.64;    //-35;
            this->compass_z_min = -72.24;    //-50;
            this->compass_z_max = -20.82;    //-25;
            this->compass_x_offset = (this->compass_x_max + this->compass_x_min) / 2;
            this->compass_y_offset = (this->compass_y_max + this->compass_y_min) / 2;
            this->compass_z_offset = (this->compass_z_max + this->compass_z_min) / 2;
        }        
        
        ESP_LOGD(TAG, "Setup is finished, STM32 is in reset");
    }

    void AlvikComponent::loop() {
        uint32_t now = millis();
        uint8_t current_action;
        bool ison = this->stm32_pin_->digital_read();
        if ((!ison) & (this->alvik_state_ > ALVIK_HW_RESET) & (this->alvik_state_ != ALVIK_EXTERNAL_SUPPLY))
        {
            this->set_alvik_state(ALVIK_STARTUP);
            this->cycle_ = 0;
        }
        switch(this->alvik_state_)
        {
            case ALVIK_STARTUP:
                {
                    if (this->cycle_ == 0)
                    {    
                        this->reset_pin_->digital_write(false);
                        this->flush();
                        while (this->available()){
                            this->read();
                        }
                        if (this->alvik_alive_sensor_ != nullptr)
                            this->alvik_alive_sensor_->publish_state(this->alvik_state_);
                    }
                    this->cycle_ = this->cycle_ + 1;
                    if (this->cycle_ == 500)
                    {
                        this->reset_pin_->digital_write(true);
                        this->set_alvik_state(ALVIK_HW_RESET);
                        this->cycle_ = 0;
                        if (this->alvik_alive_sensor_ != nullptr)
                            this->alvik_alive_sensor_->publish_state(this->alvik_state_);
                    }
                    break;
                }
            case ALVIK_HW_RESET:
                {
                    if (ison)
                    {
                        this->set_stm32_state(ison);
                        ESP_LOGD(TAG, "STM32 is up again");
                        this->cycle_ = 0;
                        this->waiting_ack = 0x00;
                        this->set_alvik_state(ALVIK_STM32_UP);
                        if (this->alvik_alive_sensor_ != nullptr)
                            this->alvik_alive_sensor_->publish_state(this->alvik_state_);
                    }
                    this->cycle_ = this->cycle_ + 1;
                    if (this->cycle_ > 100)
                    {
                        //USB supply, battery charging as the STM32 is not ON
                        this->set_alvik_state(ALVIK_EXTERNAL_SUPPLY);
                        this->cycle_ = 0;
                        this->nano_pin_->digital_write(true);
                        if (this->alvik_alive_sensor_ != nullptr)
                            this->alvik_alive_sensor_->publish_state(this->alvik_state_);
                    }
                    break;
                }
            case ALVIK_STM32_UP:
                {
                    if (read_message())
                    {
                        parse_message();
                    }
                    if (this->last_ack == this->waiting_ack)
                    {
                        this->set_alvik_state(ALVIK_FIRST_ACK);
                        ESP_LOGD(TAG, "Wait_for_Ack completed!");
                        if (this->alvik_alive_sensor_ != nullptr)
                            this->alvik_alive_sensor_->publish_state(this->alvik_state_);
                        read_compass_data();
                    }
                    break;
                }
            case ALVIK_FIRST_ACK:
                {
                    if (read_message())
                    {
                        parse_message();
                    }
                    if (this->stm32_fw_compatible_)
                    {
                        this->set_alvik_state(ALVIK_FW_COMPATIBLE);
                        this->set_behaviour(BEHAVIOUR_ILLUMINATOR_RISE);
                        this->set_behaviour(BEHAVIOUR_BATTERY_ALERT);
                        this->set_servo_positions(0,0);
                        this->yaw_est = this->compass_angle;
                        this->reset_pose(0, 0, this->compass_angle);
                        this->angle_at_offset = this->compass_angle;
                        this->last_command_time_ = now;
                        this->set_alvik_action(ACTION_MAZE_SOLVER);
                    }
                    break;
                }
            case ALVIK_FW_COMPATIBLE:
                {
                    this->cycle_ = this->cycle_ + 1;
                    current_action = this->cycle_ % 3;
                    switch (current_action)
                    {
                        case TASK_READ_UART:
                            while (this->available()){
                                if (read_message())
                                {
                                    parse_message();
                                    this->received_messages_count_ = this->received_messages_count_ + 1;
                                }
                            }
                            break;
                        case TASK_PERFORM_ACTION:
                            switch (this->alvik_action_)
                            {
                                case ACTION_PERFORM_COMMAND_LIST:
                                {
                                    this->do_one_item_from_command_list(now);
                                    break;
                                }
                                case ACTION_COLLECT_COMMAND_LIST:
                                {
                                    if ((now - this->last_command_received_time_) < 50)
                                        this->change_alvik_left_right_leds(LEFT_BLUE + RIGHT_BLUE, true);
                                    if ((now - this->last_command_received_time_) > 300)
                                        this->change_alvik_left_right_leds(LEFT_BLUE + RIGHT_BLUE, false);
                                    break;
                                }
                                case ACTION_FOLLOW:
                                {
                                    if (this->follow_start_)
                                    {
                                        alvik_follow_control();
                                    }
                                    break;
                                }
                                case ACTION_CONSTANT_DIRECTION:
                                {
                                    if (this->direction_control_start_)
                                    {
                                        alvik_constant_direction_control();
                                    }
                                    break;
                                }
                                case ACTION_MAZE_SOLVER:
                                {
                                    if (this->maze_solver_start_)
                                    {
                                        alvik_maze_solver();
                                    }
                                    else
                                    {
                                        if (this->maze_solved_)
                                        {
                                            if (this->cycle_ % 50 == 0)
                                            {
                                                this->change_alvik_left_right_leds(0xff, false);
                                            }
                                            if (this->cycle_ % 50 == 25)
                                            {
                                                this->change_alvik_left_right_leds(LEFT_GREEN + RIGHT_GREEN, true);
                                            }
                                        }
                                        this->brake();
                                    }
                                    break;
                                }

                                
                            }
                            break;
                        case TASK_WRITE_SENSOR:
                            if ((now - this->last_sensor_time_) >= 1 * 1000)
                            {
                                switch (this->sensor_group_ )
                                {
                                    case 0:
                                    {
                                        if (this->battery_sensor_ != nullptr)
                                            this->battery_sensor_->publish_state(this->battery);
                                        if (this->alvik_alive_sensor_ != nullptr)
                                            this->alvik_alive_sensor_->publish_state(this->alvik_state_);
                                        if (this->alvik_action_state_ != nullptr)
                                            this->alvik_action_state_->publish_state(this->alvik_action_);
                                        if (this->pose_x_sensor_ != nullptr)
                                            this->pose_x_sensor_->publish_state(this->robot_pose[0]);
                                        if (this->pose_y_sensor_ != nullptr)
                                            this->pose_y_sensor_->publish_state(this->robot_pose[1]);
                                        if (this->pose_ang_sensor_ != nullptr)
                                            this->pose_ang_sensor_->publish_state(this->robot_pose[2]);
                                        if (this->command_list_sensor_ != nullptr)
                                            this->command_list_sensor_->publish_state(this->alvik_command_list_);
                                        if (this->roll_sensor_ != nullptr)
                                            this->roll_sensor_->publish_state(this->orientation[0]);
                                        if (this->pitch_sensor_ != nullptr)
                                            this->pitch_sensor_->publish_state(this->orientation[1]);
                                        if (this->received_messages_counter_sensor_ != nullptr)
                                            this->received_messages_counter_sensor_->publish_state(this->received_messages_count_);

                                        this->received_messages_count_ = 0;
                                        this->sensor_group_ = this->sensor_group_ + 1;
                                        break;
                                    }
                                    case 1:
                                    {
                                        if (this->distance_l_ != nullptr)
                                            this->distance_l_->publish_state(this->distances[0]);
                                        if (this->distance_cl_ != nullptr)
                                            this->distance_cl_->publish_state(this->distances[1]);
                                        if (this->distance_c_ != nullptr)
                                            this->distance_c_->publish_state(this->distances[2]);
                                        if (this->distance_cr_ != nullptr)
                                            this->distance_cr_->publish_state(this->distances[3]);
                                        if (this->distance_r_ != nullptr)
                                            this->distance_r_->publish_state(this->distances[4]);
                                        if (this->distance_t_ != nullptr)
                                            this->distance_t_->publish_state(this->distances[5]);
                                        if (this->distance_b_ != nullptr)
                                            this->distance_b_->publish_state(this->distances[6]);
                                        if (this->yaw_sensor_ != nullptr)
                                            this->yaw_sensor_->publish_state(this->orientation[2]);
                                        if (this->yaw_est_sensor_ != nullptr)
                                            this->yaw_est_sensor_->publish_state(this->yaw_est);
                                        if (this->maze_solver_start_sensor_ != nullptr)
                                            this->maze_solver_start_sensor_->publish_state(this->maze_solver_start_);
                                        this->sensor_group_ = this->sensor_group_ + 1;
                                        break;
                                    }
                                    case 2:
                                    {
                                        if (this->joints_l_ != nullptr)
                                            this->joints_l_->publish_state(this->joints_position[0]);
                                        if (this->joints_r_ != nullptr)
                                            this->joints_r_->publish_state(this->joints_position[1]);
                                        if (this->tof_centoid_ != nullptr)
                                            this->tof_centoid_->publish_state(this->centoid_filt);
                                        if (this->wheel_speed_left_ != nullptr)
                                            this->wheel_speed_left_->publish_state(this->joints_velocity[0]);
                                        if (this->wheel_speed_right_ != nullptr)
                                            this->wheel_speed_right_->publish_state(this->joints_velocity[1]);
                                        if (this->follow_start_sensor_ != nullptr)
                                            this->follow_start_sensor_->publish_state(this->follow_start_);
                                        if (this->direction_control_start_sensor_ != nullptr)
                                            this->direction_control_start_sensor_->publish_state(this->direction_control_start_);
                                        if (this->line_sensor_left_ != nullptr)
                                            this->line_sensor_left_->publish_state(this->line_sensors[0]);
                                        if (this->line_sensor_center_ != nullptr)
                                            this->line_sensor_center_->publish_state(this->line_sensors[1]);
                                        if (this->line_sensor_right_ != nullptr)
                                            this->line_sensor_right_->publish_state(this->line_sensors[2]);
                                        
                                        this->sensor_group_ = this->sensor_group_ + 1;
                                        break;
                                    }
                                    case 3:
                                    {
                                        read_compass_data();
                                        if (this->compass_sensor_ != nullptr)
                                            this->compass_sensor_->publish_state(this->compass_angle);
                                        if (this->compass_x_ != nullptr)
                                            this->compass_x_->publish_state(this->compass_measurements[0]);
                                        if (this->compass_y_ != nullptr)
                                            this->compass_y_->publish_state(this->compass_measurements[1]);
                                        if (this->compass_z_ != nullptr)
                                            this->compass_z_->publish_state(this->compass_measurements[2]);
                                        if (this->maze_crawling_state_sensor_ != nullptr)
                                            this->maze_crawling_state_sensor_->publish_state(this->maze_crawling_state_);
                                        if (this->maze_descriptor_sensor_ != nullptr)
                                            this->maze_descriptor_sensor_->publish_state(this->maze_solution_);
                                        if (this->maze_solution_descriptor_sensor_ != nullptr)
                                            this->maze_solution_descriptor_sensor_->publish_state(this->maze_solution_saved_);
                                        if (this->maze_solved_sensor_ != nullptr)
                                            this->maze_solved_sensor_->publish_state(this->maze_solved_);
                                        this->sensor_group_ = this->sensor_group_ + 1;
                                        break;
                                    }
                                    default:
                                    {
                                        this->last_sensor_time_= now;
                                        this->sensor_group_ = 0;
                                        break;
                                    }
                                }

                            }
                            break;
                        default:
                            break;
                    }
                    break;
                }
            //USB supply, battery charging
            case ALVIK_EXTERNAL_SUPPLY:
            {
                this->cycle_ = this->cycle_ + 1;
                this->external_supply_measurement(ison);
                break;
            }
            default:
                break;
        }

    }

    void AlvikComponent::alvik_line_follower()
    {
        float sum_weight, sum_values, centoid, line_left, line_center, line_right;
        float diff_speed, diff_speed_p, diff_speed_i, diff_speed_d;
        float centoid_difference;
        float common_speed;
        
        line_left =this->line_sensors[0] - 60;
        line_center =this->line_sensors[1] - 60;
        line_right =this->line_sensors[2] - 60;
        
        common_speed = this->maze_crawling_speed_;
        
        //calculate axial (difference) wheel speeds
        sum_weight = line_left + line_center + line_right;
        sum_values = line_left + 2 * line_center + 3 * line_right;
        if (sum_weight != 0) { centoid = 2 - (sum_values / sum_weight); }
        else { centoid = 0.0; }
        
        if (this->line_follower_centoid_previous_ != 0.0)
        {
         centoid_difference = centoid - this->line_follower_centoid_previous_;
        }
        else
        {
         centoid_difference = 0.0;
        }
        this->line_follower_centoid_integral_ += centoid;
        
        common_speed = common_speed / (abs(centoid) + 1);
        //if (abs(centoid) > 0.5)
        //{
        //    common_speed = 0;
        //}
        
        diff_speed_p = centoid * this->line_follower_p_;
        diff_speed_i = this->line_follower_centoid_integral_ * this->line_follower_i_;
        diff_speed_d = centoid_difference * this->line_follower_d_;
        diff_speed = diff_speed_p + diff_speed_i + diff_speed_d;
        
        set_wheels_speed(common_speed - diff_speed, common_speed + diff_speed);
        
        this->line_follower_centoid_previous_ = centoid;
    }

    void AlvikComponent::maze_turn_right()
    {
        //turn right - we checked if line continues straight, so we neeed sharp turn as the intersection is below the robot
        //set_wheels_speed(this->maze_crawling_speed_ / 2, 0);
        set_wheels_speed(this->maze_crawling_speed_ / 2, - (this->maze_crawling_speed_ / 4));
        this->maze_saved_cycle_counter_ = this->cycle_;
        this->maze_crawling_state_ = CRAWLING_INTERSECTION ;
        this->intersection_dir_ = INTERSECTION_RIGHT;
        this->maze_solution_.push_back('R');
        ESP_LOGD(TAG, "Right turn with right confidence: %.2f;  dead-end confidence: %.2f", this->maze_right_turn_confidence, this->maze_dead_end_confidence);
        this->maze_left_turn_confidence = 0;
        this->maze_right_turn_confidence = 0;
        this->maze_dead_end_confidence   = 0;
        this->maze_straight_continue_confidence_inverze_ = 1;
        this->maze_turn_start_yaw_ = this->robot_pose[2];
    }

    void AlvikComponent::maze_turn_left()
    {
        // turn left - immediate, smooth turn enough
        //this->rotate(90);
        //set_wheels_speed(0, this->maze_crawling_speed_ / 2);
        set_wheels_speed(- (this->maze_crawling_speed_ / 4), this->maze_crawling_speed_ / 2);
        this->maze_saved_cycle_counter_ = this->cycle_;
        this->maze_crawling_state_ = CRAWLING_INTERSECTION ;
        this->intersection_dir_ = INTERSECTION_LEFT;
        this->maze_solution_.push_back('L');
        ESP_LOGD(TAG, "Left turn with confidence: %.2f", this->maze_left_turn_confidence);
        this->maze_left_turn_confidence = 0;
        this->maze_right_turn_confidence = 0;
        this->maze_dead_end_confidence   = 0;
        this->maze_straight_continue_confidence_inverze_ = 1;
        this->maze_turn_start_yaw_ = this->robot_pose[2];
        this->maze_solution_trust_confidence_ = 1;
    }

    void AlvikComponent::maze_turn_back()
    {
        //turn back - sharp, keeping the center unmoved
        //this->rotate(180);
        set_wheels_speed(this->maze_crawling_speed_ / 2, -(this->maze_crawling_speed_ / 2) );
        this->maze_saved_cycle_counter_ = this->cycle_;
        this->maze_crawling_state_ = CRAWLING_INTERSECTION ;
        this->intersection_dir_ = INTERSECTION_DEAD_END;
        this->maze_solution_.push_back('B');
        ESP_LOGD(TAG, "U turn with confidence: %.2f", this->maze_dead_end_confidence);
        this->maze_left_turn_confidence = 0;
        this->maze_right_turn_confidence = 0;
        this->maze_dead_end_confidence   = 0;
        this->maze_straight_continue_confidence_inverze_ = 1;
        this->maze_turn_start_yaw_ = this->robot_pose[2];
        this->maze_solution_trust_confidence_ = 1;
    }

    void AlvikComponent::maze_keep_straight()
    {
        this->maze_solution_.push_back('S');
        ESP_LOGD(TAG, "Keep straight with straight confidence: %.2f, and right turn confidence: %.2f", this->maze_straight_continue_confidence_inverze_, this->maze_right_turn_confidence);
        this->maze_left_turn_confidence = 0;
        this->maze_right_turn_confidence = 0;
        this->maze_dead_end_confidence   = 0;
        this->maze_straight_continue_confidence_inverze_ = 1;
        this->maze_solution_trust_confidence_ = 1;
    }

    void AlvikComponent::maze_are_we_there_yet()
    {
        float left_turn_conf = this->maze_left_turn_confidence;
        float right_turn_conf = this->maze_right_turn_confidence;
        float dead_end_conf = this->maze_dead_end_confidence;
        char c;
        
        if ((right_turn_conf>= 20) & (left_turn_conf >= 20))
        {
            this->maze_crawling_state_ = CRAWLING_SOLVED ;
        }
        else if (this->maze_solved_)
        {
            //What is the next move from the solution?
            if (this->maze_solution_saved_.length() > this->maze_solution_.length())
            {
                c = this->maze_solution_saved_[this->maze_solution_.length()];
            }
            else { c = ' '; }
            
            if ((left_turn_conf >= 1) & this->maze_left_turn_confidence_decreasing_)
            {
                if ((c == 'L') | (this->maze_solution_trust_confidence_ <= 0))
                {
                    this->maze_turn_left();
                    if (this->maze_solution_trust_confidence_ <= 0)
                    {
                        this->maze_solved_ = false;
                        ESP_LOGD(TAG,"Maze known solution is invalidated, and turned left");
                    }
                }
                else
                {
                    if ((c == 'R') & (right_turn_conf < 0.5)) { this->maze_solution_trust_confidence_ -= 0.1;}
                    if ((c == 'S') & (dead_end_conf > 0)) { this->maze_solution_trust_confidence_ -= 0.1;}
                }
            }
            if (right_turn_conf >= 1)
            {
                if (c == 'R') { this->maze_turn_right(); }
                else if ((dead_end_conf >= 1) & (this->maze_solution_trust_confidence_ <= 0))
                {
                    this->maze_turn_right();
                    this->maze_solved_ = false;
                    ESP_LOGD(TAG,"Maze known solution is invalidated, and turned right");
                }
                else if (dead_end_conf >= 1) { this->maze_solution_trust_confidence_ -= 0.1; }
            }
            if ((dead_end_conf >= 1) & (left_turn_conf <= 0.5) & (right_turn_conf <= 0.5))
            {
                if (this->maze_solution_trust_confidence_ > 0) { this->maze_solution_trust_confidence_ -= 0.1; }
                else
                {
                    this->maze_turn_back();
                    this->maze_solved_ = false;
                    ESP_LOGD(TAG,"Maze known solution is invalidated, as back turn is detected");
                }
            }
            if (((left_turn_conf >= 1)  | (right_turn_conf > 1)) & this->maze_left_turn_confidence_decreasing_)
            {
                this->maze_straight_continue_confidence_inverze_ -= 0.25;
                if (this->maze_straight_continue_confidence_inverze_ <=0)
                {
                    this->maze_keep_straight();
                    if (c != 'S') { this->maze_solved_ = false; }
                }
            }
        }
        else 
        {
            if ((left_turn_conf >= 1) & this->maze_left_turn_confidence_decreasing_)
            {
                this->maze_turn_left();
            }
            else if ((right_turn_conf >= 1) & (dead_end_conf >= 1))
            {
                this->maze_turn_right();
            }
            else if (dead_end_conf >= 1)
            {
                this->maze_turn_back();
            }
            else if  (right_turn_conf >= 1) // right turn confirmed, but we go straight 
            {
                this->maze_straight_continue_confidence_inverze_ -= 0.1;
                if (this->maze_straight_continue_confidence_inverze_ <=0)
                {
                    this->maze_keep_straight();
                }
            }
        }
    }


    void AlvikComponent::alvik_maze_solver()
    {
        float line_sum;
        line_sum = this->line_sensors[0] + this->line_sensors[1] + this->line_sensors[2];

        switch (this->maze_crawling_state_)
        {
            case CRAWLING_STRAIGHT:
            {
                if (this->maze_crawling_speed_max_ > this->maze_crawling_speed_)
                {
                    this->maze_crawling_speed_ += 0.25;
                    this->alvik_line_follower();
                }
                else
                {
                    if (line_sum > 770) //more than one line is present
                    {
                        if (this->line_sensors[0] > this->line_detection_threshold_ * 2 ) 
                        {
                            if (this->line_sensors[1] > this->line_detection_threshold_ * 2 ) 
                            {
                                this->maze_left_turn_confidence += 0.5;
                            }
                            else
                            {
                                this->maze_left_turn_confidence += 0.25;
                            }
                            //ESP_LOGD(TAG, "Possible left turn detected");
                            //this->maze_solution_.push_back('l');
                            this->maze_left_turn_confidence_decreasing_ = false;
                        }
                        if (this->line_sensors[2] > this->line_detection_threshold_ * 2 ) 
                        {
                            if (this->line_sensors[1] > this->line_detection_threshold_ * 2 ) 
                            {
                                this->maze_right_turn_confidence += 0.5;
                            }
                            else
                            {
                                this->maze_right_turn_confidence += 0.25;
                            }
                            //ESP_LOGD(TAG, "Possible right turn detected");
                            //this->maze_solution_.push_back('r');
                            this->maze_straight_continue_confidence_inverze_ = 2;
                        }
                        //as the control is highly compromized by the intersection, going straight ahead is the best chance to detect if line continues afterwards
                        set_wheels_speed(this->maze_crawling_speed_ / 2, this->maze_crawling_speed_ / 2);
                    }
                    //control line following
                    else if (line_sum > this->line_detection_threshold_)
                    {
                        //check if this is an intersection
                        this->maze_left_turn_confidence_decreasing_ = true;
                        this->maze_are_we_there_yet();
                        
                        this->maze_left_turn_confidence -= 0.1;
                        this->maze_right_turn_confidence -= 0.1;
                        this->maze_dead_end_confidence   -= 0.1;
                        if (this->maze_left_turn_confidence < 0) {this->maze_left_turn_confidence = 0;}
                        if (this->maze_right_turn_confidence < 0) {this->maze_right_turn_confidence = 0;}
                        if (this->maze_dead_end_confidence < 0) {this->maze_dead_end_confidence = 0;}
                        if (this->maze_crawling_state_ == CRAWLING_STRAIGHT)
                        {
                            this->alvik_line_follower();
                        }
                    }
                    else
                    {
                        //as the probably dead-end, no line in sight, nothing to control for, 
                        this->maze_dead_end_confidence += 0.25;
                        this->maze_left_turn_confidence -= 0.1;
                        this->maze_left_turn_confidence_decreasing_ = true;
                        this->maze_right_turn_confidence -= 0.1;
                        
                        this->maze_are_we_there_yet();
                        //ESP_LOGD(TAG, "Possible dead-end detected");
                        //this->maze_solution_.push_back('b');
                        //set_wheels_speed(this->maze_crawling_speed_, this->maze_crawling_speed_);
                    }
                //this->alvik_line_follower();
                }
                break;
            }
            case CRAWLING_INTERSECTION:
            {
                //check if the currently detected line disappeared from sight going straight ahead is the best chance to detect if this is only a glitch in the measurement
                if ((this->line_sensors[1] <  (this->line_detection_threshold_ / 1.5)) | (abs(this->maze_turn_start_yaw_ - this->robot_pose[2]) > 45))
                {
                    this->maze_turn_started_confidence += 0.2;
                    if (this->maze_turn_started_confidence > 1)
                    {
                        this->maze_crawling_state_ = CRAWLING_TURNING;
                        ESP_LOGD(TAG, "Turning");
                        this->maze_turn_started_confidence = 0;
                        if (this->joints_velocity[0] == 0) // Left turn
                        {
                            set_wheels_speed(- (this->maze_crawling_speed_ / 2), this->maze_crawling_speed_ / 2);
                        }
                        if (this->joints_velocity[1] == 0) // Right turn
                        {
                            set_wheels_speed(this->maze_crawling_speed_ / 2, - (this->maze_crawling_speed_ / 2));
                        }

                    }
                }
                break;
            }
            case CRAWLING_TURNING:
            {
                //check if the turnng brings the line to the center
                float intersection_angle;
                intersection_angle = 90; //left or right
                if (this->intersection_dir_ == INTERSECTION_DEAD_END)
                {
                    intersection_angle = 180;
                }
                
                if ((this->line_sensors[1] > this->line_detection_threshold_ * 1.5) | (abs(this->maze_turn_start_yaw_ - this->robot_pose[2]) > intersection_angle))
                {
                    this->brake();
                    this->line_follower_centoid_integral_ = 0;
                    this->maze_crawling_state_ = CRAWLING_STRAIGHT;
                    this->maze_crawling_speed_ = 0;
                }
                break;
            }
            case CRAWLING_SOLVED:
            {
                this->brake();
                this->maze_solved_ = true;
                this->maze_solver_start_ = false;
                this->maze_crawling_state_ = CRAWLING_STRAIGHT;
                this->maze_left_turn_confidence = 0;
                this->maze_right_turn_confidence = 0;
                this->maze_dead_end_confidence   = 0;
                this->maze_straight_continue_confidence_inverze_ = 1;
                if (this->maze_solution_.length() > 0)
                {
                    this->maze_optimize_solution();
                }
                this->maze_solution_saved_ = this->maze_solution_;
                
                break;
            }
            default:
                break;
        }
    }


    void AlvikComponent::maze_optimize_solution()
    {
        std::string maze_optimized_solution;
        std::string maze_solution_copy;
        std::string replaceable_substring;
        bool solution_optimimized;
        char c;
        int backturn_position;

        solution_optimimized = false;
        maze_solution_copy = this->maze_solution_;
        
        if (this->maze_solved_)
        {
            while (solution_optimimized | maze_optimized_solution.empty())
            {
                if (maze_solution_copy.find('B') != std::string::npos)
                {
                    backturn_position = maze_solution_copy.find('B'); // found
                    if (backturn_position != 0) //not the first nor the last action is a backturn - but this is also not expected
                    {
                        maze_optimized_solution = maze_solution_copy.substr(0, backturn_position - 1 );
                        replaceable_substring =  maze_solution_copy.substr(backturn_position - 1, 3);
                        if (replaceable_substring == "LBR") {maze_optimized_solution.push_back('B');}
                        if (replaceable_substring == "LBS") {maze_optimized_solution.push_back('R');}
                        if (replaceable_substring == "RBL") {maze_optimized_solution.push_back('B');}
                        if (replaceable_substring == "SBL") {maze_optimized_solution.push_back('R');}
                        if (replaceable_substring == "SBS") {maze_optimized_solution.push_back('B');}
                        if (replaceable_substring == "LBL") {maze_optimized_solution.push_back('S');}
                        maze_optimized_solution.append( maze_solution_copy.substr(backturn_position + 2 ));
                        ESP_LOGD(TAG, "Optimized solution: %s", maze_optimized_solution.c_str());
                        solution_optimimized = true;
                    }
                    else
                    {
                       solution_optimimized = false;   
                       maze_optimized_solution = maze_solution_copy;
                    }
                }
                else
                {
                    solution_optimimized = false; // not found
                    maze_optimized_solution = maze_solution_copy;
                }
                maze_solution_copy = maze_optimized_solution;
                this->maze_solution_ = maze_optimized_solution;
                ESP_LOGD(TAG, "Optimized solution: %s", maze_optimized_solution.c_str());
                ESP_LOGD(TAG, "Remaining input : %s", maze_solution_copy.c_str());
            }
        }
    }

    void AlvikComponent::alvik_constant_direction_control()
    {
        float angle_error;
        float Kp;
        float diff_speed;

        Kp = this->constant_direction_gain_;

        //MÉR, IRÁNY
        read_compass_data();

        //HIBA
        angle_error = this->constant_direction_target_angle_ - this->compass_angle;
        if (angle_error > 180) {angle_error = angle_error - 360;}
        if (angle_error < -180) {angle_error = angle_error + 360;}

        //KERÉKSEBESSÉG KÜLÖNBSÉG
        diff_speed = 0;
        if (abs(angle_error) > this->constant_direction_tolerance_angle_) { diff_speed = angle_error * Kp;}
        if (diff_speed > MOTOR_MAX_RPM) { diff_speed = MOTOR_MAX_RPM; }
        if (diff_speed < -MOTOR_MAX_RPM) { diff_speed = -MOTOR_MAX_RPM; }

        //VEZÉREL
        set_wheels_speed(-diff_speed, diff_speed);
    }

    void AlvikComponent::alvik_follow_control()
    {
        //tuning parameters
        float target_distance;
        float Kp;
        float K_horizontal;
        float distance_tolerance;

        //internal variables
        float error_distance;
        float common_speed = 0;
        float diff_speed = 0;
        float min_distance = 1500.0;
        float l, cl, c, cr, r;
        float sum_weight, sum_values, centoid;
        float max_distance = 1500.0;
        
        distance_tolerance = this->follow_tolerance_;
        target_distance =  this->follow_target_;
        Kp = this->follow_Kp_;
        K_horizontal = this->follow_K_horizontal_;

        ESP_LOGVV(TAG, "target distance is: %.1f, tolerance is: %.1f, Kp is: %.1f, K_horizontal is: %.1f", target_distance, distance_tolerance, Kp, K_horizontal);
        
        if (this->distances_updated | 1)
        {
            if (distances[0] > max_distance * 0.7) { l = 0; }
            else
            {
                if (min_distance > distances[0]) { min_distance = distances[0]; }
                l = max_distance - distances[0];
            }
    
            if (distances[1] > max_distance * 0.7) { cl = 0; }
            else
            {
                if (min_distance > distances[1]) { min_distance = distances[1]; }
                cl = max_distance - distances[1];
            }
    
            if (distances[2] > max_distance * 0.7) { c = 0; }
            else
            {
                if (min_distance > distances[2]) { min_distance = distances[2]; }
                c = max_distance - distances[2];
            }
    
            if (distances[3] > max_distance * 0.7) { cr = 0; }
            else
            {
                if (min_distance > distances[3]) { min_distance = distances[3]; }
                cr = max_distance - distances[3];
            }
            
            if (distances[4] > max_distance * 0.7) { r = 0; }
            else
            {
                if (min_distance > distances[4]) { min_distance = distances[4]; }
                r = max_distance - distances[4];
            }
    
            if (abs(distances[0] - min_distance) > 50) { l = 0; } 
            if (abs(distances[1] - min_distance) > 50) { cl = 0; } 
            if (abs(distances[2] - min_distance) > 50) { c = 0; } 
            if (abs(distances[3] - min_distance) > 50) { cr = 0; } 
            if (abs(distances[4] - min_distance) > 50) { r = 0; } 

            error_distance = min_distance - target_distance;
                        

            //calculate axial (difference) wheel speeds
            sum_weight = l + cl + c + cr + r;
            sum_values = l + cl*2.0 + c*3.0 + cr*4.0 + r*5.0;
            K_horizontal = K_horizontal * (max_distance - min_distance) / max_distance;
            if (sum_weight != 0) { centoid = sum_values / sum_weight - 3.0; }
            else { centoid = 0.0; }
            this->centoid_filt = (centoid + 2 * this->centoid_filt) / 3.0;
            if (abs(this->centoid_filt) > this->centoid_tolerance_) { diff_speed = this->centoid_filt * K_horizontal;}

            // calculate longitudinal (common) wheel speeds
            error_distance = min_distance - target_distance;
            if (abs(error_distance) > distance_tolerance)
            {
                common_speed = error_distance * Kp;
                if (common_speed >= MOTOR_MAX_RPM - abs(diff_speed)) { common_speed = MOTOR_MAX_RPM - abs(diff_speed); }
                if (common_speed <= - MOTOR_MAX_RPM + abs(diff_speed)) { common_speed = - MOTOR_MAX_RPM + abs(diff_speed); }
            }

            ESP_LOGVV(TAG, "Centoid is: %.1f, diff_speed is: %.1f, Min distance is: %.1f", this->centoid_filt, diff_speed, min_distance);
            ESP_LOGVV(TAG, "Error distance is: %.1f, Centoid is: %.1f, Common speed is: %.1f, diff_speed is: %.1f, Min distance is: %.1f", error_distance, centoid, common_speed, diff_speed, min_distance);

            //set final requested wheel speeds
            this->wheel_speeds[0] = common_speed + diff_speed;
            this->wheel_speeds[1] = common_speed - diff_speed;
            set_wheels_speed(this->wheel_speeds[0], this->wheel_speeds[1]);

            this->distances_updated = false;
        }
    }

    void AlvikComponent::external_supply_measurement(bool ison)
    {
        uint8_t batt_regs[] = {0, 0};
        uint8_t icreg = 0x06;
        uint16_t battery_val = 0;
        uint16_t battery_soc = 0;
        
        if (this->battery < CHARGE_THRESHOLD)
        {
            if (this->cycle_ % 200 == 0) {  this->red_led_pin_->digital_write(false);}
            if (this->cycle_ % 200 == 100) {  this->red_led_pin_->digital_write(true);}
        }

        if (this->cycle_ == 2)
            {
                this->nano_pin_->digital_write(true);
                //this->blue_led_pin_->digital_write(false);
                //this->green_led_pin_->digital_write(true);
                ESP_LOGVV(TAG, "Nano pin set to high");
            }
        if (this->cycle_ == 50)
        {
            this->i2c_switch1_pin_->pin_mode(gpio::FLAG_OUTPUT);
            this->i2c_switch2_pin_->pin_mode(gpio::FLAG_OUTPUT);            
            this->i2c_switch1_pin_->digital_write(true);
            this->i2c_switch2_pin_->digital_write(true);
            ESP_LOGVV(TAG, "I2C switch takeover initiated");
        }
        if (this->cycle_ == 60)
        {
            this->i2c_switch1_pin_->digital_write(false);
            this->i2c_switch2_pin_->digital_write(false);
            
            ESP_LOGVV(TAG, "I2C recover initiated");
            
            this->i2c_switch1_pin_->pin_mode(gpio::FLAG_NONE);
            this->i2c_switch2_pin_->pin_mode(gpio::FLAG_NONE); 
            
            if ((this->battery_sensor_->write(&icreg, 1, false) != i2c::ERROR_OK) || !this->battery_sensor_->read_bytes_raw(batt_regs, 2)) {
                ESP_LOGE(TAG, "I2C recover failed");
                this->cycle_ = 0;
            }
            else
            {
                battery_val = encode_uint16(batt_regs[1], batt_regs[0]);
                battery_soc = (int)((float)battery_val * 0.00390625);
                this->battery = battery_soc;
                ESP_LOGD(TAG, "Battery read:  %d, %d", battery_val, battery_soc);
            }
        }
        if (this->cycle_ == 400) 
        {
            this->cycle_=0;
            this->battery_is_charging = (battery > 0) ? true : false;
            if (this->battery_sensor_ != nullptr)
            {
                this->battery_sensor_->publish_state(this->battery);
            }
            if (this->battery >= CHARGE_THRESHOLD)
            {
                this->green_led_pin_->digital_write(false);
                this->red_led_pin_->digital_write(false);
            }
        }

        if (ison)
        {
            this->green_led_pin_->digital_write(true);
            this->red_led_pin_->digital_write(true);
            this->nano_pin_->digital_write(false);
            this->cycle_ = 0;
            this->set_alvik_state(ALVIK_STARTUP);
        }
    }


    void AlvikComponent::do_one_item_from_command_list(uint32_t now)
    {
        bool orientation_correction_needed = false;
        float orientation_error;
        float this_yaw =0;
        if ( (( now - this->last_command_time_) >= 3 * 950 ) & (( now - this->last_command_time_) < 3 * 1000 ) )
        {
            if (this->orientation_correction_enabled & (abs(this->compass_angle - this->angle_at_offset) < 20 ) )
            {
                this_yaw = this->compass_angle; //this->orientation[2];
                while (this_yaw < 0) { this_yaw += 360; }
                while (this_yaw > 360) { this_yaw -= 360; }
                if (this_yaw == 360) { this_yaw = 0; }
                if (this->yaw_est == 360) { this->yaw_est = 0; }
                orientation_error = this_yaw - this->yaw_est;
                if (abs(orientation_error) > 8)
                {
                    orientation_correction_needed = true;
                    this->rotate(-orientation_error);
                    this->last_command_time_ = now;
                }
            }
            
        }
        
        if ((this->alvik_command_list_.length() != 0 ) & ((now - this->last_command_time_) >= 3 * 1000) )
        {
            if (!orientation_correction_needed)
            {
                //do user requests
                char c = this->alvik_command_list_[0];
                if (c == 0x65) // e
                {
                    this->move(this->forward_move_distance_);
                }
                else if (c == 0x68) // h
                {
                    this->move(-this->forward_move_distance_);
                }
                else if (c == 0x6a) // j
                {
                    this->rotate(-this->turn_degree_);
                    this->yaw_est -= this->turn_degree_;
                    if (yaw_est < 0) this->yaw_est += 360;
                }
                else if (c == 0x62) // b
                {
                    this->rotate(this->turn_degree_);
                    this->yaw_est += this->turn_degree_;
                    if (yaw_est > 360) this->yaw_est -= 360;
                }
                //clear the fulfilled request
                if (this->alvik_command_list_.length() > 1)
                {
                    this->alvik_command_list_ = this->alvik_command_list_.substr(1);
                }
                else
                {
                    this->alvik_command_list_.clear();
                    this->change_alvik_left_right_leds(0xff, false);
                }
            }
            this->last_command_time_ = now;
        }
    }

    bool AlvikComponent::read_message(){                                               //it is private
      while (this->available()){
        this->b = this->read();
        this->packeter->buffer.push(b);
        if (this->packeter->checkPayload()){
            ESP_LOGVV(TAG, "Incoming Message found!"); 
            return true;
        }
      }
      return false;
    }
    
    int AlvikComponent::parse_message(){                                               //it is private
        //while (this->packeter->checkPayload())
        {
              this->code = this->packeter->payloadTop();
              uint32_t now = millis();
              switch(code){
                // get ack code
                case 'x':
                  if (this->waiting_ack == NO_ACK)
                  {
                    this->packeter->unpacketC1B(this->code, last_ack);
                    last_ack = 0x00;
                  } 
                  else 
                  {
                    this->packeter->unpacketC1B(this->code, last_ack);
                  }
                  ESP_LOGVV(TAG, "Acknowledgement recieved!");
                  break;
               
                // motion
            
                // get joints velocity in RPM
                case 'j':
                  this->packeter->unpacketC2F(this->code, joints_velocity[0], joints_velocity[1]);
                  break;
            
                // get joints position in degrees
                case 'w':
                  this->packeter->unpacketC2F(this->code, joints_position[0], joints_position[1]);
                  break;
            
                // get robot linear and angular velocities in mm/s and degrees/s
                case 'v':
                  this->packeter->unpacketC2F(this->code, robot_velocity[0], robot_velocity[1]);
                  break;
            
                // get robot pose in mm and degrees, x, y, theta
                case 'z':
                  this->packeter->unpacketC3F(this->code, robot_pose[0], robot_pose[1], robot_pose[2]);
                  break;
            
            
                // sensors
            
                // get line follower sensors, low is white - high is black: Left, Center, Right
                case 'l':
                  this->packeter->unpacketC3I(this->code, line_sensors[0], line_sensors[1], line_sensors[2]);
                  break;
            
                // get colors: red, green, blue
                case 'c':
                  this->packeter->unpacketC3I(this->code, color_sensor[0], color_sensor[1], color_sensor[2]);
                  break;
                
                // get orientation in deg: roll, pitch, yaw
                case 'q':
                  this->packeter->unpacketC3F(this->code, orientation[0], orientation[1], orientation[2]);
                  break;
            
                // get tilt and shake
                case 'm':
                  this->packeter->unpacketC1B(this->code, move_bits);
                  break;
            
                // get imu data in g and deg/s: aX, aY, aZ, gX, gY, gZ
                case 'i':
                  this->packeter->unpacketC6F(this->code, imu[0], imu[1], imu[2], imu[3], imu[4], imu[5]);
                  break;       
                
                // get data from ToF in mm: L, CL, C, CR, R, B, T
                case 'f':
                  this->packeter->unpacketC7I(this->code, distances[0], distances[1], distances[2], distances[3], distances[4], distances[5], distances[6]);
                  this->distances_updated = true;
                  ESP_LOGVV(TAG,"distance message received : %u", now);
                  break;    
            
                // get data from touch pads: any, ok, delete, center, left, down, right, up
                case 't':
                    this->packeter->unpacketC1B(this->code, touch);
                    if ((millis() - this->last_command_received_time_) >= 500)
                    {
                        // Any:    0b00000001;
                        // OK:     0b00000010; o: OK
                        // Cancel: 0b00000100; x: Cancel
                        // Center: 0b00001000; c: Center
                        // Up:     0b00010000; e: Forward (Elo"re)
                        // Left:   0b00100000; b: Turn Left (Balra)
                        // Down:   0b01000000; h: Backwards (Ha'tra)
                        // Right   0b10000000; j: Turn Right (Jobbra)
                        if (touch & 0b00000010)
                            this->ok_button_action();
                        if (touch & 0b00000100)
                            this->cancel_button_action();
                        if (touch & 0b00001000)
                            this->center_button_action();
                        if (touch & 0b00010000)
                            this->forward_button_action();
                        if (touch & 0b00100000)
                            this->left_button_action();
                        if (touch & 0b01000000)
                            this->backwards_button_action();
                        if (touch & 0b10000000)
                            this->right_button_action();
                        if (touch & 0b00000001)
                            this->last_command_received_time_ = millis();
                    }
                    break;   
                
                // get fw_version: Up, Mid, Low
                case 0x7E:
                  this->packeter->unpacketC3B(this->code, fw_version[0], fw_version[1], fw_version[2]);
                  if (fw_version == lib_version)
                      { this->set_stm32_fw_compatible(true); }
                  else
                      { this->set_stm32_fw_compatible(true); }
                  break;
            
                // get battery parcentage: state of charge
                case 'p':
                  this->packeter->unpacketC1F(this->code, battery);
                  this->battery_is_charging = (battery > 0) ? true : false;
                  battery = abs(battery);
                  break;
            
                // nothing is parsed, the command is newer to this library
                default:
                  return -1;
              }
        }
        return 0;
    }

    void AlvikComponent::read_compass_data()
    {
        uint8_t raw_data[6];
        int16_t raw_x;
        int16_t raw_y;
        int16_t raw_z;
        float x;
        float y;
        float z;
        bool learn_happened = false;
        float normalized_y;
        float normalized_z;
        
        if ((this->compass_sensor_->write(&M_REG_MEASUREMENT, 1, false) != i2c::ERROR_OK) || !this->compass_sensor_->read_bytes_raw(raw_data, M_REG_MEASUREMENT_LEN  )) 
        {
            ESP_LOGE(TAG, "Unable to read compass data");
        }
        else 
        {
            raw_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
            raw_z = (int16_t)((raw_data[2] << 8) | raw_data[3]);
            raw_y = (int16_t)((raw_data[4] << 8) | raw_data[5]);
            ESP_LOGV(TAG, "Compass data read %d, %d, %d", raw_x, raw_y, raw_z);
            x = (float)raw_x / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
            y = (float)raw_y / _lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
            z = (float)raw_z / _lsm303Mag_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;

            if (x > this->compass_x_max) 
            {
                this->compass_x_max = x; 
                learn_happened = true;
            }
            if (x < this->compass_x_min) 
            {
                this->compass_x_min = x; 
                learn_happened = true;
            }
            if (y > this->compass_y_max) 
            {
                this->compass_y_max = y; 
                learn_happened = true;
            }
            if (y < this->compass_y_min) 
            {
                this->compass_y_min = y; 
                learn_happened = true;
            }
            if (z > this->compass_z_max) 
            {
                this->compass_z_max = z;
                learn_happened = true;
            }
            if (z < this->compass_z_min) 
            {
                this->compass_z_min = z;
                learn_happened = true;
            }

            if (learn_happened)
            {
                ESP_LOGV(TAG, "Compass x min, max %.2f, %.2f", this->compass_x_min, this->compass_x_max);
                ESP_LOGV(TAG, "Compass y min, max %.2f, %.2f", this->compass_y_min, this->compass_y_max);
                ESP_LOGV(TAG, "Compass z min, max %.2f, %.2f", this->compass_z_min, this->compass_z_max);
            }

            this->compass_x_offset = (this->compass_x_max + this->compass_x_min) / 2;
            this->compass_y_offset = (this->compass_y_max + this->compass_y_min) / 2;
            this->compass_z_offset = (this->compass_z_max + this->compass_z_min) / 2;
            
            this->compass_measurements[0] = x - this->compass_x_offset;
            this->compass_measurements[1] = y - this->compass_y_offset;            
            this->compass_measurements[2] = z - this->compass_z_offset;

            normalized_y = this->compass_measurements[1] / abs(this->compass_y_max - this->compass_y_min);
            normalized_z = this->compass_measurements[2] / abs(this->compass_z_max - this->compass_z_min);
        
            this->compass_angle = - (atan2(normalized_y, normalized_z) * 180) / PI;
            if (this->compass_angle < 0) { this->compass_angle += 360; }
        
        }
    }
    
    void AlvikComponent::center_button_action()
    {
        //alvik_command_list_.push_back('c'); // c: Center
        switch(this->alvik_action_)
        {
            case ACTION_PERFORM_COMMAND_LIST:
                this->set_alvik_action(ACTION_COLLECT_COMMAND_LIST);
                break;
            case ACTION_COLLECT_COMMAND_LIST:
                this->set_alvik_action(ACTION_FOLLOW);
                break;
            case ACTION_FOLLOW:
                this->set_alvik_action(ACTION_CONSTANT_DIRECTION);
                break;
            case ACTION_CONSTANT_DIRECTION:
                this->set_alvik_action(ACTION_MAZE_SOLVER);
                break;
            case ACTION_MAZE_SOLVER:
                this->set_alvik_action(ACTION_PERFORM_COMMAND_LIST);
                break;
            
            default:
                this->set_alvik_action(ACTION_PERFORM_COMMAND_LIST);
                break;
        }
    }

    void AlvikComponent::cancel_button_action()
    {
        //alvik_command_list_.push_back('x'); // x: Cancel
        this->alvik_command_list_.clear();
        this->maze_solution_.clear();
        this->maze_solved_ = false;
        this->change_alvik_left_right_leds(LEFT_RED + RIGHT_RED, true);
        this->line_follower_centoid_integral_ = 0;
        this->maze_solver_start_ = false;
    }
    void AlvikComponent::ok_button_action()
    {
        switch(this->alvik_action_)
        {
            case ACTION_COLLECT_COMMAND_LIST:
        //alvik_command_list_.push_back('o'); // o: OK
                this->set_alvik_action(ACTION_PERFORM_COMMAND_LIST);
                this->led_state= 0;
                this->yaw_est = this->orientation[2];
                this->change_alvik_left_right_leds(LEFT_BLUE + RIGHT_BLUE, true);
                break;
            case ACTION_FOLLOW:
                this->follow_start_ = true;
                break;
            case ACTION_CONSTANT_DIRECTION:
                this->direction_control_start_ = true;
                break;
            case ACTION_MAZE_SOLVER:
                this->maze_solver_start_ = true;
                this->maze_crawling_speed_  = 0;
                this->maze_intersection_counter_ = 0;
                this->maze_solution_trust_confidence_ = 1;
                this->maze_solution_.clear();
                if (this->maze_solved_)
                {
                    ESP_LOGD(TAG,"Maze solving started with known solution");
                }
                else
                {
                    ESP_LOGD(TAG,"Maze solving started without known solution");
                }
                break;
            default:
                break;
        }
    }
    void AlvikComponent::forward_button_action()
    {
        alvik_command_list_.push_back('e'); // e: Forward (Elo"re)
        if (this->alvik_action_ == ACTION_CONSTANT_DIRECTION)
        {
            this->constant_direction_target_angle_ = 0;
            this->constant_direction_target_number_->publish_state(constant_direction_target_angle_);
        }
    }
    void AlvikComponent::backwards_button_action()
    {
        alvik_command_list_.push_back('h'); // h: Backwards (Ha'tra)
    }
    void AlvikComponent::left_button_action()
    {
        alvik_command_list_.push_back('b'); // b: Turn Left (Balra)
    }
    void AlvikComponent::right_button_action()
    {
        alvik_command_list_.push_back('j'); // j: Turn Right (Jobbra)
    }

    void AlvikComponent::move(const float distance){
        this->msg_size = this->packeter->packetC1F('G', distance);
        this->write_array(this->packeter->msg, this->msg_size);
        this->waiting_ack = 'M';
        ESP_LOGD(TAG, "Move message sent!");
    }

    void AlvikComponent::rotate(const float angle){
        this->msg_size = this->packeter->packetC1F('R', angle);
        this->write_array(this->packeter->msg, this->msg_size);
        this->waiting_ack = 'R';
        ESP_LOGD(TAG, "Rotate message sent!");
    }

    void AlvikComponent::set_servo_positions(const uint8_t a_position, const uint8_t b_position){
        servo_positions[0] = a_position;
        servo_positions[1] = b_position;
        this->msg_size = this->packeter->packetC2B('S', a_position, b_position);
        this->write_array(this->packeter->msg, this->msg_size);
        ESP_LOGD(TAG, "Servo positions set to [%d,%d]!", a_position, b_position);
    }

    void AlvikComponent::reset_pose(const float x, const float y, const float theta){
        this->msg_size = this->packeter->packetC3F('Z', x, y, theta);
        this->write_array(this->packeter->msg, this->msg_size); 
    }

    void AlvikComponent::set_wheels_speed(const float left, const float right)
    {
          this->msg_size = this->packeter->packetC2F('J', left, right);
          this->write_array(this->packeter->msg, this->msg_size);
    }

    void AlvikComponent::set_behaviour(const uint8_t behaviour){
      this->msg_size = this->packeter->packetC1B('B', behaviour);
      this->write_array(this->packeter->msg, this->msg_size);
    }

    void AlvikComponent::change_alvik_left_right_leds(uint8_t change_led_state, bool onoff)
    {
        uint8_t a_led_state;
        if (onoff)
        {
            a_led_state = this->led_state | change_led_state;
        }
        else
        {
            a_led_state = this->led_state & (~change_led_state);
        }
        if (a_led_state != this->led_state)
        {
            ESP_LOGV(TAG, "LEDs requested to %x, %x -> %x, onoff: %d", a_led_state, change_led_state, this->led_state, onoff);
            this->led_state = a_led_state;
            this->msg_size = this->packeter->packetC1B('L', this->led_state);
            this->write_array(this->packeter->msg, this->msg_size);
        }
    }

    void AlvikComponent::set_alvik_action(int an_action)
    {
        switch(an_action)
        {
            case ACTION_COLLECT_COMMAND_LIST:
                this->change_alvik_left_right_leds(0xff, false);
                this->change_alvik_left_right_leds(LEFT_GREEN + RIGHT_GREEN, true);
                this->brake();
                break;
            case ACTION_FOLLOW:
                this->follow_start_ = false;
                this->centoid_filt = 0;
                this->change_alvik_left_right_leds(0xff, false);
                this->change_alvik_left_right_leds(LEFT_GREEN + LEFT_RED + RIGHT_GREEN + RIGHT_RED, true);
                this->brake();
                break;
            case ACTION_CONSTANT_DIRECTION:
                this->direction_control_start_ = false;
                this->constant_direction_target_angle_ = this->compass_angle;
                this->constant_direction_target_number_->publish_state(constant_direction_target_angle_);
                this->change_alvik_left_right_leds(0xff, false);
                this->change_alvik_left_right_leds(LEFT_BLUE + LEFT_RED + RIGHT_BLUE + RIGHT_RED, true);
                this->brake();
                break;
            case ACTION_MAZE_SOLVER:
                this->maze_solver_start_ = false;
                this->maze_solution_.clear();
                this->maze_solved_ = false;
                this->change_alvik_left_right_leds(0xff, false);
                this->change_alvik_left_right_leds(LEFT_BLUE + LEFT_RED + LEFT_GREEN + RIGHT_BLUE + RIGHT_RED + RIGHT_GREEN, true);
                this->line_follower_centoid_integral_ = 0;
                this->line_follower_centoid_previous_ = 0;
                this->brake();
                break;
            case ACTION_PERFORM_COMMAND_LIST:
                this->change_alvik_left_right_leds(0xff, false);
                this->brake();
                break;
            default:
                this->alvik_action_= ACTION_PERFORM_COMMAND_LIST;
                this->change_alvik_left_right_leds(0xff, false);
                this->brake();
                break;
        }
        this->alvik_action_ = an_action;
    }


    void AlvikComponent::dump_config() {
        ESP_LOGCONFIG(TAG, "AlvikComponent  :");
        ESP_LOGCONFIG(TAG, "   current state  : %d", this->alvik_state_);
        switch (this->alvik_state_)
        {
            case ALVIK_STARTUP:
                ESP_LOGCONFIG(TAG, "       Alvik Nano started up");
                break;
            case ALVIK_HW_RESET:
                ESP_LOGCONFIG(TAG, "       STM32 is reset");
                break;
            case ALVIK_STM32_UP:
                ESP_LOGCONFIG(TAG, "       STM32 is on, communication not yet established");
                break;
            case ALVIK_FIRST_ACK:
                ESP_LOGCONFIG(TAG, "       STM32 is on, communication is set up");
                break;
            case ALVIK_FW_COMPATIBLE:
                ESP_LOGCONFIG(TAG, "       Everything is ready to use");
                break;
            case ALVIK_EXTERNAL_SUPPLY:
                ESP_LOGCONFIG(TAG, "       Battery charging");
                break;
            default:
                ESP_LOGCONFIG(TAG, "       State is unknown");
                break;

        }
        LOG_PIN("  Pin: ", this->stm32_pin_);
        LOG_PIN("  Pin: ", this->nano_pin_);
        LOG_PIN("  Pin: ", this->reset_pin_);

        if ((this->compass_sensor_ != nullptr) & (this->alvik_state_ != ALVIK_EXTERNAL_SUPPLY))
        {
            ESP_LOGCONFIG(TAG, "Compass x min, max %.2f, %.2f", this->compass_x_min, this->compass_x_max);
            ESP_LOGCONFIG(TAG, "Compass y min, max %.2f, %.2f", this->compass_y_min, this->compass_y_max);
            ESP_LOGCONFIG(TAG, "Compass z min, max %.2f, %.2f", this->compass_z_min, this->compass_z_max);
        }
        
        if (this->battery_sensor_ != nullptr)
        {
            ESP_LOGCONFIG(TAG, "   Battery status is : %.0f", this->battery_sensor_->get_state());
        }
    }    

    void AlvikEnableSwitch::write_state(bool state) {}


    void AlvikForwardButton::press_action() { this->parent_->forward_button_action(); } 
    void AlvikBackwardsdButton::press_action() { this->parent_->backwards_button_action(); } 
    void AlvikTurnLeftButton::press_action() { this->parent_->left_button_action(); } 
    void AlvikTurnRightButton::press_action() { this->parent_->right_button_action(); } 
    void AlvikOKButton::press_action() { this->parent_->ok_button_action(); } 
    void AlvikCancelButton::press_action() { this->parent_->cancel_button_action(); } 
    void AlvikCenterButton::press_action() { this->parent_->center_button_action(); } 
    void AlvikResetPoseButton::press_action() { this->parent_->reset_pose(); } 

    void AlvikResetButton::press_action() 
    { 
        this->parent_->set_alvik_state(ALVIK_STARTUP);
        this->parent_->set_cycle(0); 
    } 

    void AlvikForwardDistance::control(float a_distance) { this->parent_->set_forward_move_distance(a_distance); }
    void AlvikTurnDegree::control(float an_angle) { this->parent_->set_turn_degree(an_angle); }
    void AlvikFollowDistance::control(float a_distance) { this->parent_->set_follow_target(a_distance); }
    void AlvikFollowTolerance::control(float a_distance) { this->parent_->set_follow_tolerance(a_distance); }
    void AlvikFollowGainHorizontal::control(float a_gain) { this->parent_->set_follow_K_horizontal(a_gain); }
    void AlvikFollowGainFront::control(float a_gain) { this->parent_->set_follow_Kp(a_gain); }
    void AlvikConstantDirectionGain::control(float a_gain) { this->parent_->set_constant_direction_gain(a_gain); }
    void AlvikConstantDirectionTarget::control(float an_angle) { this->parent_->set_constant_direction_target(an_angle); }
    void AlvikLineFollowerP::control(float a_gain) { this->parent_->set_line_follower_p(a_gain); }
    void AlvikLineFollowerI::control(float a_gain) { this->parent_->set_line_follower_i(a_gain); }
    void AlvikLineFollowerD::control(float a_gain) { this->parent_->set_line_follower_d(a_gain); }
    void AlvikMazeCrawlingSpeed::control(float an_rpm) { this->parent_->set_maze_crawling_speed(an_rpm); }

}  // namespace alvik
}  // namespace esphome
