#ifndef __PORT_H
#define __PORT_H

//右边电机驱动：
#define right_motor_pwm_port                        GPIOA
#define fr_right_motor_pwm_out                      GPIO_Pin_0 //PA0
#define right_motor_port                            GPIOD
#define dir_right_motor_io_out                      GPIO_Pin_0 //PD0
#define break_right_motor_io_out                    GPIO_Pin_1 //PD1
#define enb_right_motor_io_out                      GPIO_Pin_2 //PD2
#define alarm_right_motor_io_input                  GPIO_Pin_3 //PD3
#define pg_right_motor_io_input                     GPIO_Pin_8 //PD8

/*
PA0--fr_right_motor_pwm_out
PD0--dir_right_motor_io_out
PD1--break_right_motor_io_out
PD2--enb_right_motor_io_out
PD3--alarm_right_motor_io_input
PD8--pg_right_motor_io_input
*/

//左边电机驱动：
#define left_motor_pwm_port                         GPIOA
#define fr_left_motor_pwm_out                       GPIO_Pin_1 //PA1
#define left_motor_port                             GPIOD
#define dir_left_motor_io_out                       GPIO_Pin_4 //PD4
#define break_left_motor_io_out                     GPIO_Pin_5 //PD5
#define enb_left_motor_io_out                       GPIO_Pin_6 //PD6
#define alarm_left_motor_io_input                   GPIO_Pin_7 //PD7
#define pg_left_motor_io_input                      GPIO_Pin_9 //PD9

/*
PA1--fr_left_motor_pwm_out
PD4--dir_left_motor_io_out
PD5--break_left_motor_io_out
PD6--enb_left_motor_io_out
PD7--alarm_left_motor_io_input
PD9--pg_left_motor_io_input
*/

//前面方向轮电机驱动：
#define ahead_motor_pwm_port                        GPIOB    
#define fr_ahead_motor_pwm_out                      GPIO_Pin_8  //PB8

#define dir_ahead_motor_port                        GPIOB 
#define dir_ahead_motor_io_out                      GPIO_Pin_9  //PB9

#define enb_ahead_motor_port                        GPIOB
#define enb_ahead_motor_io_out                      GPIO_Pin_10 //PB10

#define detected_ahead_motor_port                   GPIOE
#define detected_left_ahead_motor_io_input          GPIO_Pin_12 //PE12
#define detected_middle_ahead_motor_io_input        GPIO_Pin_13 //PE13
#define detected_right_ahead_motor_io_input         GPIO_Pin_14 //PE14

/*
PB8--fr_ahead_motor_pwm_out
PB9--dir_ahead_motor_io_out
PB10--enb_ahead_motor_io_out
PE12--detected_left_ahead_motor_io_input
PE13--detected_middle_ahead_motor_io_input
PE14--detected_right_ahead_motor_io_input
*/

//球包箱体电机驱动：
#define  run_dir_port                                      GPIOB
#define  run1_ballbag_motor_io_out                         GPIO_Pin_6 //PB6
#define  run2_ballbag_motor_io_out                         GPIO_Pin_7 //PB7

#define  current_ballbag_motor_ad_port             				 GPIOC
#define  current_running_ballbag_motor_ad                  GPIO_Pin_0 //PC0  这个还没有写
#define  current_break_ballbag_motor_ad                    GPIO_Pin_3 //PC3

#define  detected_ballbag_port                             GPIOE
#define  detected_center_ballbag_motor_io_input            GPIO_Pin_7 //PE7
#define  detected_limited_ahead_ballbag_motor_io_input     GPIO_Pin_8 //PE8
#define  detected_ahead_ballbag_motor_io_input             GPIO_Pin_9 //PE9
#define  detected_limited_back_ballbag_motor_io_input      GPIO_Pin_10 //PE10
#define  detected_back_ballbag_motor_io_input              GPIO_Pin_11 //PE11

#define  break_port                                        GPIOB
#define  break_ballbag_motor_io_out                        GPIO_Pin_13 //PE13

#define  button_ballbag_port                               GPIOG
#define  up_button_ballbag_motor_io                        GPIO_Pin_8 //PG8
#define  down_button_ballbag_motor_io                      GPIO_Pin_7 //PG7

/*
PB6--up_ballbag_motor_io_out
PB7--down_ballbag_motor_io_out

PC0--current_ballbag_motor_ad
PC3--current_break_ballbag_motor_ad
PE7--detected_center_ballbag_motor_io_input
PE8--detected_limited_ahead_ballbag_motor_io_input
PE9--detected_ahead_ballbag_motor_io_input
PE10--detected_limited_back_ballbag_motor_io_input
PE11--detected_back_ballbag_motor_io_input
PB13--break_ballbag_motor_io_out
*/

//盖子推杆右：
#define up_electric_push_rod_right_io_out           GPIO_Pin_6 //PC6
#define down_electric_push_rod_right_io_out         GPIO_Pin_7 //PC7

#define current_electric_push_rod_right_ad_port     GPIOC
#define current_electric_push_rod_right_ad          GPIO_Pin_2  //PC2//这三个还没有写
#define cw_pg_electric_push_rod_right_pwm_input     GPIO_Pin_12 //PD12
#define ccw_pg_electric_push_rod_right_pwm_input    GPIO_Pin_13 //PD13

#define off_electric_push_rod_right_io_input        GPIO_Pin_2  //PE2
#define on_electric_push_rod_right_io_input         GPIO_Pin_3  //PE3

/*
PC6--up_electric_push_rod_right_io_out
PC7--down_electric_push_rod_right_io_out
PC2--current_electric_push_rod_right_ad
PD12--cw_pg_electric_push_rod_right_pwm_input
PD13--ccw_pg_electric_push_rod_right_pwm_input
PE2--off_electric_push_rod_right_io_input
PE3--on_electric_push_rod_right_io_input
*/

//盖子推杆左:
#define electric_push_rod_out_port                  GPIOC
#define up_electric_push_rod_left_io_out            GPIO_Pin_8  //PC8
#define down_electric_push_rod_left_io_out          GPIO_Pin_9  //PC9

#define current_electric_push_rod_left_ad_port      GPIOC
#define current_electric_push_rod_left_ad           GPIO_Pin_1  //PC1  这三个还没写
#define cw_pg_electric_push_rod_left_pwm_input      GPIO_Pin_14 //PD14
#define ccw_pg_electric_push_rod_left_pwm_input     GPIO_Pin_15 //PD15

#define electric_push_rod_detected_port             GPIOE
#define off_electric_push_rod_left_io_input         GPIO_Pin_4  //PE4
#define on_electric_push_rod_left_io_input          GPIO_Pin_5  //PE5

/*
PC8--up_electric_push_rod_left_io_out
PC9--down_electric_push_rod_left_io_out
PC1--current_electric_push_rod_left_ad
PD14--cw_pg_electric_push_rod_left_pwm_input
PD15--ccw_pg_electric_push_rod_left_pwm_input
PE4--off_electric_push_rod_left_io_input
PE5--on_electric_push_rod_left_io_input
*/

//MIC MOTOR
#define mic_motor_out1_port   GPIOA
#define mic_motor_io_out1 GPIO_Pin_8 //PA8

#define mic_motor_out2_port   GPIOF
#define mic_motor_io_out2 GPIO_Pin_0 //PF0

#define mic_close_detected_port GPIOG 
#define mic_close_detected_io_input GPIO_Pin_0 

//其他AD检测：
#define AD48V_port                                  GPIOB
#define AD48V                                       GPIO_Pin_0 //PB0

#define throttle_port  														  GPIOB
#define AD_throttle                                 GPIO_Pin_1 //PB1

#define battery_temperature_detected_port           GPIOC
#define battery_temperature_detected                GPIO_Pin_4 //PC4

/*
PB0--AD48V
PB1--AD_throttle
PC4--AD_temperature
*/

//其他IO按键检测控制：
#define car_forward_back_port                       GPIOA
#define car_forward_back_button_io                  GPIO_Pin_4 //PA4 

#define open_car_lid_port                           GPIOA
#define open_car_lid_button_io                      GPIO_Pin_6 //PA6

#define close_car_lid_port                          GPIOA
#define close_car_lid_button_io                     GPIO_Pin_7 //PA7

#define detected_car_lid_port                       GPIOE      //球杆检测
#define detected_car_lid_io_input                   GPIO_Pin_6 //PE6

#define detected_people_seatsensor_port             GPIOG
#define detected_people_seatsensor_io_input         GPIO_Pin_1 //PG1

#define detected_carbreak_port                      GPIOG
#define detected_carbreak_io_input                  GPIO_Pin_2 //PG2 中断

#define stopnow_port                                GPIOG
#define stopnow_button_io                           GPIO_Pin_3 //PG3

#define detected_rain_port                          GPIOG
#define detected_rain_io_input                      GPIO_Pin_4 //PG4

#define detected_obstacle_from_ultrasonic_port      GPIOG
#define detected_obstacle_from_ultrasonic_io_input  GPIO_Pin_5 //PG5

#define switch_auto_manual_drivecar_port            GPIOG
#define switch_auto_manual_drivecar_button_io       GPIO_Pin_6 //PG6

/*
PA4--car_forward_back_button_io
PA6--open_car_lid_button_io
PA7--close_car_lid_button_io
PE6--detected_car_lid_io_input
PG1--detected_people_seatsensor_io_input
PG2--detected_carbreak_io_input
PG3--stopnow_button_io
PG4--detected_rain_io_input
PG5--detected_obstacle_from_ultrasonic_io_input
PG6--switch_auto_manual_drivecar_button_io
*/

//电源控制
#define usb_power_port                             GPIOC
#define switch_usb_power_io_out                    GPIO_Pin_10 //PC10

#define ultrasonic_power_port                      GPIOC
#define switch_ultrasonic_power_io_out             GPIO_Pin_11 //PC11

#define backmotor_power_port                       GPIOB
#define switch_backmotor_power_io_out              GPIO_Pin_11 //PB11

#define aheadmotor_power_port                      GPIOB
#define switch_aheadmotor_power_io_out             GPIO_Pin_12 //PB12

#define ahead_light_power_port                     GPIOB
#define switch_ahead_light_power_io_out            GPIO_Pin_14 //PB14

#define car24v_power_port                          GPIOB       //给我的IO接口pdf上面有，但原理图上没连接
#define switch_car24v_power_io_out                 GPIO_Pin_15 //PB15

/*
PC10--switch_usb_power_io_out
PC11--switch_ultrasonic_power_io_out
PB11--switch_backmotor_power_io_out
PB12--switch_aheadmotor_power_io_out
PB14--switch_ahead_light_power_io_out
PB15--switch_car24v_power_io_out
*/

//通信口：
#define usart_db_rx                                GPIO_Pin_2  //PA2
#define usart_db_tx                                GPIO_Pin_3  //PA3
#define usart_cb_rx                                GPIO_Pin_9  //PA9
#define usart_cb_tx                                GPIO_Pin_10 //PA10
#define USB_DM                                     GPIO_Pin_11 //PA11
#define USB_DP                                     GPIO_Pin_12 //PA12

/*
PA2--usart_db_rx
PA3--usart_db_tx
PA9--usart_cb_rx
PA10--usart_cb_tx
PA11--USB_DM
PA12--USB_DP
*/

//备用：
/*
PF0--reserve_io
PF1--reserve_io
PF2--reserve_io
PF3--reserve_io
PF4--reserve_io
PF5--reserve_io
PF6--reserve_io
PF7--reserve_io
*/

#endif








