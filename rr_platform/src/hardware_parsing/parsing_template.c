/**
 * @file parsing_template.c
 * @author Charles Jenkins
 * @brief File to handle the details of sending data to the hardware
 * @version 0.1
 * @date 2022-04-03
 * 
 * @copyright Copyright (c) 2022
 * 
 * 
 * https://docs.google.com/document/d/1oRHm5xBQiod_YXESQ9omFy5joJqMfeQjY3NCvdzRTjk/edit#
 * Specification overview:
 * Data to send: 
 * 1. Speed sent to the Drive board in the format: “v=$float”
 * 2. Steering angle sent to the Steering board in the format: “A=$float”
 * 
 * Data to receive:
 * 1. Speed and Motor Current from Drive “v=$float I=$float”, target breaking speed “B=$float”
 * 2. Actual braking force from Brake board “F=$float”
 * 3. Steering angle Steering board “A=$float”
 * 4. Estop: "G" - Go, “L” - Limited, “D” - Disabled
 * 5. Manual board velocity “M=$char v=$float” and steering “M=$char A=$float”, the $char is either autonomous: "A or manual: "M". Just “M=$char” when mode changes
 * 6. Battery Monitor (voltage and current) “V=$float I=$float”
 */
