#include <Arduino.h>
#include "proj_types.h"
#include "robot.h"
#include "IRline.h"

extern IRLine_t IRLine;
float RobotVelocity = 0.2;


void control(robot_t& robot)
{
    robot.tis = millis() - robot.tes;

    // Rules for the state evolution
     if(robot.state == 0 && robot.LastTouchSwitch && !robot.TouchSwitch) {
      robot.rel_s = 0; //relaciona com o quanto o robô andou (distância)
      robot.setState(1);

    } else if (robot.state == 1 && robot.TouchSwitch) { //touchswitch é o botão que permite mandar um sinal para ligar 
      robot.setState(2);

    } else if(robot.state == 2 && robot.tis > 100) {
      robot.rel_s = 0;
      robot.setState(3);

    } else if(robot.state == 3 && robot.rel_s < -0.12) {
      robot.rel_theta = 0; //relação com ângulo rodado do robô
      robot.setState(4);

    //} else if(robot.state == 4 && robot.rel_theta > radians(90) && IRLine.total > 1500) {
    } else if(robot.state == 4 && robot.rel_theta > radians(170)) {
      robot.setState(5);
      IRLine.crosses = 0;

    } else if(robot.state == 5 && IRLine.crosses >= 4) {
      robot.rel_s = 0;
      robot.rel_theta = 0;
      robot.setState(6);

    } else if(robot.state == 6 && robot.rel_theta < radians(-70) && IRLine.total > 1500) {
      robot.rel_theta = 0;
      robot.rel_s = 0;
      robot.setState(7);

    } else if(robot.state == 7 && robot.tis > 2000) {
      robot.rel_s = 0;
      IRLine.crosses = 0;
      robot.setState(8);

    }else if(robot.state ==8 && robot.rel_s < -0.12 && robot.tis > 2000) {
      robot.rel_theta = 0;
      IRLine.crosses = 0;
      robot.rel_s = 0;
      robot.setState(9);

    }else if(robot.state ==9 && robot.rel_theta > radians(70))  {
      robot.rel_theta = 0;
      IRLine.crosses = 0;
      robot.rel_s = 0;
      robot.setState(10);

    }else if(robot.state ==10 && IRLine.total > 1500 && IRLine.crosses >=4) {
      IRLine.crosses = 0;
      robot.setState(11);

    }
    
    else if (robot.state == 202 && robot.tis > robot.T1) {
      robot.setState(200);
    }









    // Actions in each state

  // GET THE FIRST BOX 

    if (robot.state == 0) {         // Robot Stoped            
      robot.solenoid_state = 0;
      robot.setRobotVW(0, 0);
    
    } else if (robot.state == 1) {  // Go: Get first box
      robot.solenoid_state = 0;
      robot.followLineLeft(IRLine, RobotVelocity, -0.04);
      // if the request is older than 1000 ms repeat the request

    } else if (robot.state == 2) { // Turn Solenoid On and Get the Box
      robot.solenoid_state = 1;
      robot.followLineLeft(IRLine, RobotVelocity, -0.04);
    
    } else if (robot.state == 3) {  // Go back with the box
      robot.solenoid_state = 1;
      robot.setRobotVW(-0.1, 0);
      
    } else if (robot.state == 4) {  // Turn 180 degrees
      robot.solenoid_state = 1;
      robot.setRobotVW(0, 2.5);
      
    } else if (robot.state == 5) {  // long travel to the box final destination
      robot.solenoid_state = 1;
      robot.followLineRight(IRLine, RobotVelocity, -0.04);
      
    } else if (robot.state == 6) {  // Advance a little then turn to place the box
      robot.solenoid_state = 1;
      if (robot.rel_s < 0.1) robot.followLineRight(IRLine, RobotVelocity, -0.04);
      else if (robot.rel_s < 0.33) robot.followLineLeft(IRLine, RobotVelocity, -0.04);
      else robot.setRobotVW(0.0, -1);
      
    }
     else if (robot.state == 7) { 
      robot.solenoid_state = 1;
      robot.followLineRight(IRLine, 0.2, -0.04); 

    } 
    else if (robot.state == 8) { // Drop the box and go back
      robot.solenoid_state = 0;
      if (robot.solenoid_state == 0 ) {
        robot.setRobotVW(-0.1, 0);  // if solenoid is off, then go back a little bit
        }
      else {
        robot.setRobotVW(0, 0);
        
        }   
    
    }


    // GET THE SECOND BOX 
    
      else if (robot.state == 9) {  // Turn 90 degrees and continue pending to the right side
      robot.solenoid_state = 0;
      robot.followLineRight(IRLine, 0.15, -0.04);
    } 


    else if (robot.state == 10) {
      robot.solenoid_state = 0;
      robot.followLineLeft(IRLine, RobotVelocity, -0.05);
      if (IRLine.crosses > 2 && robot.rel_s < -0.5) robot.followLineRight(IRLine, RobotVelocity, -0.04);
      

    }
    //  else if (robot.state == 9) {  // Turn 90 degrees and continue pending to the right side
    //   robot.solenoid_state = 0;
    //   robot.followLineRight(IRLine, 0.1, -0.04);
    //   if (robot.rel_s > 0.1) robot.followLineRight(IRLine, 0.15, -0.04);
    //   // if (IRLine.crosses > 4 
    //   //  && robot.rel_s > 0.8
    //   // ) robot.followLineLeft(IRLine, RobotVelocity, -0.04);
    //   // else robot.setRobotVW(0,0);
      
    // } 
     

    // else if (robot.state == 10) {
    //   //robot.setRobotVW(0,0);
    //   robot.solenoid_state = 0;
    //   robot.followLineLeft(IRLine, RobotVelocity, -0.05);

    // } 
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    else if (robot.state == 100) {
      robot.v_req = 0;
      robot.w_req = 0;

    } else if (robot.state == 101) {
      //robot.v_req = 0;
      //robot.w_req = 0;

    } else if (robot.state == 199) {
      /*robot.v_req = 0.1;
      robot.w_req = 4 * IRLine.IR_values[4] / 1024.0 
                  + 2 * IRLine.IR_values[3] / 1024.0
                  - 2 * IRLine.IR_values[1] / 1024.0
                  - 4 * IRLine.IR_values[0] / 1024.0;*/

    } else if (robot.state == 200) {
      robot.PWM_1 = 0;
      robot.PWM_2 = 0;

    } else if (robot.state == 201) {
      robot.PWM_1 = robot.PWM_1_req;
      robot.PWM_2 = robot.PWM_2_req;
    
    } else if (robot.state == 202) {
      robot.PWM_1 = robot.PWM_1_req;
      robot.PWM_2 = robot.PWM_2_req;      
    }
  
}
