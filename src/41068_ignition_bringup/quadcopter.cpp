#include "quadcopter.h"

Quadcopter::Quadcopter(){
    // set default values

    move_f_b.data = 0.0;
    turn_l_r.data = 0.0;
    move_u_d.data = 0.0;
    move_l_r.data = 0.0;

    command_Pub  = this->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel",3);  
    takeOff_Pub = this->create_publisher<std_msgs::msg::Empty>("drone/takeoff",3); 

    // set flag to false 
    inZposition = false;
    goalReached = true;
    SetInitialDistance = false;

    state = controlQuad::State::IDLE;

    threads_.push_back(std::thread(&Quadcopter::run,this));
}

void Quadcopter::run(void){
  
 std::unique_lock<std::mutex> lck(mtxStart_);

 updateCurrentGoal();

    // while it has not reached the goal yet
            while (!goalReached && isDriving_ && goalSet_){
                std::cout<<"In control"<<std::endl;
                //updates the values needed to reach goal and check it is reachable
                goalReachable = checkOriginToDestination(odom_, goal_.location, goal_.distance, goal_.time, estimatedGoalPose_);
                
                // if it cannot reach the goal, returns false and exits function
                if (!goalReachable){
                    state = controlQuad::State::STOPPING;
                }

                 if (!firstOdomReceived) {
                    previousPos = odom_.position;
                    firstOdomReceived = true;
                }
                else{
                    distanceTravelled();
                    previousPos = odom_.position;
                }
                // update the variable by calling the functions
                timeTravelled();

                if (inZposition && odom_.position.z - goal_.location.z < -0.2){
                inZposition = false;
                state = controlQuad::State::MOVEUP; 
              }

              if (inZposition && odom_.position.z - goal_.location.z > 0.2){
                inZposition = false;
                state = controlQuad::State::MOVEDOWN; 
              }

            switch(state)
              {   // initial state of quad when it is stable
                  case controlQuad::State::IDLE   : 
                      if (goal_.distance > tolerance_ && !inZposition){ 
                          // accelerate when the distance to the goal is set and is greater than the tolerance
                          // start the timer when the quad starts moving
                          startTime = std::chrono::steady_clock::now();
                          state = controlQuad::State::MOVEUP;
                      }  
                      else if (goal_.distance > tolerance_ && inZposition){
                        state = controlQuad::State::TURN;
                      }  
                      else{
                        if (odom_.position.z < 1){
                          command(0.0, 0.0, 0.0, 0.0);
                        }
                      }
                      break;
                  // state when the quad is moving up
                  case controlQuad::State::MOVEUP : 
                      if (fabs(odom_.position.z - goal_.location.z) < 1e-1){ 
                          // move to Turn state when close to the z goal
                          command(move_f_b.data, move_l_r.data, turn_l_r.data, 0.0);
                          inZposition = true;
                          state = controlQuad::State::TURN;
                      }
                      // if else for if it has gone way over z goal
                      else if (odom_.position.z - goal_.location.z > 0.1){
                        state = controlQuad::State::MOVEDOWN;
                      }
                      else{
                          // do MOVING UP
                          if (odom_.position.z < 0.1){
                            std::cout<<"Trying to takeoff"<<std::endl;
                            std_msgs::msg::Empty message = std_msgs::msg::Empty();
                            takeOff_Pub->publish(message);
                          }
                          else{
                          command(0.6, move_l_r.data, turn_l_r.data, 0.5);
                          if (angleDifference > 0){
                            command(move_f_b.data, move_l_r.data, 0.5, move_u_d.data);
                          }
                          else{
                            command(move_f_b.data, move_l_r.data, -0.5, move_u_d.data);
                          }

                          }
                      }
                      break;
                  // state when the car needs to be slowed down as it is close to goal
                  case controlQuad::State::TURN : 
                      if (fabs(angleDifference) < 12e-1 && goal_.distance > 6){ //DO THIS
                        command(1.0, move_l_r.data, turn_l_r.data, move_u_d.data);
                        state = controlQuad::State::FORWARD;
                      }
                      else if (fabs(angleDifference) < 4e-1 && goal_.distance < 6){
                        command(0.3, move_l_r.data, turn_l_r.data, move_u_d.data);
                        state = controlQuad::State::FORWARD;
                      }
                      else{ 
                        // DO TURNING
                        command(0.0, move_l_r.data, turn_l_r.data, move_u_d.data);
                        if (angleDifference > 0){
                          command(move_f_b.data, move_l_r.data, 1.0, move_u_d.data);
                        }
                        else{
                          command(move_f_b.data, move_l_r.data, -1.0, move_u_d.data);
                        }
                      }
                      break;
                  case controlQuad::State::FORWARD : 
                      if (goal_.distance < (tolerance_+ 0.5) && move_f_b.data== 1.4){ 
                          state = controlQuad::State::STOPPING;
                      }
                      if (fabs(angleDifference) > 1.5e-1 && goal_.distance > (tolerance_ + 1)){
                        state = controlQuad::State::ADJUST;
                       }
                      else if (goal_.distance < 1 && move_f_b.data< 0.1){
                        command(0.3, move_l_r.data, 0.0, move_u_d.data);
                      }
                      else if (goal_.distance < tolerance_ && move_f_b.data < 1.4){
                        state = controlQuad::State::STOPPING;
                      }
                      else{ 
                        // DO MOVING FORWARD
                        command(1.4, move_l_r.data, 0.0, move_u_d.data);
                        
                      }
                      break;
                  case controlQuad::State::ADJUST : 
                      if (goal_.distance < (tolerance_+ 0.2) && move_f_b.data == 1.0){ 
                          state = controlQuad::State::STOPPING;
                      }
                      if (fabs(angleDifference) < 1e-1 || goal_.distance < (tolerance_ + 1)){ //DO THIS
                        command(move_f_b.data, move_l_r.data, 0.0, move_u_d.data);
                        state = controlQuad::State::FORWARD;
                      }

                      else{ 
                        if (angleDifference > 0){
                          command(move_f_b.data, move_l_r.data, 0.5, move_u_d.data);
                        }
                        else{
                          command(move_f_b.data, move_l_r.data, -0.5, move_u_d.data);
                        }
                        
                      }
                      break;
                  
                  // state for stopping the car
                  case controlQuad::State::STOPPING :
                      // if the car has stopped within the tolerance, update values and return true
                      if (goal_.distance < tolerance_){ 
                          // stop the timer when the car is stopped
                          command(0.0, 0.0, 0.0, 0.0);
                          endTime = std::chrono::steady_clock::now();
                          // indicate the goal was reached within the tolerance
                          goalReached = true;
                          SetInitialDistance = false;
                          timeTravelled();
                          state = controlQuad::State::IDLE;
                          goals_.pop_front();
                          percentComplete();
                      }
                      else{ 
                        
                        command(0.0, 0.0, 0.0, 0.0);
                      }
                      break;
                  case controlQuad::State::MOVEDOWN : 
                      if (fabs(odom_.position.z - goal_.location.z) < 1e-1){ //DO THIS
                        command(move_f_b.data, move_l_r.data, 0.0, 0.0);
                        state = controlQuad::State::TURN;
                      }
                      else{ 
                        command(move_f_b.data, move_l_r.data, 0.0, -0.2);
                      }
                      break;
                  default:
                      break;
              }


            updateCurrentGoal();
            // distanceTravelled();
            // previousPos = goal_.location;
            // This slows down the loop to 10Hz
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    
            if (goals_.empty()){
                break;
            }
          }
          while (!goalSet_){ // makes the drone stop when the service is called with data 'false'
            command(0.0, 0.0, 0.0, 0.0);
          }

    lck.unlock();
    
}

bool Quadcopter::checkOriginToDestination(geometry_msgs::msg::Pose origin,
                                        geometry_msgs::msg::Point goal,
                                        double& distance,
                                        double& time,
                                        geometry_msgs::msg::Pose& estimatedGoalPose){
    
  // difference between goal and origin
  double dx = goal.x - origin.position.x;
  double dy = goal.y - origin.position.y;
  double dz = goal.z - origin.position.z;

  // calculate distance between the origin and goal
  // distance needed to travel
  double horizontalDist = sqrt(dx * dx + dy * dy);
  double verticalDist = fabs(dz);
  
  distance =  verticalDist + horizontalDist;
  
  // max speed
  double speed = 1;

  // time to move to z position
  double timeToClimb = verticalDist / speed;

  // calculate time to drive to goal once in z spot
  double timeToMove = horizontalDist / speed; 

  // get the initial angle that car is at
  double originAngle = tf2::getYaw(origin.orientation);
  
  // calculates the angle required to face the goal, in radians
  double angle = atan2(dy, dx);
  
  // variable for storing angle difference between origin and goal
  // needed to be able to calculate time
  angleDifference = normaliseAngle(angle - originAngle);
  
  // set max angular acceleration
  double angularAcceleration = 1.0;
  
  // rearranging angular acceleration formula for time
  double timeToTurn = fabs(angleDifference) / angularAcceleration;
  
  // return value for total time needed to reach goal
  time = timeToClimb + timeToTurn + timeToMove;

  // Update the position and orientation of the estimated goal pose
  estimatedGoalPose.position = goal;
  estimatedGoalPose.orientation = yawToQuaternion(angle);


  // return true if the goal is reacheable
  // returns false otherwise
  if (distance < 0 && time < 0){
    distance = -1;
    time = -1;
    return false;
  }
  else{
    
    return true;
  }

}

double Quadcopter::normaliseAngle(double theta) {
    if (theta > (2 * M_PI))
      theta = theta - (2 * M_PI);
    else if (theta < 0)
      theta = theta + (2 * M_PI);

    if (theta > M_PI){
        theta = -( (2* M_PI) - theta);
    }

    return theta;
  }

  void Quadcopter::command(double move_forward_back, double move_left_right, double turn_left_right, double move_up_down) {

    move_f_b.data = move_forward_back;
    turn_l_r.data = turn_left_right;
    move_u_d.data = move_up_down;
    move_l_r.data = move_left_right;

    geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();
    
    msg.linear.x= move_f_b.data;
    msg.linear.y= move_l_r.data;
    msg.linear.z= move_u_d.data;
    msg.angular.z = turn_l_r.data;
    command_Pub->publish(msg);
}