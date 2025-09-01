#include "quadcopter.h"

Quadcopter::Quadcopter(){
    // set default values
    quadcopter_.move_f_b = 0.0;
    quadcopter_.move_l_r = 0.0;
    quadcopter_.move_u_d = 0.0;
    quadcopter_.seq = 0.0;
    quadcopter_.turn_l_r = 0.0;

    // set flag to false 
    inZposition = false;


    state = controlQuad::State::IDLE;

}

void Quadcopter::run(void){
  
  std::unique_lock<std::mutex> lck(mtxStart_);
    
  cvStart_.wait(lck, [&](){return running_ == true;});

    while (running_){
      
      // updates that it is still in motion
      goalReached = false;

      // send visualised goal to display
      seq = 0;

      for (int i = 0; i < goals_.size(); i++) {
          goalReached = false; 
          
          pfms::geometry_msgs::Goal goal{seq++,goals_.at(i)};//if we want to send another goal to be visualised we need to increase seq number
          pfmsConnectorPtr_->send(goals_.at(i));

          // while it has not reached the goal yet
          while (!goalReached){
              // update odom_ with current position of car
              goalReachable = pfmsConnectorPtr_->read(odom_);
              
              //updates the values needed to reach goal and check it is reachable
              goalReachable = checkOriginToDestination(odom_, goals_.at(i), distance_, time_, estimatedGoalPose_);

              // if it cannot reach the goal, returns false and exits function
              if (!goalReachable){
                  state = controlQuad::State::STOPPING;
              }

              // Sets the total distance to the goal
              // used for calculating the distance it has travelled as it moves
              if (!SetInitialDistance){
                  InitialDistance = distance_;
                  SetInitialDistance = true;
              }

              if (inZposition && odom_.position.z - goals_.at(i).z < -0.2){
                inZposition = false;
                state = controlQuad::State::MOVEUP; 
              }

              if (inZposition && odom_.position.z - goals_.at(i).z > 0.2){
                inZposition = false;
                state = controlQuad::State::MOVEDOWN; 
              }
              
              // update the time variable by calling the function
              timeTravelled();

          switch(state)
              {   // initial state of quad when it is stable
                  case controlQuad::State::IDLE   : 
                      if (distance_ > tolerance_ && !inZposition){ 
                          // accelerate when the distance to the goal is set and is greater than the tolerance
                          // start the timer when the quad starts moving
                          startTime = std::chrono::steady_clock::now();
                          state = controlQuad::State::MOVEUP;
                      }  
                      else if (distance_ > tolerance_ && inZposition){
                        state = controlQuad::State::TURN;
                      }  
                      else{
                        status_ = pfms::PlatformStatus::IDLE;
                        if (odom_.position.z < 1){
                          quadcopter_.move_f_b = 0.0;
                          quadcopter_.move_l_r = 0.0;
                          quadcopter_.move_u_d = 0.0;
                          quadcopter_.turn_l_r = 0.0;
                        }
                      }
                      break;
                  // state when the quad is moving up
                  case controlQuad::State::MOVEUP : 
                      if (fabs(odom_.position.z - goals_.at(i).z) < 1e-1){ 
                          // move to Turn state when close to the z goal
                          quadcopter_.move_u_d = 0.0;
                          inZposition = true;
                          state = controlQuad::State::TURN;
                      }
                      // if else for if it has gone way over z goal
                      else if (odom_.position.z - goals_.at(i).z > 0.1){
                        state = controlQuad::State::MOVEDOWN;
                      }
                      else{
                          // do MOVING UP
                          if (odom_.position.z < 0.1){
                            status_ = pfms::PlatformStatus::TAKEOFF;
                            pfmsConnectorPtr_->send(status_);
                          }
                          else{
                          quadcopter_.move_u_d = 0.5;
                          quadcopter_.move_f_b = 0.6;
                          if (angleDifference > 0){
                            quadcopter_.turn_l_r = 0.5;
                          }
                          else{
                            quadcopter_.turn_l_r = -0.5;
                          }

                          }
                      }
                      break;
                  // state when the car needs to be slowed down as it is close to goal
                  case controlQuad::State::TURN : 
                      if (fabs(angleDifference) < 12e-1 && distance_ > 6){ //DO THIS
                        //quadcopter_.turn_l_r = 0.0;
                        quadcopter_.move_f_b = 1.0;
                        state = controlQuad::State::FORWARD;
                      }
                      else if (fabs(angleDifference) < 4e-1 && distance_ < 6){
                        quadcopter_.move_f_b = 0.3;
                        state = controlQuad::State::FORWARD;
                      }
                      else{ 
                        // DO TURNING
                        status_ = pfms::PlatformStatus::RUNNING;
                        quadcopter_.move_f_b = 0.0;
                        if (angleDifference > 0){
                          quadcopter_.turn_l_r = 1.0;
                        }
                        else{
                          quadcopter_.turn_l_r = -1.0;
                        }
                      }
                      break;
                  case controlQuad::State::FORWARD : 
                      if (distance_ < (tolerance_+ 0.5) && quadcopter_.move_f_b == 1.4){ 
                          state = controlQuad::State::STOPPING;
                      }
                      if (fabs(angleDifference) > 1.5e-1 && distance_ > (tolerance_ + 1)){
                        state = controlQuad::State::ADJUST;
                       }
                      else if (distance_ < 1 && quadcopter_.move_f_b < 0.1){
                        quadcopter_.turn_l_r = 0.0;
                        quadcopter_.move_f_b = 0.3;  
                      }
                      else if (distance_ < tolerance_ && quadcopter_.move_f_b < 1.4){
                        state = controlQuad::State::STOPPING;
                      }
                      else{ 
                        // DO MOVING FORWARD
                        quadcopter_.turn_l_r = 0.0;
                        quadcopter_.move_f_b = 1.4;
                        
                      }
                      break;
                  case controlQuad::State::ADJUST : 
                      if (distance_ < (tolerance_+ 0.2) && quadcopter_.move_f_b == 1.0){ 
                          state = controlQuad::State::STOPPING;
                      }
                      if (fabs(angleDifference) < 1e-1 || distance_ < (tolerance_ + 1)){ //DO THIS
                        quadcopter_.turn_l_r = 0.0;
                        state = controlQuad::State::FORWARD;
                      }

                      else{ 
                        if (angleDifference > 0){
                          quadcopter_.turn_l_r = 0.5;
                        }
                        else{
                          quadcopter_.turn_l_r = -0.5;
                        }
                        
                      }
                      break;
                  
                  // state for stopping the car
                  case controlQuad::State::STOPPING :
                      // if the car has stopped within the tolerance, update values and return true
                      if (distance_ < tolerance_ && fabs(odom_.linear.x) < 1e-1 && fabs(odom_.linear.y) < 1e-1){ 
                          // stop the timer when the car is stopped
                          quadcopter_.move_f_b = 0.0;
                          quadcopter_.turn_l_r = 0.0;
                          quadcopter_.move_u_d = 0.0;
                          endTime = std::chrono::steady_clock::now();
                          // indicate the goal was reached within the tolerance
                          goalReached = true;
                          SetInitialDistance = false;
                          // ensure timer is stopped by calling function after endTime is set
                          state = controlQuad::State::IDLE;
                          status_ = pfms::PlatformStatus::IDLE;
                          break;
                      }
                      else if(distance_ > tolerance_ && fabs(odom_.linear.x) < 1e-1 && fabs(odom_.linear.y) < 1e-1){
                      // if vehicle stopped outside of tolerance of the goal, update values and return false
                          // stop the timer when the car is stopped
                          quadcopter_.move_f_b = 0.0;
                          quadcopter_.turn_l_r = 0.0;
                          quadcopter_.move_u_d = 0.0;
                          endTime = std::chrono::steady_clock::now();
                          // indicate the goal was not reached within the tolerance
                          goalReached = false;
                          SetInitialDistance = false;
                          // ensure timer is stopped by calling function after endTime is set
                          state = controlQuad::State::IDLE;
                          status_ = pfms::PlatformStatus::IDLE;
                          break;
                      }
                      else{ 
                        
                        quadcopter_.move_f_b = 0.0;
                        quadcopter_.turn_l_r = 0.0;
                        quadcopter_.move_u_d = 0.0;
                        break;

                      }
                  case controlQuad::State::MOVEDOWN : 
                      if (fabs(odom_.position.z - goals_.at(i).z) < 1e-1){ //DO THIS
                        quadcopter_.turn_l_r = 0.0;
                        quadcopter_.move_u_d = 0.0;
                        state = controlQuad::State::TURN;
                      }
                      else{ 
                        quadcopter_.turn_l_r = 0.0;
                        quadcopter_.move_u_d = -0.2;
                      }
                      break;
                  default:
                      break;
              }


            // This sends the command to the platform
            pfmsConnectorPtr_->send(quadcopter_);
            quadcopter_.seq++;

            pfmsConnectorPtr_->read(updatedOdom_);

            distanceTravelled();

            // This slows down the loop to 20Hz
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
        }
          quadcopter_.move_f_b = 0.0;
          quadcopter_.move_l_r = 0.0;
          quadcopter_.move_u_d = 0.0;
          quadcopter_.turn_l_r = 0.0;
          TotalReachedAllGoals = true;
          running_ = false;
          break;
    }
    ready_ = true;
    
    lck.unlock();
}
  
bool Quadcopter::checkOriginToDestination(geometry_msgs::msg::Pose origin
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose){
    
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
  double originAngle = origin.yaw;
  
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
  estimatedGoalPose.yaw = angle;

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