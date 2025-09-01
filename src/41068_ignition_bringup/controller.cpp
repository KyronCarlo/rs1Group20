#include "controller.h"

Controller::Controller() : Node("controller") {
   
    odometrySub_ = this->create_subscription<nav_msgs::msg::Odometry>("orange/odom", 1000, std::bind(&Controller::odoCallback, this, std::placeholders::_1));
    goalsSub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("orange/goals", 1000, std::bind(&Controller::setGoal, this, std::placeholders::_1));

    missionService_ = this->create_service<std_srvs::srv::SetBool>("/orange/mission",std::bind(&Controller::runMission, this, std::placeholders::_1, std::placeholders::_2));

    goalPub_ = this->create_publisher<geometry_msgs::msg::Point>("/goal_point", 1);
    
    waypointsPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("orange/waypoints", 1);

    boolSub_ = this->create_subscription<std_msgs::msg::Bool>("/goalInCentre", 1, std::bind(&Controller::boolCallback, this, std::placeholders::_1));

    // set flag to false 
    SetInitialDistance = false;
    isDriving_ = false;
    goalSet_ = false;
    isGoalInCentre = false;
    firstOdomReceived = false;

    // update variables that need to be zero
    distanceTravelled_ = 0.0;
    timeInMotion_ = 0.0;
    percentCompleted = 0.0;

    goal_.distance = 0;
    goal_.time = 0;
    tolerance_ = 0.5;

};

Controller::~Controller(){

    //Join threads
    for(auto & t: threads_){
        t.join();
    }
}

void Controller::setGoal(const geometry_msgs::msg::PoseArray& msg){

    std::lock_guard<std::mutex> lock(goalMtx_);

    goals_.clear();
    for (const auto& pose : msg.poses) {
        goals_.push_back(pose.position);
    }
    goal_.location = goals_.front(); 
    goalPub_->publish(goal_.location);
    goalSet_ = true;
    getTotalDistance();
    percentComplete();
    std::cout<<"Goals Set"<<std::endl;
}

double Controller::distanceToGoal(void){
    // returns the distance to the goal
    return goal_.distance;
}

double Controller::timeToGoal(void){
    // returns the time to the goal
    return goal_.time;
}

bool Controller::setTolerance(double tolerance){
    // update the internal variable for the tolerance value
    // based on inputted value
    // return false if tolerance inputted is negative
    if (tolerance < 0){
        return false;
    }
    else{
        tolerance_ = tolerance;

        return true;
    }
}

double Controller::distanceTravelled(void){
    // calculates the distance that has been travelled

    double xDist = odom_.position.x - previousPos.x;
    double yDist = odom_.position.y - previousPos.y;
    double dist = std::sqrt(xDist * xDist + yDist * yDist);

    // Accumulate distance travelled
    distanceTravelled_ += dist;
    //returns the total distance that has been travelled. 
    return distanceTravelled_;
}

double Controller::timeTravelled(void){
    // declare timeTaken variable used for calculations
    std::chrono::duration<double> timeTaken;

    // when the goal has been reached and total time has not been set
    // store the total time to reach the first goal and accumulate it
    if (goalReached && !TotalTimeSet){
        totalTimeTaken += (endTime - startTime);
        timeTaken = totalTimeTaken;
        TotalTimeSet = true;
    }
    else if (!goalReached) { // if the goal has not been reached
        // update the current time in motion
        timeTaken = std::chrono::steady_clock::now() - startTime;
        // if it has completed previous goals
        // add this to the time from the other goals
        // returns the total time the platform has been moving
        timeTaken += totalTimeTaken; //+ timeTaken;
        TotalTimeSet = false;
    }
    //if the function is called after the goal has been reached
    else if (goalReached && TotalTimeSet){ 
        timeTaken = totalTimeTaken;
    }
    
    // convert the time into a double and return it
    timeInMotion_ = timeTaken.count();
    
    return timeInMotion_;
}

 geometry_msgs::msg::Pose Controller::getOdometry(void){
    // returns the current odometry information of the car 
    std::lock_guard<std::mutex> lock(poseMtx_);
    return odom_;
}


void Controller::odoCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    std::lock_guard<std::mutex> lock(poseMtx_);
    odom_ = msg->pose.pose;
    speed_ = msg->twist.twist;
}

void Controller::runMission(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,std::shared_ptr<std_srvs::srv::SetBool::Response> res){
    std::unique_lock<std::mutex> lck(mtxStart_mission);
   
    if (req->data){
        threads_.emplace_back(&Controller::run, this);
        goalSet_ = true;
    }
    else{
        goalSet_ = false;
    }
    res->success = isGoalInCentre;
    percentComplete();
    res->message = std::to_string(percentCompleted) + "% Complete";
}

void Controller::updateCurrentGoal(){
    if (goals_.size() > 0 && goalReached){
        goal_.location = goals_.front(); 
        isDriving_ = true;
        goalReached = false;
    }
    else if(goals_.size() > 0 && !goalReached){ 
        isDriving_ = true;
    } 
    else {
        isDriving_ = false;
    }
}

geometry_msgs::msg::Point Controller::getCurrentGoal(){
    return goal_.location;
}

void Controller::boolCallback(const std_msgs::msg::Bool::SharedPtr msg){
    isGoalInCentre = msg->data;
}

void Controller::getTotalDistance(){

    totalDistanceToAllGoalsValue = 0.0;
    
    geometry_msgs::msg::Pose goalOdom;
    
    for (size_t i = 0; i < goals_.size(); i++){
        
        // check if the goal is reacheable
        if (i == 0){
            goalReachable = checkOriginToDestination(odom_, goals_.at(i), goal_.distance, goal_.time,
                                                            estimatedGoalPose_);
        }

        else{
            goalReachable = checkOriginToDestination(goalOdom, goals_.at(i), goal_.distance, goal_.time,
                                                           estimatedGoalPose_);
        }
            
        goalOdom = estimatedGoalPose_; // updates the odom with estimated for each goal

        totalDistanceToAllGoalsValue += goal_.distance;
        
    }
}

void Controller::percentComplete(){
    if (percentCompleted >= 99 && !goals_.empty()){
        percentCompleted = 99;
    }
    if (goals_.empty()){
        percentCompleted = 100;
    }
    percentCompleted = (distanceTravelled_ / totalDistanceToAllGoalsValue) * 100;
}

geometry_msgs::msg::Quaternion Controller::yawToQuaternion(double yaw){
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw); 

    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();
    return q_msg;
}