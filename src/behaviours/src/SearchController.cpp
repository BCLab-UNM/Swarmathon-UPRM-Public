#include <ros/ros.h>

#include "SearchController.h"
#include <angles/angles.h>

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;

  search_queue = priority_queue<SearchLocation>();
  
}

void SearchController::Reset() {
  result.reset = false;

  search_queue = priority_queue<SearchLocation>();
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {


  if (!result.wpts.waypoints.empty()) {
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.15) {
      attemptCount = 0;
    }
  }


  if (attemptCount > 0 && attemptCount < 5) {
    attemptCount++;
    if (succesfullPickup) {
      succesfullPickup = false;
      attemptCount = 1;
    }
    return result;
  }
  else if (attemptCount >= 5 || attemptCount == 0) 
  {
    attemptCount = 1;


    result.type = waypoint;
    Point  searchLocation;

    //select new position 50 cm from current location
    if (first_waypoint)
    {
      first_waypoint = false;
      searchLocation.theta = currentLocation.theta + M_PI;
      searchLocation.x = currentLocation.x + (1.5 * cos(searchLocation.theta));
      searchLocation.y = currentLocation.y + (1.5 * sin(searchLocation.theta));

      ROS_INFO("UPRM: First Waypoint");
      ROS_INFO_STREAM("UPRM: Current Location[" << currentLocation.x << ", " << currentLocation.y << ", " << currentLocation.theta << "]");
      ROS_INFO_STREAM("UPRM: Search  Location[" << searchLocation.x << ", " << searchLocation.y << ", " << searchLocation.theta << "]");

    }
    else
    {
      //select new heading from Gaussian distribution around current heading
      searchLocation.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
      //searchLocation.x = currentLocation.x + (5 * cos(searchLocation.theta));
      // searchLocation.y = currentLocation.y + (5 * sin(searchLocation.theta));

      ROS_INFO("UPRM: Next Waypoint");
      ROS_INFO_STREAM("UPRM: Current Location[" << currentLocation.x << ", " << currentLocation.y << ", " << currentLocation.theta << "]");
      ROS_INFO_STREAM("UPRM: Search  Location[" << searchLocation.x << ", " << searchLocation.y << ", " << searchLocation.theta << "]");
    }

    result.wpts.waypoints.clear();
    searchLocation.x = 0;
    searchLocation.y = 0;
    
    /*if (!search_queue.empty()) {
      SearchLocation topSearch = search_queue.top();
      // search_queue.pop();
      // searchLocation.x = topSearch.coordinates->x;
      //searchLocation.y = topSearch.coordinates->y;
      }*/
    
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
    //result.wpts.waypoints.clear();
    return result;
  }

}

void SearchController::AddSearchLocation(Point coordinate, int numOfTargets) {
  SearchLocation searchLocation = SearchLocation{numOfTargets,(Point*)(&coordinate)};
  search_queue.push(searchLocation);
}

void SearchController::SetCenterLocation(Point centerLocation) {
  
  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;
  
  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
  }
  
}

void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool SearchController::HasWork() {
  return true;
}

void SearchController::SetSuccesfullPickup() {
  succesfullPickup = true;
}

// BEGIN UPRM
void SearchController::SetTotalRovers(int numRovers) {
  totalRovers = numRovers;
  //ROS_INFO_STREAM("UPRM Total rovers in SearchController: " << totalRovers);
}
// END UPRM
