#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"

#include <vector>
#include <queue>

struct SearchLocation {
  int targetCount = -1;
  Point* coordinates = nullptr;

  SearchLocation(int targets, Point* coords) : targetCount(targets), coordinates(coords) {}

  inline bool operator <(const SearchLocation& other) const {
    return targetCount < other.targetCount;
  }
};

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : virtual Controller {

public:

  SearchController();

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // sets the value of the current location
  //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);
  void SetSuccesfullPickup();
  void AddSearchLocation(Point coordinate, int numOfTargets);

  // BEGIN UPRM
  void SetTotalRovers(int numRovers);
  Point localToGlobal(Point local, Point center);
  Point globalToLocal(Point global, Point center);
  // END UPRM
    

protected:

  void ProcessData();

private:

  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
  int attemptCount = 0;
  //struct for returning data to ROS adapter
  Result result;

  // Our waypoint search priority queue
  priority_queue<SearchLocation> search_queue;

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;

  int totalRovers; // UPRM
};

#endif /* SEARCH_CONTROLLER */
