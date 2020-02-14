Multi_goal waypoint provider for racetrack
==================

# Usage

1. Requires (x,y) coordinates to be provided as list to `goalListX` and `goalListY` respectively.

- Initial goal is published upon loading the stack
- Checks for /move_base/feedback (gives current location) from simple action client server messages and if the euclidean distance is less than a threshold  
  then provides the next goal in the list 