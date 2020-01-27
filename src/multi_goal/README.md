Multi_goal waypoint provider for racetrack
==================

# Usage

1. Requires (x,y) coordinates to be provided as list to `goalListX` and `goalListY` respectively.

- Initial goal is published upon loading the stack
- Checks for /move_base/resutls from simple action client server messages and if the status is "reached" then provides the next goal in the list 