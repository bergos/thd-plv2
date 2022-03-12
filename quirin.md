# Report Wieser Quirin

##Problems

###Challenge 2: Rotation with laser distance sensor data
The first problem we faced was in challenge two. The task was to rotate the robot approximately 90 degrees, using only laser distance sensor data. We quickly realised that the robot was rotating at a different angle with each execution, even though we changed nothing in the code. The robot would either turn more than 90 degrees or would turn not enough. It was rare that the robot turned exactly or near 90 degrees and was very dependant on luck.

###Challenge 4: Robot getting stuck in a divider wall if it sees the red wall around a corner
The next problem we faced was in challenge four. The task was for the robot to recognize a red wall, drive to it and stop infront of it. The problem was, that when the robot saw part of the red wall around a divider wall, the robot would start driving towards the red wall and then approach the divider wall and stopping infront of it, essentially getting stuck.

###Challenge 5: Robot moving too close to the walls in a corner and knocking over the wall when rotating
The next problem we faced was in challenge five, although it is rather a small problem. Sometimes when the robot drived into a specific corner, it would drive too close to the walls, and knock over a wall when rotating. However, this problem was only present in one specific corner and we were not able to reproduce this problem in other corners.

###Challenge 5: Robot not rotating sharp and quick enough when having to drive 180 degrees around a divider wall
The last major problem we faced, was also in challenge five. The task is the same as in challenge 4, but this time the maze was more complex and intricate, with more corners and divider walls to drive around. The problem in this challenge was, that there were several divider walls in the maze, where the robot had to drive around the divider wall and do a 180 degree turn, then keep driving forward. With our initial code, the robot was not able to rotate fast and sharp enough, but would instead drive forward instead of rotating enough and then recgonize the wall infront of it and go back the way it came from, essentially being stuck in an endless loop. At other times, the robot would just rotate endlessly after passing a divider wall, instead of driving forward.

##Relevant Concepts

##Approaches and Solutions

###Challenge 2: Rotation with laser distance sensor data
Our approach to solving challenge two was to make use of the sleep() function and have the robot sleep for a certain amount of time, that we specified by using a calculation to determine how long the robot would take to turn approximately 90 degrees, while it rotates. Once the time we specificed was over, the robot would stop rotating and move forward. However, we encountered the problem that the rotation was somewhat random, as mentioned above. After observing a few simulation runs, we started changing the way we calculate the time, adding factors and using trial and error to determine if we can eliminate, or atleats minimes the randomness of the rotation angle. However, despite all this, we could not figure out how to fix the randomness with this approach. So instead of starting over and implementing a different approach, we decided to move on to the next challenge, since it was about using Odometry to rotate the robot, instead of the laser distance sensor, and we figured that this approach would be more accurate and reliable. In hindsight it was the right choice, as we were correct in that the Odometry was a much better method of rotating the robot, and we ended up using Odometry for challenge four and five.

###Challenge 4: Robot getting stuck in a divider wall if it sees the red wall around a corner

###Challenge 5: Robot moving too close to the walls in a corner and knocking over the wall when rotating

###Challenge 5: Robot not rotating sharp and quick enough when having to drive 180 degrees around a divider wall

##Results and Takeaways
