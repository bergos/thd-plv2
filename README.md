# thd-plv2

## Build

The HTML files in this repository are built with [Sphinx](https://www.sphinx-doc.org/).
The `build.sh` file contains all required steps from the setup up of the virtual environment to the sphinx command.

## LICENSE

MIT License.

## quirin.md

Contains the report of Quirin Wieser and is the text that is being displayed on the corresponding HTML site.

## thomas.md

Contains the report of Thomas Bergwinkl and is the text that is being displayed on the corresponding HTML site.

## state_diagram.drawio

Visual diagram of the different states in our python code and explaining their functions and relationships.

## docs/html

Contains the HTML sites for our reports (quirin.html, thomas.html).

## ch4.py

Code for programming the robot and solving Challenges four and five. The code and how it works is explained in more detail in the reports. To use the code you need to have Gazebo with a maze simulation open, then simply execute the code on another terminal. The gazebo maze needs to have a red wall that is possible to reach with the robot (or else the robot will drive infinitely) and it needs to be an enclosed maze without any exits (or the robot will eventually drive out of the maze and drive outside along the walls).
