
### Introduction

This software is built to act like a simulator for the SLAM problem (Simultaneous Localization and Mapping). It has been built to be as realistic as possible, including a realistic sensor and motion model. The software is built to handle any 2D SLAM algorithm, and currently has Gmapping implemented. This program runs natively on Windows, and simulates a SLAMTEC RP-LiDAR A1 sensor.

### Layout

![An example of the full layout of the software; this iteration, the robot experienced medium-heavy carpet (17.5% translational noise, 7.5% rotational noise) and a hard environment.](/Resources/ReadmePics/image1.jpg)

The application GUI is split into 4 quadrants. In the upper right, we have the world map. In the bottom left, we have the sensor, and everything it sees. In the top left, the robot shows the map with highest confidence. You might notice the top left looks like a combination of the other two. The bottom right is special. At any given moment, the robot may have different ideas with varying confidence about exactly where it is. This quadrant shows the spread of these positions. This GUI is built around Gmapping; in this case, these locations are the estimated poses of each particle.

### Usage

This program is built using Windows libraries, and can be compiled using:

```mingw32-make```

After compiling, the program can be rerun like so:

```.\main.exe```

Note this will not work without the required Windows libraries, including Direct2D 1.1.

Settings can be changed in the config.h file. At some point, this will be converted into a parameter specification sheet. The config.h file is organized into sections of changability. Settings higher up in the file are meant to be messed with. Settings lower down in the file should rarely if ever be changed.

### Results

![An example of the map created by the robot.](/Resources/ReadmePics/image2.jpg)

This simulation shows a robot succesfully mapping a room in several environments, from hardwood floor to heavy shag carpet. In the above example, the robot mapped a simple 12 x 6.75 meter room with medium-heavy carpet. Through all environments, the robot maps successfully. Its only weaknesses are failures inherent to the Gmapping algorithm (large environments, long feature-less hallways, repetition, etc).

### Bugs

Currently, there are two known bugs in the software.

First, the map starts slowly rotated if the robot starts with a non-zero bearing. This is easily remedied by forcing the robot to stop for exactly one period when the simulation starts; and hasn't proven to be an issue.

Second, I discovered there may be a logic error with how particle poses are being updated. Upon further review, and trying to "patch" the problem, I made the problem worse. It's likely I don't fully understand exactly where the logic error starts. This will be reviewed and fixed in the future. For now, the software works well and the robot can map most of the environments given to it with ease.

### Future Work

This simulator is built to handle multiple kinds of 2D algorithms. In the future, I'd like to add more algorithms to the simulator. I'd also like to try these algorithms on a real robot, which I'm currently working on making.

Additionally, I find fine-tuning the Gmapping parameters an intriguing and difficult challenge. I quite enjoy it, and may write some guide to how the different paramters are related.

###### Written by Devin Frandsen; April 2026