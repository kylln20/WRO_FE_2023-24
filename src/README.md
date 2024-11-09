# Source Code
This directory contains all the code and other files used in the process of programming our vehicle. 

- `ColourTesterLAB.py` - A modified version of the ColourTester.py, which was originally found in [this OpenCV tutorial](https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html) that we use to find and adjust the colour masks used to detect different obstacles. After switching to using LAB to define our colour masks for avoiding obstacles, the ColourTester.py was also updated.
- `ObstacleChallengeV2.py` - The code that is run for the obstacle challenge. This version was made after the Canada National WRO Competition
- `OpenChallenge.py` - The code that is run for the open challenge. This has been largely unchanged since the Canada National WRO Competition
- `WRO_Randomizer.py` - A randomiser of all possible obstacle challenge arrangements, made to ensure that all possible cases of the obstacle challenge layout are tested and to make more efficient the testing process. This gives a map with a randomised setup of the signal pillars, the starting position of the car, and the location of parking lot.
- `functions.py` - A file containing all self-defined functions that are used in both the ObstacleChallengeV2.py code and the OpenChallenge.py code. Such makes the code easier to update.
- masks.py - A file to record the colour masks (ranges of colour of the obstacles that we wish to detect) that we use in the code.
