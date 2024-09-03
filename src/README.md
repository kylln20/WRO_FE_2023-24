# (Obstacle Challenge) Code Logic
The open challenge code is similar to the obstacle challenge code but simplified. To avoid repetition, here is the breakdown of the obstacle challenge code.

### Import Libraries
#### Libraries in use:
* OpenCV (for computer vision)
* picamera2 (for camera control)
* HiwonderSDK.Board (for HAT communication)
* RPi.GPIO (for GPIO control on the Raspberry Pi)
* Time (for time-related functions, mainly: sleep())
* numpy (for numerical operation and array processing)
* Math (for calculations to find the distance from objects)

### Function Declarations
Functions were written to make our code easier to write, as well as easier to understand.
* write(value): takes in an angle or speed to send to the servo or DC motor respectively
* multi_write(sequence): takes an array of values to send to the servo or DC motor
* stop_car(): Sents the DC motor a command to stop, sends the servo motor a command to point the wheels straight and turns off LEDs
* contours(hsvRange, ROI): returns list of contours when given a colour mask and region of interest
* max_contour(contours): returns the area of the largest contour in an array of contours
* display _roi(): displays all regions of interest (ROI) for debugging
* buzz(): activates buzzer for debugging
* LED1(r, g, b), LED2(r, g, b): takes in RGB values to display on 2 LEDs, so that we know what decisions the code is generally making
* display_variables(): displays multiple variables and their values in the terminal for debugging purposes

### Program Setup
This includes different lines of code that only need to be executed once, at the beginning of the match. 
* Initialising the camera
 Declaring variables and assigning them initial values
* Checking for any system arguments (for debugging in general, or specific functions, like the three-point turn, or parking debugging)
* Waiting for the button press to start the round

### Main Loop
Everything else in our program exists in a loop. Every time a photo is captured from the camera, the loop reformats the photo, then runs through a series of decisions (largely in IF/ELSE statements) to calculate a number to write() to the necessary motor(s). 

#### Reformatting:
OpenCV by default uses BGR, as opposed to RGB. Additionally, using either format makes it difficult to create a colour range that accounts for the values colours have when in different lighting. So we convert the image to HSV.
>         img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#### Contour Searching
Using contour(), the walls, red and green signal pillar contours, and coloured lines on the mat are searched for. The contours of those colours are stored in arrays. For 

#### Signal Pillar Processing
The arrays of the green and red contours are searched to see if a signal pillar is present. This is done by checking the contour’s area.

After that, based on the coordinates of the signal pillar, the pillar may or may not be selected.
However, certain conditions will de-select the pillar:
* If a pillar is too far
* If the car is about to crash into a wall
* If the pillar is near the bottom of the screen (indicating that it has already been passed

If there are multiple pillars visible, the control variables (Kp, Kd) used to calculate the turning angle are changed so that the car’s turns are less extreme to one side or the other.
  
#### Coloured Line Processing
The orange and blue contours are searched for to see if the car is close enough to them to indicate being at a corner. If the direction the car is turning (right or left) corresponds to the colour of the nearest line (orange or blue respectively), then the program knows the car should be turning.

If the colour of the nearest line and the turning direction do not match, then the program checks if a three-point turn needs to be performed.

If a red pillar is visible, the three-point turn is performed after the pillar is passed

If there is no red pillar visible but the previous pillar was red, then the car performs a three-point turn

#### Parking Lot
After the car has turned twelve times, it will have completed three laps. So the program looks for magenta contours in the left, right, and centre regions of interest. The contour of the largest area in each region of interest is what is used to make decisions. 

After thirteen turns, if the car detects a magenta contour in the left or right region of interest, the program waits for the y-coordinate of that contour to pass a certain threshold before beginning to turn.

To park, the car turns towards the parking walls. If there is a magenta contour detected in the central region of interest, this is a sign that the car could crash into the walls. This is undesirable, so the car instead backs up, allowing more distance to re-adjust. Another mechanism of adjusting is based on the area of the black wall contours. If while turning left, the area of the wall contour in the right region of interest passes a certain value, then that indicates that the car is too close to the left parking wall, so the car turns right slightly. The opposite is also true.

Eventually, when the area of the black wall contour detected in the centre region of interest is big enough, then the car is inside the parking lot, and the car is stopped.

#### Three-Point Turn
After eight turns are performed, the program checks if a three-point turn needs to be made. 
If there is a signal pillar after the twelfth turn, and it is red, then it will drive until the red pillar is not visible for ten iterations of the main loop, then perform a three-point turn.
Otherwise, it will perform a three-point turn if the previous obstacle seen was red. 

#### Debugging
The ROI and variables are displayed if the code is being run in debugging mode, and the program checks if ‘q’ is pressed. If ‘q’ is pressed, then the program ends.
