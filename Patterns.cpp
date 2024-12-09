#include "Patterns.h"
#include <math.h>
#include <random>

using std::min;
using std::max;

#define STEPS_PER_MOTOR_REV   2048                                    //Number of motor steps in one revolution of the output shaft of the motor. 
#define STEPS_PER_A_AXIS_REV  2 * STEPS_PER_MOTOR_REV                 //the number of steps required to move the angular axis one full revolution
#define TRAVEL_PER_PINION_REV 50.267                                  //Distance in mm the rack moves in one complete revolution of the pinion.
#define STEPS_PER_MM          81.4849                                 //Precomputed motor steps per mm of radial axis travel. 
#define MM_PER_STEP           1.0 / STEPS_PER_MM                      //Millimeters of travel per motor step on radial axis. Evaluates to 0.01227 if STEPS_PER_REV is 2048. 
#define STEPS_PER_DEG         (STEPS_PER_A_AXIS_REV) / 360            //Motor steps per degree of motion on angular axis. Should be about 11.378 steps per degree.
#define DEG_PER_STEP          1 / STEPS_PER_DEG                       //Degrees of rotation on angular axis per motor step. About .08799 degrees.
#define STEPS_PER_RAD         STEPS_PER_MOTOR_REV / PI                //Motor steps per radian of motion on angular axis. About 652. 
#define RAD_PER_STEP          1 / STEPS_PER_RAD                       //Radians travelled on angular axis per motor step. About 0.00153

#define ACTUAL_LEN_R_MM       87.967                                  //Length in mm of the radial axis (hard limits). Derived from the CAD model of the hardware.
#define ACTUAL_LEN_R_STEPS    ACTUAL_LEN_R_MM * STEPS_PER_MM          //Maximum possible length of radius in steps of motor (hard limits). Should be 7167 when 2048 steps per rev in motor.
#define MAX_R_STEPS           7000                                    //Soft limit on how far the radius can move in terms of steps of the motor. This leaves a slight buffer on each end.
#define MAX_R_MM              MAX_R_STEPS * MM_PER_STEP               //Soft limit length in mm of the radial axis. 85.91mm. 

#define HOMING_BUFFER         (ACTUAL_LEN_R_STEPS - MAX_R_STEPS) / 2  //Crash home R axis to 0, then move this many steps in positive direction to create a soft stop.
#define RELAXATION_BUFFER     STEPS_PER_DEG * 5                       //Crash homing tensions the bead chain, and backlash and flex in the gantry need to be released.

#define MAX_SPEED_R_MOTOR     550.0                                   //Maximum speed in steps per second for radius motor. Faster than this is unreliable.
#define MAX_SPEED_A_MOTOR     550.0                                   //Maximum speed in steps per second for angle motor.

#define PI                    3.1415926535


std::random_device rd;  // Non-deterministic seed source
std::mt19937 gen(rd()); // Mersenne Twister engine

int random(int min, int max)
{
  std::uniform_int_distribution<int> dist(min, max);

    // Generate a random integer
  return dist(gen);
}

int constrain(int value, int min, int max)
{
  if (value < min)
    value = min;
  else if (value > max)
    value = max;
  return value;
}

#pragma region Motion

/**
 * @brief Calculates the effective radial change, accounting for the motion of the angular axis.
 *
 * The radial axis movement is influenced by the angular axis movement, so this function computes the
 * actual change in the radial axis by considering the steps taken by both the angular and radial motors.
 *
 * @param angularMoveInSteps The number of steps the angular motor has moved.
 * @param radialMoveInSteps The number of steps the radial motor has moved.
 *
 * @return int The effective radial change in steps, with the angular axis movement accounted for.
 *         A positive value indicates a decrease in radius, while a negative value indicates an increase in radius.
 */
int calcRadialChange(int angularMoveInSteps, int radialMoveInSteps)
{
  int actualChangeR = angularMoveInSteps - radialMoveInSteps;

  //should return the number of steps R axis has moved, with A axis motion accounted for.
  //if actualChangeR is positive, radius is decreasing. 
  //if actualChangeR is negative, radius is increasing.
  return actualChangeR;
}


/**
 * @brief Calculates the shortest path to the target position on the angular axis.
 *
 * This function determines the shortest distance required to move from the current position to the
 * target position on a circular axis, considering both clockwise and counterclockwise directions.
 * It returns the shortest distance, taking into account a wraparound value for circular motion.
 *
 * @param current The current position on the angular axis, in steps.
 * @param target The desired target position on the angular axis, in steps.
 * @param wrapValue The wraparound point for the axis (e.g., the total number of steps per revolution).
 *
 * @return int The shortest distance, in steps, required to move to the target position.
 *         Positive values indicate clockwise movement, while negative values indicate counterclockwise movement.
 */
int findShortestPathToPosition(int current, int target, int wrapValue)
{
  int dist1 = modulus((target - current), wrapValue);
  int dist2 = -1 * modulus((current - target), wrapValue);
  if (abs(dist1) <= abs(dist2))
  {
    return dist1;
  }
  else
  {
    return dist2;
  }
}



/**
 * @brief Calculates the number of steps required for the radial axis motor to move, accounting for the angular axis motion.
 *
 * This function computes the necessary steps for the radial axis motor to move from the current position
 * to the target position. It compensates for the fact that the angular axis motion influences the radial
 * axis but not vice versa. The function adjusts the radial movement based on the planned angular axis movement.
 *
 * @param current The current position of the radial axis in steps.
 * @param target The desired target position of the radial axis in steps.
 * @param angularOffsetSteps The number of steps the angular axis motor will move in the next planned move.
 *
 * @return int The total number of steps the radial axis motor needs to move, adjusted for the angular axis offset.
 */
int calcRadialSteps(int current, int target, int angularOffsetSteps)
{
  return ((current - target) + angularOffsetSteps);
}



#pragma endregion Motion

#pragma region GeometryGeneration

/**
 * @brief Precomputes and returns points approximating a straight line between two positions in polar coordinates.
 *
 * This function precomputes an array of points that approximate a straight line by interpolating between two
 * end points specified in cartesian coordinates, then converts them to polar coordinates. It stores these points
 * in a static array and returns the next point on each function call, allowing efficient streaming of precomputed
 * line points. The line is divided into a specified number of segments (resolution), with a maximum of 100 points.
 *
 * @param point0 The starting point of the line, specified in radial and angular steps.
 * @param point1 The ending point of the line, specified in radial and angular steps.
 * @param current The current position of the gantry, used to calculate relative motion if needed.
 * @param resolution The number of segments to divide the line into, defaulting to 100 and capped at 100.
 * @param reset Bool - set true to force recalculation for a new line.
 *
 * @return Positions The next point along the precomputed line, with radial and angular values in steps.
 *
 * @note The function handles vertical lines by temporarily rotating the points 90 degrees to avoid calculation
 * issues, then rotates them back before returning. The line is broken into segments up to the maximum length of
 * the array, and lines close to the center of the field are handled with a higher resolution to maintain accuracy.
 *
 * @details The first call to this function precomputes all points along the line, and subsequent calls return
 * each point in sequence. The function resets for a new line after the last point is returned.
 */
Positions drawLine(Positions point0, Positions point1, Positions current, int resolution, bool reset)
{
//this is the nested array that will store the precomputed points. has to be static so values persist between function calls.
//it will be of the form pointArray[100][2] = {{r0, theta0}, {r1, theta1}, ... {r99, theta99}}.
//to access the theta value for point3 (4th point in array), you would call point3.angular = pointArray[3][1];

//Future update: make this a single layer array of type Positions instead of type Int for simplicity.
  static int pointArray[100][2];

  static int numPoints = 0;                           //the number of points the line will be approximated with.
  static bool newLine = true;                         //used to track if the function is being called for a new line, or if it needs to provide points for an extant line
  static float x0 = 0, x1 = 0, y0 = 0, y1 = 0;        //end points of the line
  static float xtemp = 0, ytemp = 0, thetaTemp = 0.0; //temporary storage for calculations
  static float stepover = 0;                          //how far to move along x-axis for interpolating along line
  static float m = 0;                                 //the slope of the line (y = mx + b)
  static float denom = 0;                             //the denominator in the slope calculation (x1 - x0)
  static float b = 0;                                 //the y-intercept of the line (y = mx + b)
  static bool pointsRotated = false;                  //used to indicate if points have been rotated to deal with vertical lines and need to be rotated back on output.

  Positions p0 = point0, p1 = point1;                 //containers for the points (we may need to modify their values to deal with vertical lines)
  Positions outputPoint;                              //the struct we'll use for passing the target positions out of the function
  static int outNum = 0;                              //used for tracking which point to return on each call to this function

  if (newLine || reset)
  {                             //if this is a new line, or the reset flag is set
    numPoints = constrain(resolution, 0, 100);     //we can approximate the line with up to 100 points. recalculate this number for each new line.

    //check now to see if there will be a vertical line after the coordinate transformation from polar to rectangular coords
    int comparisonA = STEPS_PER_A_AXIS_REV - max(p0.angular, p1.angular);        //units are in steps
    int comparisonB = min(p0.angular, p1.angular);

    //this next step checks to see if the line connecting these two points is within half a degree of vertical in the rectangular coordinate system.
    //From my early testing, if the lines are more than half a degree off of vertical, they render perfectly fine without special handling.
    //It's really just a vertical line that gets weird (e.g., a line connecting two points that are 45 and 315 degrees off the origin ray at the same radius).
    if ((comparisonA - comparisonB <= convertDegreesToSteps(0.5)) && (comparisonA - comparisonB >= convertDegreesToSteps(-0.5)))
    {
      pointsRotated = true;   //we're going to rotate the points by 90 degrees to deal with the nearly vertical line, so set this flag.
      p0.angular += convertDegreesToSteps(90);
      p1.angular += convertDegreesToSteps(90);
    }

    //take in the points, convert them to radians for the angular unit. only need to do this one time for a new line.
    //also convert each point from polar to cartesian coordinates.
    x0 = p0.radial * cos(convertStepsToRadians(p0.angular));        //x = r*cos(theta)
    y0 = p0.radial * sin(convertStepsToRadians(p0.angular));        //y = r*sin(theta)
    x1 = p1.radial * cos(convertStepsToRadians(p1.angular));        //x = r*cos(theta)
    y1 = p1.radial * sin(convertStepsToRadians(p1.angular));        //y = r*sin(theta)

    denom = x1 - x0;

    //calculate the slope
    m = (y1 - y0) / denom;
    //calculate the y-intercept   y = mx + b, so b = y - mx. Use point0 values for y and x
    b = y0 - (m * x0);


    if (b < 100.0 && b > -100.0)
    {      //if the line is within 100 steps of the origin
//This takes care of lines that come really close to intercepting the origin. First, I'm using this range of values rather 
//than saying if (b == 0.0) because this is using floating point math, and equalities like that almost never evaluate to
//true with floats. Lines that come really close to the origin require the gantry to flip around 180 degrees in the
//angular axis once the ball is at the center of the field. The straight line algorithm already handles this well, but if
//the line is broken into a small number of segments, that large rotation at the center winds up drawing a small arc 
//around the center. I dealt with this by just having the program maximize the number of segments the lines is broken
//into for lines which come close to the center. You can adjust the values in the condition above to change what it means
//for a line to be close to the center to fine tune how well straight lines are drawn.
      numPoints = 100;
    }
    //This line doesn't come really close to intersecting the origin, so we'll handle it differently.

    //divide one axis into the number of segments required by resolution, up to a maximum of the length of the array they'll be stored in.
    //defining all of these values as static means the value will persist between function calls, but also means I have to reset them
    //to initial values once the last point in the line is returned.
    stepover = (x1 - x0) / (float)numPoints;       //should define how far to move along x axis for interpolation along line.

    for (int i = 0; i < numPoints; i++)
    {
//start by generating absolute position values for the points along the line in terms of {r, theta}.
//We are starting with absolute values because the end points of the line are specified in absolute coordinates.

      if (i == 0)
      {                                             //if it's the first point in the line, put the point0 values into the list to ensure we start there
        pointArray[i][0] = p0.radial;                       //these units are already in steps as absolute coordinates
        pointArray[i][1] = p0.angular;                      //units in steps, absolute coordinates. need to be changed to relative later.
      }
      else if (i == numPoints - 1)
      {                          //If it's the last point in the line, put point1 values into the list to make sure we end there.
        pointArray[i][0] = p1.radial;
        pointArray[i][1] = p1.angular;
      }
      else
      {                                                  //We're somewhere along the line that isn't the beginning or end, so we need to generate these values.
       //Calculate the next x value in the series. Use the values of i and stepover to figure out how many line segments to increment along from the starting point.
       //I'm using (i + 1) instead of i in the calculation because I'm handling the first and last points separately,
       //so by the time we get to this code, we need to move over by at least one increment of stepover, but i starts counting from 0.
        xtemp = x0 + (i + 1) * stepover;
        ytemp = m * xtemp + b;                                  //y = mx + b gives next y value in the series.

        //calculate the angular position of the current point.
        //atan2f(y, x) is a special version of the arctan function that returns the angle based on y and x.
        thetaTemp = atan2f(ytemp, xtemp);

        //ata2f() has a range of (-pi, pi), and we'll need to shift that to be (0, 2pi)
        if (thetaTemp < 0) thetaTemp = 2.0 * PI + thetaTemp;    //this is in radians, ranging from 0 to 2pi

        //now that we know the anglular position of the point in radians, we need to find the radial position in units of steps
        //using the Pythagorean theorem (square roots calculate more efficiently than trig functions on Arduino Nano).
        //Then store the r and theta points in the array.
        pointArray[i][0] = sqrt(xtemp * xtemp + ytemp * ytemp); //the radial value of the point. This is absolute coordinates from the origin. Units are steps.
        //store the angular position converted from radians to steps. This is still in absolute coordinates, not relative.
        pointArray[i][1] = convertRadiansToSteps(thetaTemp);
      }

      //finally, if we rotated the points to deal with a vertical line, rotate them back.
      if (pointsRotated)
      {
        pointArray[i][1] -= convertDegreesToSteps(90);
      }
    }

    //we need to set the newLine flag to false so that the next time this function is called,
    //we can output the points along the line rather than recalculating the points.
    newLine = false;       //later in the program, we have to reset this to true once the last line of the point is returned.
    reset = false;
    outNum = 0;            //need to reset this to 0 so we can start outputting the points, starting from the first one.
    pointsRotated = false;
  }

  //now we need to output the correct point in the array.
  if (outNum < numPoints)
  {
    outputPoint.radial = pointArray[outNum][0];   //put the r value into the struct
    outputPoint.angular = pointArray[outNum][1];  //put the theta value into the struct
    outNum++;                                     //increment to the next point in the array
  }

  //once the last point is ready for return, reset all the variables necessary to rerun all the calculations on the next call to this function.
  if (outNum >= numPoints)
  {
    newLine = true;
  }

  //finally, return the value of the point to be moved to!
  return outputPoint;
}

/**
 * @brief Generates the vertices of a regular n-sided polygon (n-gon) and stores them in an array of Positions.
 *
 * This function computes the vertices of a regular polygon (n-gon) with a specified number of sides, radius,
 * center point, and optional rotation. The vertices are generated in polar coordinates, with the first vertex
 * starting at angle 0 (or rotated by the specified degrees) and are then translated to be centered around the
 * specified center point. The generated points are stored in the provided pointArray.
 *
 * @param pointArray A pointer to the array of Positions to be filled with the vertices of the polygon.
 * @param numPoints The number of vertices (or sides) of the polygon.
 * @param centerPoint The center point of the polygon, specified as a Positions struct (radial and angular coordinates).
 * @param radius The radius of the polygon, which is the distance from the center to each vertex (in motor steps).
 * @param rotationDeg An optional rotation of the polygon in degrees, defaulting to 0.0. This rotates the polygon around its center.
 *
 * @return void
 *
 * @note The function first generates the vertices centered on the origin in polar coordinates, then translates
 * them to the specified center point by converting to rectangular coordinates, performing the translation, and
 * converting back to polar. The translatePoints() function is used to handle this translation process.
 *
 * @example
 * // Example of generating an octagon with a radius of 4000 steps centered on the origin:
 * int numberVertices = 8;
 * Positions vertices[numberVertices];
 * Position center = {0, 0};
 * nGonGenerator(vertices, numberVertices, center, 4000, 0.0);
 *
 * // Example of generating a circle with 360 points and a radius of 2000 steps, centered at {3000, 60 degrees}:
 * int numberVertices = 360;
 * Positions vertices[numberVertices];
 * Position center = {3000, convertDegreesToSteps(60)};
 * nGonGenerator(vertices, numberVertices, center, 2000, 0.0);
 */
void nGonGenerator(Positions *pointArray, int numPoints, Positions centerPoint, int radius, float rotationDeg)
{
//*pointArry is the pointer to the array that will be built out.
//numPoints is the length of that array (equal to number of desired vertices).
//centerPoint is the center point of the polygon (supply as a Position struct)
//radius is the distance from the center point to a vertex. Units are motor steps.
//rotationDeg rotates the polygon in degrees. The first vertex will always be at angle = 0, unless you specify a rotation angle.

//Start by generating vertices in polar coords, centered on origin. 
  int angleStep = STEPS_PER_A_AXIS_REV / numPoints;      //calculate how much to step the angle over for each point

  for (int i = 0; i < numPoints; i++)
  {
//define each vertex.
//What i have done below is the same as doing:
//pointArray[i].radial = radius; pointArray[i].angular = i * angleStep + convertDegreesToSteps(rotationDeg);
//This is called aggregate initialization.

    pointArray[i] = { radius, i * angleStep + (int)convertDegreesToSteps(rotationDeg) };
  }

  //Currently all the points in the array are centered on the origin. We need to shift the points to be centered on the
  //desired center point. You can do this in polar coordinates, but it's much simpler to convert to rectangular coordinates,
  //move all the points, and then convert back to polar.

  if (centerPoint.radial != 0)
  {        //if the radial coordinate of the center point is not 0, we need to translate the polygon
    translatePoints(pointArray, numPoints, centerPoint);      //This moves all points in the array to be centered on the correct point
  }
}

/**
 * @brief Translates an array of points along a given translation vector, shifting their position in polar coordinates.
 *
 * This function translates the points in the pointArray by converting both the points and the provided
 * translation vector from polar to rectangular coordinates, performing the translation, and then converting
 * the points back to polar coordinates. It is useful for shifting polygons or target positions by a specified
 * offset. For example, this function can be used to shift the center of a polygon generated by nGonGenerator().
 *
 * @param pointArray A pointer to an array of points (of type Positions) representing the points to be translated.
 * @param numPoints The number of points in the array.
 * @param translationVector The translation vector to shift the points by, specified as a Positions struct.
 *
 * @return void - the array is modified directly because it is passed into this function as a pointer.
 *
 * @note The translation is performed by first converting the points and translation vector to rectangular
 * coordinates (x, y), adding the corresponding components, and then converting the updated points back to
 * polar coordinates (r, ?). This ensures that the points are translated accurately in both radial and angular
 * dimensions. The function assumes the angular component of the translation vector is in steps, and the
 * radial component is in motor steps.
 *
 * @example
 * // Example usage to shift a polygon to a new center:
 * Positions vertices[8];
 * Positions translationVector = {3500, convertDegreesToSteps(45)};
 * nGonGenerator(vertices, 8, {0, 0}, 4000, 0.0);
 * translatePoints(vertices, 8, translationVector);
 */
void translatePoints(Positions *pointArray, int numPoints, Positions translationVector)
{
  if (translationVector.angular != 0 || translationVector.radial != 0)
  {    //desired polygon is not centered on origin, so we need to shift the points.
    for (int i = 0; i < numPoints; i++)
    {
      float x = pointArray[i].radial * cos(convertStepsToRadians(pointArray[i].angular));
      float y = pointArray[i].radial * sin(convertStepsToRadians(pointArray[i].angular));

      //now figure out where the center point is in rectangular coordinates
      //NOTE: at some point I want to move this calculation out of the for loop for efficiency
      float centerX = translationVector.radial * cos(convertStepsToRadians(translationVector.angular));
      float centerY = translationVector.radial * sin(convertStepsToRadians(translationVector.angular));

      //now use centerX and centerY to translate each point.
      x += centerX;      //this should shift the X coordinate appropriately
      y += centerY;     //this should shift the Y coordinate appropriately

      //now convert back into polar coordinates

      //calculate the angular position of the current point. Units are in radians.
      //atan2f(y, x) is a special version of the arctan function that returns the angle based on y and x.
      float angleTemp = atan2f(y, x);

      //atan2f() has a range of (-pi, pi), and we'll need to shift that to be (0, 2pi)
      if (angleTemp < 0) angleTemp = 2.0 * PI + angleTemp;    //this is in radians, ranging from 0 to 2pi

      //now that we know the anglular position of the point in radians, we need to find the radial position in units of steps
      //using the Pythagorean theorem (square roots calculate more efficiently than trig functions on Arduino Nano).
      //Then store the r and theta points in the array.
      pointArray[i].radial = round(sqrt(x * x + y * y));   //the radial value of the point. This is absolute coordinates from the origin. Units are steps.

      //store the angular position converted from radians to steps. This is still in absolute coordinates.
      pointArray[i].angular = convertRadiansToSteps(angleTemp);
    }
  }
}

#pragma endregion GeometryGeneration

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
This region of code contains useful math functions for doing things like converting between units, doing modulus math that doesn't allow negative
numbers, and finding the distance between points in polar coordinates.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region Math


/**
 * @brief Maps a float value from one range to another.
 *
 * This function works similarly to the standard map() function but allows for floating-point inputs
 * and outputs. It maps a float n from a specified input range (in_min to in_max) to a corresponding
 * output range (out_min to out_max).
 *
 * @param n The float value to map.
 * @param in_min The lower bound of the input range.
 * @param in_max The upper bound of the input range.
 * @param out_min The lower bound of the output range.
 * @param out_max The upper bound of the output range.
 *
 * @return float The mapped value in the output range.
 */
float fmap(float n, float in_min, float in_max, float out_min, float out_max)
{
  return (n - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/**
 * @brief Converts an angular measurement in degrees to the corresponding number of steps for a stepper motor.
 *
 * This function converts a given angle in degrees to the number of motor steps required for the stepper motor
 * to rotate by that angle. The conversion is based on the number of steps per full revolution of the motor.
 *
 * @param degrees The angle in degrees to convert.
 *
 * @return long The number of steps required for the motor to move the specified angle.
 */
long convertDegreesToSteps(float degrees)
{
  return round(fmap(degrees, 0.0, 360.0, 0.0, 2.0 * STEPS_PER_MOTOR_REV));
}

/**
 * @brief Converts an angular measurement in radians to the corresponding number of steps for a stepper motor.
 *
 * @param rads The angle in radians to convert.
 * @return long The number of steps required for the motor to move the specified angle in radians.
 */
long convertRadiansToSteps(float rads)
{
  return round(fmap(rads, 0.0, 2.0 * PI, 0.0, 2.0 * STEPS_PER_MOTOR_REV));
}

/**
 * @brief Converts a number of steps to the corresponding angle in radians.
 *
 * @param steps The number of motor steps to convert.
 * @return float The equivalent angle in radians.
 */
float convertStepsToRadians(float steps)
{
  return fmap(steps, 0.0, 2.0 * STEPS_PER_MOTOR_REV, 0.0, 2.0 * PI);
}

/**
 * @brief Converts a number of steps to the corresponding angle in degrees.
 *
 * @param steps The number of motor steps to convert.
 * @return float The equivalent angle in degrees.
 */
float convertStepsToDegrees(int steps)
{
  return fmap(float(steps), 0.0, 2.0 * STEPS_PER_MOTOR_REV, 0.0, 360.0);
}


/**
 * @brief Converts a distance in millimeters to the corresponding number of steps for the radial axis.
 *
 * @param mm The distance in millimeters to convert.
 * @return int The equivalent number of steps.
 */
int convertMMToSteps(float mm)
{
  return round(mm * STEPS_PER_MM);
}


/**
 * @brief Converts a number of steps to the corresponding distance in millimeters for the radial axis.
 *
 * @param steps The number of motor steps to convert.
 * @return float The equivalent distance in millimeters.
 */
float convertStepsToMM(float steps)
{
  return steps * MM_PER_STEP;
}


/**
 * @brief Computes the modulus of two integers, ensuring the result is non-negative.
 *
 * This function is a replacement for the % operator that prevents negative results by wrapping
 * negative values around to the positive range. It is mainly used for handling angular values
 * when the gantry wraps from 360 degrees to 0 degrees.
 *
 * @param x The dividend.
 * @param y The divisor.
 *
 * @return int The modulus result, always non-negative.
 */
int modulus(int x, int y)
{
  return x < 0?((x + 1) % y) + y - 1:x % y;
}


/**
 * @brief Calculates the distance between two points in polar coordinates using the law of cosines.
 *
 * This function computes the distance between two points specified in polar coordinates (radii and angles).
 * It uses the law of cosines to perform the calculation, assuming the angles are provided in degrees
 * and the radii in arbitrary units. The returned distance is in the same units as the radii.
 *
 * @param p1 The first point, represented as a Positions struct (with radial and angular values).
 * @param p2 The second point, represented as a Positions struct (with radial and angular values).
 *
 * @return int The calculated distance between the two points, rounded to the nearest integer.
 */
int calculateDistanceBetweenPoints(Positions p1, Positions p2)
{
  return round(sqrt(pow(p1.radial, 2) + pow(p2.radial, 2) - 2 * p1.radial * p2.radial * cos(convertStepsToRadians(p2.angular) - convertStepsToRadians(p1.angular))));
}

#pragma endregion Math


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
This region of code contains the different pattern generating functions.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region Patterns

/**
 * @brief Pattern: Simple Spiral. Generates the next target position for drawing a simple inward and outward spiral.
 *
 * This function calculates the next target position for a simple spiral pattern, starting from the current position.
 * The pattern progresses by incrementally adding small values to the current angular and radial positions. The spiral
 * moves inward more quickly than it moves outward due to the mechanical relationship between the radial and angular axes.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows for restarting the pattern (not used in this simple version). Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note The pattern starts from the current position, so if the previous pattern leaves the ball in a specific position,
 * the spiral will continue from there. The pattern adjusts the radial and angular steps incrementally and reverses
 * direction when the radial boundaries are reached.
 */
Positions pattern_SimpleSpiral(Positions current, bool restartPattern)
{
  Positions target;                                        //This is where we'll store the value of the next target position.

  const float angleDivisions = 100.0;                      //Going to divide a full revolution into 100ths. "const" because this never changes.
  const int radialDivisions = 10 * (int)angleDivisions;    //Going to divide the radial axis into 1000ths. Try changing the 10 to a different number, like 20.

  //Calculate how many degrees we'll move over in the angular axis for the next step.
  const int angleStep = convertDegreesToSteps(360.0 / angleDivisions);

  //Calculate how far along we'll move the radial axis for the next step. 
  //The "static" keyword means that this variable is defined once when the function is run for the first time.
  //This is different than "const" because this is a variable, not a constant, so we can still change the value.
  //If the following line were to omit the "static" keyword, this variable would be reset to its initial value
  //every time the function is called, meaning that we couldn't change it between positive and negative to 
  //make the spiral grow inward or outward.
  static int radialStep = MAX_R_STEPS / radialDivisions;

  target.angular = current.angular + angleStep;            //Set the angular position of the new point to move to (target position)
  target.radial = current.radial + radialStep;             //Set the radial target position as the current radial position plus the radialStep

  if (target.radial > MAX_R_STEPS || target.radial < 0)
  {  //Logic to see if the targeted radial position is out of bounds of the radial axis
    radialStep *= -1;                                      //If we're out of bounds, switch the sign of the radialStep (now we move in the opposite direction)
    target.radial += 2 * radialStep;                       //We were already out of bounds, so we undo that by moving 2 radialSteps in the new direction.
  }

  return target;                                           //Return the target position so that the motion control functions can move to it.
}


/**
 * @brief Pattern: Cardioids. Generates the next target position for drawing repeating, slowly rotating cardioids.
 *
 * This function generates the next target position for a cardioid pattern, moving in relative coordinates by adding 43 degrees
 * to the current angular position and adjusting the radial position by 1/8th of the total radial axis. The pattern alternates
 * the direction of radial movement, creating a stepped approximation of a triangle wave along the radial axis.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern, setting the angular and radial positions to 0. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note This pattern works best after a reset, as it always operates in relative coordinates. If started after running another pattern,
 * the results may vary, since it builds upon the current position of the gantry.
 */
Positions pattern_Cardioids(Positions current, bool restartPattern)
{
  Positions target;
  const int radialStep = ((MAX_R_STEPS) / 8);       //we're going to take huge steps radially (this defaults to 1/8th of the radial axis)
  static int direction = 1;                         //1 means counterclockwise, -1 means clockwise
  static bool firstRun = true;

  if (firstRun || restartPattern)
  {                 //if it's the first time we're running the pattern, or if we start it from another pattern
    target.angular = 0;
    target.radial = 0;
    firstRun = false;
  }
  else
  {

    target.angular = current.angular + convertDegreesToSteps(43);   //add 43 degrees to current position

    //this block of code moves the radial axis back and forth in increments that are 1/8th the length of the total radial axis.
    //Basically, this is a stepped approximation of a triangle wave.

    int nextRadial = current.radial + (direction * radialStep);      //calculate potential next radial position

    if ((nextRadial <= MAX_R_STEPS) && (nextRadial >= 0))
    {          //If the next radial position is in bounds of the radial axis soft limits
      target.radial = nextRadial;                                    //Moves the radial axis positive direction by 1/8th the length of the axis
    }
    else
    {
      direction *= -1;                                               //switch the radial step direction
      target.radial = current.radial + (direction * radialStep);
    }
  }

  return target;
}


/**
 * @brief Pattern: Wavy Spiral. Generates the next target position for drawing a wavy spiral pattern.
 *
 * This function creates a wavy spiral pattern, which is similar to the simple spiral pattern but with an additional sine wave
 * component added to the radial position. The result is a spiral with oscillating radial movement, creating a wavy effect.
 * The sine wave's amplitude and frequency can be adjusted to control the wave's characteristics.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note This pattern adds a sine wave to the radial position to create the wavy effect. You can modify the amplitude and frequency
 * of the wave to achieve different variations of the pattern. The radial movement is reversed when the limits of the radial axis are reached.
 */
Positions pattern_WavySpiral(Positions current, bool restartPattern)
{

  Positions target;                                  //This is where we'll store the value of the next target position.

  float angleDivisions = 100.0;                      //Going to divide a full revolution into 100ths. "const" because this never changes.
  int radialDivisions = 10 * (int)angleDivisions;    //Going to divide the radial axis into 1000ths. Try changing the 10 to a different number, like 20.

  //Add in values for the amplitude and frequency of the sine wave
  float amplitude = 200.0;
  int period = 8;

  //Calculate how many degrees we'll move over in the angular axis for the next step.
  const int angleStep = convertDegreesToSteps(360.0 / angleDivisions);

  static int radialStep = MAX_R_STEPS / radialDivisions;

  target.angular = current.angular + angleStep;            //Set the angular position of the new point to move to (target position)
  target.radial = current.radial + radialStep;             //Set the radial target position as the current radial position plus the radialStep

  if (target.radial > MAX_R_STEPS || target.radial < 0)
  {  //Logic to see if the targeted radial position is out of bounds of the radial axis
    radialStep *= -1;                                      //If we're out of bounds, switch the sign of the radialStep (now we move in the opposite direction)
    target.radial += 2 * radialStep;                       //We were already out of bounds, so we undo that by moving 2 radialSteps in the new direction.
  }

  target.radial += (int)(amplitude * sin(period * convertStepsToRadians(target.angular)));

  return target;                                           //Return the target position so that the motion control functions can move to it.
}



/**
 * @brief Pattern: Rotating Squares. Generates the next target position for drawing rotating squares, each rotated by 10 degrees.
 *
 * This function draws squares of the same size by connecting four points in sequence and rotating the square by 10 degrees
 * after completing each one. The function uses a switch-case statement to control the drawing process, ensuring each side
 * of the square is drawn in order. Once a square is complete, the vertices are rotated for the next iteration.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note This pattern relies on a static variable to track the current step in the drawing process and uses the drawLine function
 * to move between the vertices of the square. After each square is completed, the vertices are rotated by 10 degrees for the next square.
 */
Positions pattern_RotatingSquares(Positions current, bool restartPattern)
{
  Positions target;
  static int step = 0;
  static int segments = 20;                         //Use  20 points to approximate a straight line
  static Positions p1, p2, p3, p4;                  //the four vertices of our square
  static bool firstRun = true;                      //used to track if this is the first time the function is called
  const int angleShift = convertDegreesToSteps(10); //how much we'll rotate the square
  if (firstRun || restartPattern)
  {
    p1.angular = 0;                                 //angular position of first point in absolute coordinates
    p1.radial = 7000;                               //radial position of first point in absolute coordiantes (units are steps)
    p2.angular = convertDegreesToSteps(90);
    p2.radial = 7000;
    p3.angular = convertDegreesToSteps(180);
    p3.radial = 7000;
    p4.angular = convertDegreesToSteps(270);
    p4.radial = 7000;
    firstRun = false;
  }

  switch (step)
  {
    case 0:                                                                   //if step == 0
      target = drawLine(p1, p2, current, segments);                  //target positions are the result of calling drawLine between points p1 and p2
      if ((target.angular == p2.angular) && (target.radial == p2.radial))
      {   //If we've reached the end of the line
        step++;                                                               //Increment the value of step so we can move on to the next line
      }
      break;                                                                  //have to include "break;" to avoid case fall through

    case 1:                                                                   //if step == 1
      target = drawLine(p2, p3, current, segments);
      if ((target.angular == p3.angular) && (target.radial == p3.radial))
      {
        step++;
      }
      break;

    case 2:
      target = drawLine(p3, p4, current, segments);
      if ((target.angular == p4.angular) && (target.radial == p4.radial))
      {
        step++;
      }
      break;

    case 3:
      target = drawLine(p4, p1, current, segments);
      if ((target.angular == p1.angular) && (target.radial == p1.radial))
      {
        step++;                                                               //incrementing again would take us to case 4, but we don't have that, so default gets called next
      }
      break;

    default:
      //assuming that the step number was out of bounds, so reset it
      step = 0;                 //start the square over
      target = current;         //set the target position to the current position just as a default for the default option in the switch statement.
      p1.angular += angleShift; //rotate all points in the square by 10 degrees
      p2.angular += angleShift;
      p3.angular += angleShift;
      p4.angular += angleShift;
      break;
  }

  return target;
}




/**
 * @brief Pattern: Pentagon Spiral. Generates the next target position for drawing a growing and shrinking pentagon spiral.
 *
 * This function creates a pentagon using the nGonGenerator function to generate the vertices and then iterates through
 * the vertices, connecting them with straight lines. After completing a pentagon, the radius of each vertex is adjusted
 * by a radial step value (radialStepover). When the radius exceeds the maximum or falls below zero, the direction of
 * the radial change is reversed, creating a pattern of growing and shrinking pentagons.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note This pattern does not use a switch-case statement for sequencing but instead iterates over a list of precomputed points
 * (the vertices of the pentagon) and adjusts the radial distance of each point to create a spiral effect. The vertices are
 * recalculated when a complete pentagon is drawn.
 */

Positions pattern_PentagonSpiral(Positions current, bool restartPattern)
{
  Positions target;                                                   //Output position will be stored here
  static int start = 0;                                               //Index to the starting point of the line in the array
  static int end = 1;                                                 //Index to the end point of the line in the array
  static bool firstRun = true;                                        //Flag for tracking if a new polygon needs to be generated
  const int vertices = 5;                                             //Change this to make a different polygon
  static Positions vertexList[vertices];                               //construct an array to store the vertices of the polygon
  static int radialStepover = 500;                                    //Amount to change the radius of the polygon each cycle

  if (firstRun || restartPattern)
  {                                                     //On first function call, construct the polygon vertices
    nGonGenerator(vertexList, vertices, { 0,0 }, 1000, 0.0);             //generate the vertices of the polygon  
    firstRun = false;                                                 //Use already generated points next time this function is called
  }
  target = drawLine(vertexList[start], vertexList[end], current, 100);  //draw the line between the appropriate points

  if ((target.angular == vertexList[end].angular) &&                   //If the line is complete, need to move on to the next line
    (target.radial == vertexList[end].radial))
  {
    start++;                                                          //increment start and end points of the line in the array
    end++;
    start = modulus(start, vertices);                                 //wrap around to beginning of array if needed
    end = modulus(end, vertices);
    if (start == 0 && end == 1)
    {                                     //If we're onto a new iteration of the polygon
      for (int i = 0; i < vertices; i++)
      {                            //Increase or decrease the radius of each point
        int newR = vertexList[i].radial + radialStepover;
        if (newR > MAX_R_STEPS || newR < 0)
        {                         //If the radius is getting out of bounds
          radialStepover *= -1;                                       //Switch direction of radial change
          newR += 2 * radialStepover;                                 //move the other way
        }
        vertexList[i].radial = newR;                                   //change the radius for each point
      }
    }
  }
  return target;                                                      //return the new target position
}





/**
 * @brief Pattern: Hex Vortex. Generates the next target position for drawing a series of growing, shrinking, and rotating hexagons.
 *
 * This function generates a hexagon vortex pattern, where hexagons grow and shrink over time while rotating.
 * When the outer edge of the radial axis is reached, the ball moves along the rim before shrinking back inward.
 * The ball also dwells at the center of the field. The pattern is controlled using a switch-case sequence that
 * moves between the six vertices of the hexagon.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note The hexagon grows and shrinks by adjusting the radius incrementally (radialStepover) and rotates
 * by shifting the angular positions of each vertex. The pattern reverses direction when the radius exceeds
 * the maximum limit or falls below zero.
 */
Positions pattern_HexagonVortex(Positions current, bool restartPattern)
{
  Positions target;
  static int step = 0;                                //using switch case to track steps again
  static int segments = 100;
  static Positions p1, p2, p3, p4, p5, p6;            //vertices of the hexagon
  static bool firstRun = true;
  const int angleShift = convertDegreesToSteps(5);
  static int radialStepover = 350;                    //how much we'll increase or decrease the size of the hexagon each iteration
  static int radius = 1000;                           //starting radius

  if (firstRun || restartPattern)
  {
    p1.angular = 0;
    p1.radial = radius;
    p2.angular = convertDegreesToSteps(60);
    p2.radial = radius;
    p3.angular = convertDegreesToSteps(120);
    p3.radial = radius;
    p4.angular = convertDegreesToSteps(180);
    p4.radial = radius;
    p5.angular = convertDegreesToSteps(240);
    p5.radial = radius;
    p6.angular = convertDegreesToSteps(300);
    p6.radial = radius;
    firstRun = false;
  }

  //the step sequencer works just like the rotating square example, but with more steps
  switch (step)
  {
    case 0:
      target = drawLine(p1, p2, current, segments);
      if ((target.angular == p2.angular) && (target.radial == p2.radial))
      {
        step++;
      }
      break;

    case 1:
      target = drawLine(p2, p3, current, segments);
      if ((target.angular == p3.angular) && (target.radial == p3.radial))
      {
        step++;
      }
      break;

    case 2:
      target = drawLine(p3, p4, current, segments);
      if ((target.angular == p4.angular) && (target.radial == p4.radial))
      {
        step++;
      }
      break;

    case 3:
      target = drawLine(p4, p5, current, segments);
      if ((target.angular == p5.angular) && (target.radial == p5.radial))
      {
        step++;
      }
      break;

    case 4:
      target = drawLine(p5, p6, current, segments);
      if ((target.angular == p6.angular) && (target.radial == p6.radial))
      {
        step++;
      }
      break;

    case 5:
      target = drawLine(p6, p1, current, segments);
      if ((target.angular == p1.angular) && (target.radial == p1.radial))
      {
        step++;
      }
      break;

    case 6:
      //reset to the beginning
      step = 0;
      target = current;         //set the target position to the current position just as a default for the default option in the switch statement.

      p1.angular += angleShift; //rotate all points
      p2.angular += angleShift;
      p3.angular += angleShift;
      p4.angular += angleShift;
      p5.angular += angleShift;
      p6.angular += angleShift;

      if ((radius + radialStepover >= MAX_R_STEPS + 2000) || (radius + radialStepover <= 0)) radialStepover *= -1;    //If we're too far out of bounds, switch directions
      radius += radialStepover;  //increase or decrease the radius for the points

      p1.radial = radius;
      p2.radial = radius;
      p3.radial = radius;
      p4.radial = radius;
      p5.radial = radius;
      p6.radial = radius;

      break;

    default:
      //assuming that the step number was out of bounds, so reset it
      step = 0;
      target = current;         //set the target position to the current position just as a default for the default option in the switch statement.
      break;
  }

  return target;
}






/**
 * @brief Pattern: Pentagon Rainbow. Generates the next target position for drawing an off-center pentagon that rotates and moves.
 *
 * This function creates a pentagon pattern that is off-center, moving the center of the pentagon to a new position and
 * rotating it slightly with each iteration. The pentagon is generated using nGonGenerator and translated to the
 * appropriate location, while the center and orientation are adjusted progressively.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note The center of the pentagon is translated and rotated slightly on each iteration, creating a "rainbow" effect
 * as the pentagon appears in different positions. The nGonGenerator and translatePoints functions are used to
 * generate and move the pentagon.
 */
Positions pattern_PentagonRainbow(Positions current, bool restartPattern)
{
  Positions target;
  //target = current;               
  static int start = 0;
  static int end = 1;
  static bool firstRun = true;
  const int vertices = 5;
  static Positions pointList[vertices];
  static int radialStepover = 500;
  const int shiftDeg = 2;
  static int angleShift = convertDegreesToSteps(shiftDeg);
  static int shiftCounter = 1;

  if (firstRun || restartPattern)
  {
    nGonGenerator(pointList, vertices, { 0, 0 }, 3000, 0.0);      //create the polygon
    translatePoints(pointList, vertices, { 4000, 0 });            //move the polygon to the appropriate spot
    firstRun = false;
  }


  target = drawLine(pointList[start], pointList[end], current, 100);

  if ((target.angular == pointList[end].angular) && (target.radial == pointList[end].radial))
  {
    start++;
    end++;
    start = modulus(start, vertices);
    end = modulus(end, vertices);
    nGonGenerator(pointList, vertices, { 0, 0 }, 3000, shiftCounter * shiftDeg);    //build a new polygon that is rotated relative to the previous one
    translatePoints(pointList, vertices, { 4000, shiftCounter * angleShift });      //move to the correct point
    shiftCounter++;
  }
  return target;
}



/**
 * @brief Pattern: Random Walk 1. Generates random target positions, moving via the shortest path to each point.
 *
 * This function creates a random walk pattern by generating random target positions in both the radial and angular axes.
 * The motion controller moves the gantry via the shortest path to each randomly generated point, resulting in random arcs.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next randomly generated target position for the motion controller, represented as a Positions struct.
 *
 * @note This pattern moves the gantry using the shortest path between points, leading to random arc-shaped movements.
 */
Positions pattern_RandomWalk1(Positions current, bool restartPattern)
{
  Positions target;

  // Generate a random radial position within the bounds of your system.
  int randomRadial = random(0, MAX_R_STEPS + 1); // +1 because the upper bound is exclusive

  // Generate a random angular position within a full circle in steps.
  int randomAngular = random(0, STEPS_PER_A_AXIS_REV);

  // Set the target position to the randomly generated values.
  target.radial = randomRadial;
  target.angular = randomAngular;

  return target;
}


/**
 * @brief Pattern: Random Walk 2. Generates random target positions and moves in straight lines to each point.
 *
 * This function creates a random walk pattern by generating random target positions in both the radial and angular axes.
 * Unlike Random Walk 1, this version moves the gantry in straight lines to each random point by connecting the current
 * position to the random target using the drawLine function.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note The function generates new random points once the gantry reaches the current random target and continues the random walk.
 */
Positions pattern_RandomWalk2(Positions current, bool restartPattern)
{
  Positions target;
  static Positions randomPoint, lastPoint = current;
  static bool makeNewRandomPoint = true;

  if (makeNewRandomPoint)
  {
// Generate a random radial position within the bounds of your system.
    randomPoint.radial = random(0, MAX_R_STEPS + 1); // +1 because the upper bound is exclusive

    // Generate a random angular position within a full circle in steps.
    randomPoint.angular = random(0, STEPS_PER_A_AXIS_REV);
    makeNewRandomPoint = false;
  }

  // Set the target position to the randomly generated values.
  target = drawLine(lastPoint, randomPoint, current, 100);

  if (target.angular == randomPoint.angular && target.radial == randomPoint.radial)
  {
    makeNewRandomPoint = true;        //next time we'll generate a new random point
    lastPoint = randomPoint;          //save this as the previous point for the next iteration
  }

  return target;
}



/**
 * @brief Pattern: Accidental Butterfly. Generates the next target position for drawing a butterfly-like pattern with oscillating radial and angular movement.
 *
 * This function creates a butterfly-shaped pattern by modifying a simple spiral pattern with sine and cosine waves that adjust both the radial
 * and angular positions. The radial and angular positions are oscillated to create the butterfly pattern. I was actually trying to do something
 * entirely different and accidentally made this butterfly.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note The pattern adds sine and cosine-based offsets to both the radial and angular positions to create oscillating movements, leading to the butterfly shape.
 * The amplitude and frequency of the sine and cosine waves can be adjusted for different effects.
 */
Positions pattern_AccidentalButterfly(Positions current, bool restartPattern)
{
//This pattern starts out exactly the same as pattern_SimpleSpiral. The only difference is that after calculating the next position
//in the spiral, it adds the sine of the current angular position to the radial axis to make a wavy line.

  Positions target;                                        //This is where we'll store the value of the next target position.

  const float angleDivisions = 100.0;                      //Going to divide a full revolution into 100ths. "const" because this never changes.
  const int radialDivisions = 10 * (int)angleDivisions;    //Going to divide the radial axis into 1000ths. Try changing the 10 to a different number, like 20.

  //Add in values for the amplitude and frequency of the sine wave
  const float amplitude = 200.0;
  const int frequency = 8;

  //Calculate how many degrees we'll move over in the angular axis for the next step.
  const int angleStep = convertDegreesToSteps(360.0 / angleDivisions);

  //Calculate how far along we'll move the radial axis for the next step. 
  static int radialStep = MAX_R_STEPS / radialDivisions;

  target.angular = current.angular + angleStep;            //Set the angular position of the new point to move to (target position)
  target.radial = current.radial + radialStep;             //Set the radial target position as the current radial position plus the radialStep

  if (target.radial > MAX_R_STEPS || target.radial < 0)
  {  //Logic to see if the targeted radial position is out of bounds of the radial axis
    radialStep *= -1;                                      //If we're out of bounds, switch the sign of the radialStep (now we move in the opposite direction)
    target.radial += 2 * radialStep;                       //We were already out of bounds, so we undo that by moving 2 radialSteps in the new direction.
  }

  //Add a new component to the radial position to make it oscillate in and out as a sine wave.
  int rOffset = (int)(200.0 * sin(8 * convertStepsToRadians(target.angular)));
  int aOffset = (int)(40.0 * cos(3 * convertStepsToRadians(target.angular)));
  target.radial += rOffset;


  //Now do the same for the angular axis so we get some back and forth:
  target.angular += aOffset;

  return target;        //Return the target position so that the motion control functions can move to it.
}




#pragma endregion Patterns
