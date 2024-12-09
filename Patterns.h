#pragma once

struct Positions
{
  int radial = 0;                     //the units for these values are motor steps
  int angular = 0;
};

int constrain(int value, int min, int max);


/**
 * @brief Typedef for storing pointers to pattern-generating functions.
 *
 * This typedef defines a custom data type PatternFunction for storing pointers to pattern functions.
 * It allows pattern functions to be called by passing the appropriate index number to an array of pattern function pointers,
 * simplifying the process of switching between patterns. Each pattern function takes a Positions struct and a bool as parameters
 * and returns the next target position as a Positions struct.
 *
 * @typedef PatternFunction
 *
 * This typedef enables pattern switching by indexing into an array of pattern functions, making it easy to select and execute
 * different patterns dynamically.
 */
typedef Positions(*PatternFunction)(Positions, bool);

//Math related functions
long convertDegreesToSteps(float degrees);                                      //for converting degrees to motor steps
float convertStepsToDegrees(int steps);                                         //for converting motor steps to degrees
long convertRadiansToSteps(float rads);                                         //For converting radians to steps on angular axis
float convertStepsToRadians(float steps);                                       //For converting steps to radians on angular axis
int convertMMToSteps(float mm);                                                 //for converting millimeters to steps on radial axis
float convertStepsToMM(float steps);                                            //for converting steps to millimeters on the radial axis
float fmap(float n, float in_min, float in_max, float out_min, float out_max);  //version of map() that works for floating point numbers
int modulus(int x, int y);                                                      //Use for wrapping values around at ends of range. like %, but no negative numbers.

//Movement related functions
int findShortestPathToPosition(int current, int target, int wrapValue);         //For finding the shortest path to the new position on the angular axis
int calcRadialChange(int angularMoveInSteps, int radialMoveInSteps);            //for figuring out the relative change on the radial axis
int calcRadialSteps(int current, int target, int angularOffsetSteps);           //For calculating actual number of steps radial motor needs to take.
int calculateDistanceBetweenPoints(Positions p1, Positions p2);                 //calculate distance between two points in polar coordinates. Not currently used, but useful

//Geometry generation functions
Positions drawLine(Positions point0, Positions point1, Positions current, int resolution = 100, bool reset = false);             //For drawing a straight line between two points
void nGonGenerator(Positions *pointArray, int numPoints, Positions centerPoint, int radius, float rotationDeg = 0.0);   //generates a list of points that form a polygon's vertices
void translatePoints(Positions *pointArray, int numPoints, Positions translationVector);              //For moving an array of points along a vector to a new position

//Function prototypes for pattern generators. Each pattern function has to return a struct of type Positions. 
//This will be used as the target position for the motion controller. Note that these are just
//function prototypes. They are put up here to let the compiler know that they will be defined later in the code.
Positions pattern_SimpleSpiral(Positions current, bool restartPattern = false);               //Simple spiral. Grows outward, then inward.
Positions pattern_Cardioids(Positions current, bool restartPattern = false);                  //Cardioids
Positions pattern_WavySpiral(Positions current, bool restartPattern = false);                 //Wavy spiral.
Positions pattern_RotatingSquares(Positions current, bool restartPattern = false);            //Rotating squares
Positions pattern_PentagonSpiral(Positions current, bool restartPattern = false);             //Pentagon spiral
Positions pattern_HexagonVortex(Positions current, bool restartPattern = false);              //Hexagon vortex
Positions pattern_PentagonRainbow(Positions current, bool restartPattern = false);            //Pentagon rainbow
Positions pattern_RandomWalk1(Positions current, bool restartPattern = false);                //Random walk 1 (connected by arcs)
Positions pattern_RandomWalk2(Positions current, bool restartPattern = false);                //Random walk 2 (connected by lines)
Positions pattern_AccidentalButterfly(Positions current, bool restartPattern = false);        //Accidental Butterfly
