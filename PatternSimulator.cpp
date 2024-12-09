// PatternSimulator.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <SDL.h>
#undef main
#include "Renderer.h"
#include "Patterns.h"
#include <iostream>
#include <vector>

PatternFunction patterns[] = { pattern_SimpleSpiral, pattern_Cardioids, pattern_WavySpiral, pattern_RotatingSquares, pattern_PentagonSpiral, pattern_HexagonVortex, pattern_PentagonRainbow, pattern_RandomWalk1,
                              pattern_RandomWalk2, pattern_AccidentalButterfly };

std::vector<std::string> patternNames = { "SimpleSpiral", "Cardioids", "WavySpiral", "RotatingSquares", "PentagonSpiral", "HexagonVortex", "PentagonRainbow", "RandomWalk1",
                              "RandomWalk2", "AccidentalButterfly"};

int main()
{
  int num = -1;
  int speedScale = 1;

  while (num < 0 || num >= patternNames.size())
  {
    std::cout << "Select pattern to simulate: " << std::endl;
    for (int i = 0; i < patternNames.size(); i++)
      std::cout << (i + 1) << ": " << patternNames[i] << std::endl;
    std::cin >> num;
    num--;

    if (num < 0 || num >= patternNames.size())
      std::cout << "Invalid choice!" << std::endl << std::endl;
  }
  std::cout << "Enter speed scale (1-10000): ";
  std::cin >> speedScale;
  speedScale = constrain(speedScale, 1, 10000);

  Renderer visualizer(patterns[num], speedScale);
  if (!visualizer.Execute())
    return -1;

  return 0;
}
