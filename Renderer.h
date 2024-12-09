#pragma once

#include <SDL.h>
#include "Patterns.h"

class Renderer
{
public:
  Renderer(PatternFunction pattern, int speedScale);

  bool Execute();

private:
  bool Init();
  void OnEvent(const SDL_Event &evt);
  void OnLoop();
  void OnRender();
  void OnExit();

  void DrawCircle(int32_t centreX, int32_t centreY, int32_t radius);

private:
  PatternFunction m_pPattern;
  int m_SpeedScale;

  bool m_IsRunning;
  bool m_FirstPass;
  Positions m_CurrentPositions;

  SDL_Window *m_pWindow;
  SDL_Renderer *m_pRenderer;
};

