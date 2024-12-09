#include "Renderer.h"
#include <thread>
#include <iostream>

Renderer::Renderer(PatternFunction pattern, int speedScale)
  : m_pPattern{ pattern }
  , m_SpeedScale{ speedScale }
  , m_IsRunning{ true }
  , m_FirstPass{ true }
{

}

bool Renderer::Init()
{
  if (m_pWindow != nullptr)
    return true; // Already setup

  if (SDL_Init(SDL_INIT_EVERYTHING) < 0)
    return false;

  m_pWindow = SDL_CreateWindow("Pattern window", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 800, SDL_WINDOW_SHOWN);
  if (m_pWindow == nullptr)
    return false;

  m_pRenderer = SDL_CreateRenderer(m_pWindow, -1, 0);

  return true;
}

bool Renderer::Execute()
{
  SDL_Event evt;

  if (!Init())
    return false;

  //Set background to white
  SDL_SetRenderDrawColor(m_pRenderer, 255, 255, 255, 255);
  SDL_RenderClear(m_pRenderer);
  SDL_SetRenderDrawColor(m_pRenderer, 255, 0, 0, 255);
  for (int radius = 390; radius <= 400; radius++)
    DrawCircle(400, 400, radius);
  SDL_RenderPresent(m_pRenderer);
  OnRender();

  while (m_IsRunning)
  {
    while (SDL_PollEvent(&evt) != 0)
      OnEvent(evt);

    OnLoop();
    OnRender();
  }
  OnExit();
  return true;
}

void Renderer::OnEvent(const SDL_Event &evt)
{
  if (evt.type == SDL_QUIT)
    m_IsRunning = false;
}

void Renderer::OnLoop()
{
  auto targetPositions = m_pPattern(m_CurrentPositions, m_FirstPass);

  m_FirstPass = false;

  // Draw red rect
  SDL_SetRenderDrawColor(m_pRenderer, 0, 0, 0, 255);
  auto x1 = (int)(m_CurrentPositions.radial * cos(convertStepsToRadians(m_CurrentPositions.angular)));
  auto y1 = (int)(m_CurrentPositions.radial * sin(convertStepsToRadians(m_CurrentPositions.angular)));
  auto x2 = (int)(targetPositions.radial * cos(convertStepsToRadians(targetPositions.angular)));
  auto y2 = (int)(targetPositions.radial * sin(convertStepsToRadians(targetPositions.angular)));
  
  // Adjust from steps to screen
  x1 = x1/20 + 400;
  x2 = x2/20 + 400;
  y1 = y1/20 + 400;
  y2 = y2/20 + 400;
  SDL_RenderDrawLine(m_pRenderer, x1, y1, x2, y2);

  auto dist = calculateDistanceBetweenPoints(m_CurrentPositions, targetPositions);
  std::this_thread::sleep_for(std::chrono::microseconds(dist * (10000 / m_SpeedScale)));

  m_CurrentPositions = targetPositions;
}

void Renderer::OnRender()
{
  // Show result
  SDL_RenderPresent(m_pRenderer);
}

void Renderer::OnExit()
{
  SDL_DestroyWindow(m_pWindow);
  m_pWindow = nullptr;
  m_pRenderer = nullptr;
  SDL_Quit();
}

void Renderer::DrawCircle(int32_t centreX, int32_t centreY, int32_t radius)
{
  const int32_t diameter = (radius * 2);

  int32_t x = (radius - 1);
  int32_t y = 0;
  int32_t tx = 1;
  int32_t ty = 1;
  int32_t error = (tx - diameter);

  while (x >= y)
  {
     //  Each of the following renders an octant of the circle
    SDL_RenderDrawPoint(m_pRenderer, centreX + x, centreY - y);
    SDL_RenderDrawPoint(m_pRenderer, centreX + x, centreY + y);
    SDL_RenderDrawPoint(m_pRenderer, centreX - x, centreY - y);
    SDL_RenderDrawPoint(m_pRenderer, centreX - x, centreY + y);
    SDL_RenderDrawPoint(m_pRenderer, centreX + y, centreY - x);
    SDL_RenderDrawPoint(m_pRenderer, centreX + y, centreY + x);
    SDL_RenderDrawPoint(m_pRenderer, centreX - y, centreY - x);
    SDL_RenderDrawPoint(m_pRenderer, centreX - y, centreY + x);

    if (error <= 0)
    {
      ++y;
      error += ty;
      ty += 2;
    }

    if (error > 0)
    {
      --x;
      tx += 2;
      error += (tx - diameter);
    }
  }
}