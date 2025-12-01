// sdl3_software_pixel.cpp
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include "MathUtils.hpp"
#include "GraphicsUtils.hpp"


int main(int argc, char* argv[])
{
    RenderContext renderContext{ 0, 0, 800, 600, 1.0f, 1.0f };
    InitSDL(renderContext);

    bool running = true;
    SDL_Event event;

    // Front face vertices (z=5)
    Vector3 vA = { -2.0f, -0.5f, 5.0f };
    Vector3 vB = { -2.0f,  0.5f, 5.0f };
    Vector3 vC = { -1.0f,  0.5f, 5.0f };
    Vector3 vD = { -1.0f, -0.5f, 5.0f };

    // Back face vertices (z=6)
    Vector3 vAb = { -2.0f, -0.5f, 6.0f };
    Vector3 vBb = { -2.0f,  0.5f, 6.0f };
    Vector3 vCb = { -1.0f,  0.5f, 6.0f };
    Vector3 vDb = { -1.0f, -0.5f, 6.0f };

    while (running)
    {
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_EVENT_QUIT)
            {
                running = false;
            }
            if (event.type == SDL_EVENT_WINDOW_RESIZED
                || event.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED)
            {
                // Recreate the RGBA surface with new dimensions
                SDL_DestroySurface(renderContext.surface);
                int w, h;
                SDL_GetWindowSize(renderContext.window, &w, &h);
                renderContext.windowWidth = w;
                renderContext.windowHeight = h;
                renderContext.surface = SDL_CreateSurface(w, h, SDL_PIXELFORMAT_RGBA8888);
            }
        }

        // Clear the RGBA surface (e.g., to white with full opacity)
        SDL_ClearSurface(renderContext.surface, 0, 0, 0, 255);

        // Draw front face (BLUE) - z=5
        RenderLine(renderContext, ProjectVertex(vA, renderContext), ProjectVertex(vB, renderContext), Color::Blue);
        RenderLine(renderContext, ProjectVertex(vB, renderContext), ProjectVertex(vC, renderContext), Color::Blue);
        RenderLine(renderContext, ProjectVertex(vC, renderContext), ProjectVertex(vD, renderContext), Color::Blue);
        RenderLine(renderContext, ProjectVertex(vD, renderContext), ProjectVertex(vA, renderContext), Color::Blue);

        // Draw back face (RED) - z=6
        RenderLine(renderContext, ProjectVertex(vAb, renderContext), ProjectVertex(vBb, renderContext), Color::Red);
        RenderLine(renderContext, ProjectVertex(vBb, renderContext), ProjectVertex(vCb, renderContext), Color::Red);
        RenderLine(renderContext, ProjectVertex(vCb, renderContext), ProjectVertex(vDb, renderContext), Color::Red);
        RenderLine(renderContext, ProjectVertex(vDb, renderContext), ProjectVertex(vAb, renderContext), Color::Red);

        // Draw connecting edges (GREEN)
        RenderLine(renderContext, ProjectVertex(vA, renderContext), ProjectVertex(vAb, renderContext), Color::Green);
        RenderLine(renderContext, ProjectVertex(vB, renderContext), ProjectVertex(vBb, renderContext), Color::Green);
        RenderLine(renderContext, ProjectVertex(vC, renderContext), ProjectVertex(vCb, renderContext), Color::Green);
        RenderLine(renderContext, ProjectVertex(vD, renderContext), ProjectVertex(vDb, renderContext), Color::Green);

        // CRITICAL: Blit your RGBA surface to the window surface
        SDL_Surface* windowSurface = SDL_GetWindowSurface(renderContext.window);
        SDL_BlitSurface(renderContext.surface, NULL, windowSurface, NULL);

        // Now update the window
        SDL_UpdateWindowSurface(renderContext.window);
    }

    SDL_DestroySurface(renderContext.surface);
    SDL_DestroyWindow(renderContext.window);
    SDL_Quit();
    return 0;
}
