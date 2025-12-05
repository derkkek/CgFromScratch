// sdl3_software_pixel.cpp
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include "MathUtils.h"
#include "GraphicsUtils.h"


int main(int argc, char* argv[])
{
    RenderContext renderContext{ 0, 0, 800, 600, 1.0f, 1.0f };
    InitSDL(renderContext);

    bool running = true;
    SDL_Event event;
    
    // Initialize camera with position and orientation (matching the JavaScript example)
    Mat4x4 cameraOrientation = MakeRotationAboutY(-30);  // Rotate camera -30 degrees
    Camera cam{ Vector3(-3, 1, 2), cameraOrientation };

    // Create the model ONCE (shared by all instances)
    Model cubeModel = CreateCube();

    // Create multiple instances with different positions, orientations, and scales
    Mat4x4 identityMatrix = MakeIdentityMatrix();
    Mat4x4 rotation195 = MakeRotationAboutY(195);
    
    ModelInstance cube0{ cubeModel, Vector3(-1.5, 0, 7), identityMatrix, 0.75f };
    cube0.transform = ComputeInstanceTransform(cube0);
    
    ModelInstance cube1{ cubeModel, Vector3(1.25, 2.5, 7.5), rotation195, 1.0f };
    cube1.transform = ComputeInstanceTransform(cube1);
    
    ModelInstance cube2{ cubeModel, Vector3(0, -1, 8), identityMatrix, 0.5f };
    cube2.transform = ComputeInstanceTransform(cube2);

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
        RenderModelInstance(cube0, cam, renderContext);
        RenderModelInstance(cube1, cam, renderContext);
        RenderModelInstance(cube2, cam, renderContext);

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
