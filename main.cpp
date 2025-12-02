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

    Model cube;


    cube.vertices.push_back(Vector3{ 1,1,1 });
    cube.vertices.push_back(Vector3{ -1,1,1 });
    cube.vertices.push_back(Vector3{ -1,-1,1 });
    cube.vertices.push_back(Vector3{ 1,-1,1 });
    cube.vertices.push_back(Vector3{ 1,1,-1 });
    cube.vertices.push_back(Vector3{ -1,1,-1 });
    cube.vertices.push_back(Vector3{ -1,-1,-1 });
    cube.vertices.push_back(Vector3{ 1,-1,-1 });
    //cube.triangles = ConstructTrianglesOfObjectMesh(cube.vertices, renderContext);

    ModelInstance cube0(cube, Vector3(1.5, 0, 7));
    TranslateObject(cube0, cube0.position);
    cube0.model.triangles = ConstructTrianglesOfObjectMesh(cube0.model.vertices, renderContext);

    ModelInstance cube1(cube, Vector3(-1.5, 0, 7));
    TranslateObject(cube1, cube1.position);
    cube1.model.triangles = ConstructTrianglesOfObjectMesh(cube1.model.vertices, renderContext);

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

        RenderObject(cube0.model.triangles, renderContext);
        RenderObject(cube1.model.triangles, renderContext);

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
