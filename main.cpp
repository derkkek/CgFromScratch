// sdl3_software_pixel.cpp
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include "MathUtils.hpp"
#include "GraphicsUtils.hpp"

// width / height of window / buffer
int WIDTH = 800;
int HEIGHT = 600;

Vector3 C{};
float radius = 20.0f;

int main(int argc, char* argv[]) {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("SDL3 Soft-Pixel Test",
        WIDTH, HEIGHT,
        SDL_WINDOW_RESIZABLE);
    if (!window) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << "\n";
        SDL_Quit();
        return 1;
    }

    // Get the window surface (software buffer)
    SDL_Surface* surface = SDL_GetWindowSurface(window);
    if (!surface) {
        std::cerr << "SDL_GetWindowSurface failed: " << SDL_GetError() << "\n";
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    bool running = true;
    SDL_Event event;

    Vector3 O{0, 0, 0};
    float viewportWidth = 1.0f;
    float viewportHeight = 1.0f;

    Sphere sphere0{ Vector3(0, -1, 3) , 1, Vector3(255, 0, 0) };

    Sphere sphere1{ Vector3(2, 0, 4) , 1, Vector3(0, 0, 255) };
    Sphere sphere2{ Vector3(-2, 0, 4) , 1, Vector3(0, 255, 0) };
    std::vector<Sphere> spheres;
    spheres.push_back(sphere0);
    spheres.push_back(sphere1);
    spheres.push_back(sphere2);

    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) {
                running = false;
            }
            if (event.type == SDL_EVENT_WINDOW_RESIZED
                || event.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED) {
                surface = SDL_GetWindowSurface(window);  // Get updated surface
				WIDTH = surface->w;
				HEIGHT = surface->h;
            }
        }
        
        RenderLine(surface, WIDTH, HEIGHT, Vector3{ 10, 200, 0 },O, Vector3{ 255, 0, 0 });

        // Then update the window to show surface
        SDL_UpdateWindowSurface(window);
    }

    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
