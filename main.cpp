// sdl3_software_pixel.cpp
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <cstdint>
#include <cstdlib>
#include <iostream>

// width / height of window / buffer
int WIDTH = 800;
int HEIGHT = 600;
struct Vector3
{
    float x;
    float y;
    float z;
};
float Length(Vector3 v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

float DotProduct(Vector3 v, Vector3 w)
{
    return (v.x * w.x + v.y * w.y + v.z * w.z);
}
Vector3 SubstractV2FromV1(Vector3 v, Vector3 w)
{
    return(Vector3{ v.x - w.x, v.y - w.y, v.z - w.z });
}
Vector3 MultiplyVectorByScalar(Vector3 v, float s)
{
    return Vector3{ s * v.x, s * v.y, s * v.z };
}
Vector3 SumTwoVectors(Vector3 v, Vector3 w)
{
    return Vector3{ v.x + w.x, v.y + w.y, v.z + w.z };
}

void PutPixel(SDL_Surface* surface, int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    SDL_WriteSurfacePixel(surface, WIDTH/2 + x, HEIGHT/2 - y, r, g, b, a);
}

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

    Vector3 O{};
    Vector3 positionOnRay = SumTwoVectors(O, MultiplyVectorByScalar((SubstractV2FromV1(Vector3{ 50,50,0 }, O)),5));

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

		PutPixel(surface, positionOnRay.x, positionOnRay.y, 0, 255, 0, 255); 

        // Then update the window to show surface
        SDL_UpdateWindowSurface(window);
    }

    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
