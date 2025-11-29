#pragma once
#include <SDL3/SDL.h>

inline void PutPixel(SDL_Surface* surface, float windowWidth,float windowHeight,int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    SDL_WriteSurfacePixel(surface, windowWidth / 2 + x, windowHeight / 2 - y, r, g, b, a);
}

/*First constructs a line from start to end after that renders it on a sdl surface context.*/
inline void RenderLine(SDL_Surface* surface, float windowWidth, float windowHeight, Vector3 start, Vector3 end, Vector3 color)
{
    Line line = DrawLine(start, end, color);

    for (Vector3 point : line.points)
    {
        PutPixel(surface, windowWidth, windowHeight, point.x, point.y, line.color.x, line.color.y, line.color.z, 255);
    }
}
