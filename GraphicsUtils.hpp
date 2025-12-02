#pragma once
#include <SDL3/SDL.h>
#include <iostream>
#include <vector>
#include "MathUtils.hpp"


struct RenderContext
{
    SDL_Window* window;
    SDL_Surface* surface;
    float windowWidth;
    float windowHeight;
    float viewportWidth;
    float viewportHeight;
    float cameraToViewportDistance;
};
struct Triangle
{
    int v0, v1, v2;  // Indices into the vertices array
};

struct Model
{
    std::string name;
    std::vector<Vector3> vertices;
    std::vector<Triangle> triangles;
};
struct ModelInstance
{
    Model model;
    Vector3 position;
};

namespace Color
{
    constexpr Vector3 Red = { 255, 0, 0 };
    constexpr Vector3 White = { 255, 255, 255 };
    constexpr Vector3 Blue = { 0, 0, 255 };
    constexpr Vector3 Green = { 0, 255, 0 };
}

inline Vector3 WorldToViewportProjection(Vector3 worldPosition, float cameraToViewportDistance)
{
    float d = cameraToViewportDistance;
    return Vector3{ worldPosition.x * d / worldPosition.z, worldPosition.y * d / worldPosition.z, d };
}
inline Vector2 ViewPortToScreenProjection(Vector3 viewportPos, RenderContext renderContext)
{
    return Vector2{ viewportPos.x * renderContext.windowWidth / renderContext.viewportWidth, viewportPos.y * renderContext.windowHeight / renderContext.viewportHeight };
}

inline Vector2 ProjectVertex(Vector3 vertex, RenderContext context)
{
    Vector3 worldPos = Vector3{ vertex.x, vertex.y, vertex.z };
    Vector3 viewPortPos = WorldToViewportProjection(worldPos, 1);
    Vector2 screenPos = ViewPortToScreenProjection(viewPortPos, context);
    return screenPos;
}

inline int InitSDL(RenderContext& renderContext)
{
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n";
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("SDL3 Soft-Pixel Test",
        renderContext.windowWidth, renderContext.windowHeight,
        SDL_WINDOW_RESIZABLE);
    if (!window) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << "\n";
        SDL_Quit();
        return 1;
    }

    // Get the window surface (software buffer)
    SDL_Surface* surface = SDL_CreateSurface(
        renderContext.windowWidth,
        renderContext.windowHeight,
        SDL_PIXELFORMAT_RGBA8888
    );
    if (!surface) {
        std::cerr << "SDL_GetWindowSurface failed: " << SDL_GetError() << "\n";
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    renderContext.window = window;
    renderContext.surface = surface;
    renderContext.viewportHeight = 1.0f;
    renderContext.viewportHeight = 1.0f;
    renderContext.cameraToViewportDistance = 1.0f;
    return 0;
}
inline void PutPixel(SDL_Surface* surface, float windowWidth,float windowHeight,int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    SDL_WriteSurfacePixel(surface, windowWidth / 2 + x, windowHeight / 2 - y - 1, r, g, b, a);
}

/*First constructs a line from start to end after that renders it on a sdl surface context.*/
inline void RenderLine(RenderContext context, Vector2 start, Vector2 end, Vector3 color)
{
    Line line = DrawLine(start, end, color);

    for (Vector2 point : line.points)
    {
        PutPixel(context.surface, context.windowWidth, context.windowHeight, point.x, point.y, line.color.x, line.color.y, line.color.z, 255);
    }
}

inline void DrawWireframeTriangle(RenderContext context, Vector2 P0, Vector2 P1, Vector2 P2, Vector3 color)
{
    RenderLine(context, P0, P1, color);
    RenderLine(context, P1, P2, color);
    RenderLine(context, P2, P0, color);
}

std::vector<float> Interpolate(float i0, float d0, float i1, float d1)
{
    std::vector<float> values;
    if (i0 == i1)
    {
        values.push_back(d0);
        return values;
    }
    float a = (d1 - d0) / (i1 - i0);
    float d = d0;
    for (int i = i0; i <= i1; i++)
    {
        values.push_back(d);
        d = d + a;
    }
    return values;
}

inline void DrawFilledTriangle(RenderContext context, Vector3 P0, Vector3 P1, Vector3 P2, Vector3 color)
{
    // Step 1: Sort the points so that y0 <= y1 <= y2
    if (P1.y < P0.y) {
        Vector3 temp = P1;
        P1 = P0;
        P0 = temp;
    }
    if (P2.y < P0.y) {
        Vector3 temp = P2;
        P2 = P0;
        P0 = temp;
    }
    if (P2.y < P1.y) {
        Vector3 temp = P2;
        P2 = P1;
        P1 = temp;
    }

    // Step 2: Compute the x coordinates and h values of the triangle edges
    std::vector<float> x01 = Interpolate(P0.y, P0.x, P1.y, P1.x);
    std::vector<float> h01 = Interpolate(P0.y, P0.z, P1.y, P1.z);

    std::vector<float> x12 = Interpolate(P1.y, P1.x, P2.y, P2.x);
    std::vector<float> h12 = Interpolate(P1.y, P1.z, P2.y, P2.z);

    std::vector<float> x02 = Interpolate(P0.y, P0.x, P2.y, P2.x);
    std::vector<float> h02 = Interpolate(P0.y, P0.z, P2.y, P2.z);

    // Step 3: Concatenate the short sides
    x01.pop_back();
    std::vector<float> x012 = x01;
    x012.insert(x012.end(), x12.begin(), x12.end());

    h01.pop_back();
    std::vector<float> h012 = h01;
    h012.insert(h012.end(), h12.begin(), h12.end());

    // Step 4: Determine which is left and which is right
    int m = x012.size() / 2;
    std::vector<float> x_left, x_right, h_left, h_right;

    if (x02[m] < x012[m])
    {
        x_left = x02;
        h_left = h02;
        x_right = x012;
        h_right = h012;
    }
    else {
        x_left = x012;
        h_left = h012;
        x_right = x02;
        h_right = h02;
    }

    // Step 5: Draw the horizontal segments
    for (int y = P0.y; y <= P2.y; y++)
    {
        int index = y - (int)P0.y;

        if (index >= 0 && index < x_left.size() && index < x_right.size())
        {
            int xl = (int)x_left[index];
            int xr = (int)x_right[index];

            // Interpolate h values across this scanline
            std::vector<float> h_segment = Interpolate(xl, h_left[index], xr, h_right[index]);

            for (int x = xl; x <= xr; x++)
            {
                int x_index = x - xl;  // Convert to 0-based index

                if (x_index >= 0 && x_index < h_segment.size()) {
                    float h = h_segment[x_index];

                    uint8_t r = (uint8_t)(color.x * h);
                    uint8_t g = (uint8_t)(color.y * h);
                    uint8_t b = (uint8_t)(color.z * h);

                    PutPixel(context.surface, context.windowWidth, context.windowHeight,
                        x, y, r, g, b, 255);
                }
            }
        }
    }
}
Model CreateCube()
{
    Model cube;
    cube.vertices = 
    {
        {1,1,1}, {-1,1,1}, {-1,-1,1}, {1,-1,1},
        {1,1,-1}, {-1,1,-1}, {-1,-1,-1}, {1,-1,-1}
    };

    // Define topology using indices
    cube.triangles = 
    {
        {0,1,2}, {0,2,3}, {4,0,3}, {4,3,7},
        {5,4,7}, {5,7,6}, {1,5,6}, {1,6,2},
        {4,5,1}, {4,1,0}, {2,6,7}, {2,7,3}
    };

    return cube;
}
void RenderModelInstance(const ModelInstance& instance, RenderContext context)
{
    for (const Triangle& tri : instance.model.triangles)
    {
        // Get world-space vertices
        Vector3 v0 = instance.model.vertices[tri.v0] + instance.position;
        Vector3 v1 = instance.model.vertices[tri.v1] + instance.position;
        Vector3 v2 = instance.model.vertices[tri.v2] + instance.position;

        // Project to screen space
        Vector2 p0 = ProjectVertex(v0, context);
        Vector2 p1 = ProjectVertex(v1, context);
        Vector2 p2 = ProjectVertex(v2, context);

        DrawWireframeTriangle(context, p0, p1, p2, Color::Blue);
    }
}

inline void TranslateObject(ModelInstance& object, Vector3 translationVector)
{
    for (Vector3& vertex : object.model.vertices)
    {
        Vector3 translation = translationVector;
        vertex = vertex + translation;
    }
}
