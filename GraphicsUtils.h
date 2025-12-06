#pragma once
#include <SDL3/SDL.h>
#include <iostream>
#include <vector>
#include "MathUtils.h"


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
    int v0, v1, v2;
};

struct Model
{
    std::string name;
    std::vector<Vector3> vertices;
    std::vector<Triangle> triangles;
    Vector3 bounds_center;
    float bounds_radius;
};

struct Plane
{
    Vector3 normal;
    float distance;
};

struct Camera
{
    Vector3 position;
    Mat4x4 orientation;
    std::vector<Plane> clipping_planes;
};

struct ModelInstance
{
    Model model;
    Vector3 position;
    Mat4x4 orientation;
    float scale;
    Mat4x4 transform;
};

namespace Color
{
    constexpr Vector3 Red = { 255, 0, 0 };
    constexpr Vector3 White = { 255, 255, 255 };
    constexpr Vector3 Blue = { 0, 0, 255 };
    constexpr Vector3 Green = { 0, 255, 0 };
}


Vector3 WorldToViewportProjection(Vector3 worldPosition, float cameraToViewportDistance);

Vector2 ViewPortToScreenProjection(Vector3 viewportPos, RenderContext renderContext);

Vector2 ProjectVertex(Vector3 vertex, RenderContext context);

int InitSDL(RenderContext& renderContext);

void PutPixel(SDL_Surface* surface, float windowWidth, float windowHeight, int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a);

/*First constructs a line from start to end after that renders it on a sdl surface context.*/
void RenderLine(RenderContext context, Vector2 start, Vector2 end, Vector3 color);

void DrawWireframeTriangle(RenderContext context, Vector2 P0, Vector2 P1, Vector2 P2, Vector3 color);

std::vector<float> Interpolate(float i0, float d0, float i1, float d1);

void DrawFilledTriangle(RenderContext context, Vector3 P0, Vector3 P1, Vector3 P2, Vector3 color);

Model CreateCube();

void RenderModelInstance(ModelInstance& instance, Camera& camera, RenderContext context);

// Camera helper function
void TranslateCamera(Camera& cam, Vector3 translation);

// Matrix-based transformation functions
Mat4x4 ComputeInstanceTransform(const ModelInstance& instance);

Mat4x4 ComputeCameraMatrix(const Camera& camera);

// Clipping functions
void ClipTriangle(const Triangle& triangle, const Plane& plane, std::vector<Triangle>& triangles, std::vector<Vector3>& vertices);

Model* TransformAndClip(const std::vector<Plane>& clipping_planes, const Model& model, float scale, const Mat4x4& transform);