#pragma once
#include <corecrt_math.h>
#include <vector>

#define PI 3.14159265359

// Forward declaration to avoid circular dependency
struct RenderContext;

struct Vector3
{
    float x;
    float y;
    float z;
};
struct Vector2
{
    float x;
    float y;
};
struct Vector4
{
    float x;
    float y;
    float z;
    float w;
};
struct Mat4x4
{
    float cells[4][4];
};
struct Mat3x4
{
    float cells[3][4];
};

struct Sphere
{
    Vector3 center;
    float radius;
    Vector3 color;
};
struct Line
{
    std::vector<Vector2> points;
    Vector3 color;
};

struct QuadraticEquationSolutions
{
    float t1;
    float t2;
};


inline Vector3 operator+(Vector3 v, Vector3 w)
{
    return Vector3{ v.x + w.x, v.y + w.y, v.z + w.z };
}
inline Vector3 operator-(Vector3 v, Vector3 w)
{
    return(Vector3{ v.x - w.x, v.y - w.y, v.z - w.z });
}

inline float operator*(Vector3 v, Vector3 w)
{
    return (v.x * w.x + v.y * w.y + v.z * w.z);
}
inline Vector3 operator*(Vector3 v, float s)
{
    return { v.x * s, v.y * s, v.z * s };
}
inline Vector3 operator/(Vector3 v, float s)
{
    return { v.x / s, v.y / s, v.z / s };
}
Vector3 CrossProduct(Vector3 a, Vector3 b);

float Length(Vector3 v);

Vector3 MultiplyVectorByScalar(Vector3 v, float s);

Line DrawLine(Vector2 P0, Vector2 P1, Vector3 color);

/*the function returns constant 1 for z value(?) it's the distance between the camera and the projection plane.*/
Vector3 CanvasToViewport(float x, float y, float canvasWidth, float canvasHeight, float viewportWidth, float viewportHeight);

QuadraticEquationSolutions IntersectRaySphere(Vector3 O, Vector3 D, Sphere sphere);

Vector3 TraceRay(Vector3 O, Vector3 D, float t_min, float t_max, std::vector<Sphere>& spheres);

Mat4x4 MakeScale(float scale_x, float scale_y, float scale_z);

Mat4x4 MakeTranslation(float Tx, float Ty, float Tz);

Mat4x4 MakeRotationAboutX(float angle);

Mat4x4 MakeRotationAboutY(float angle);

Mat4x4 MakeRotationAboutZ(float angle);

// Matrix operations
Mat4x4 MultiplyMat4x4(const Mat4x4& A, const Mat4x4& B);

Vector3 MultiplyMat4x4ByVector3(const Mat4x4& M, const Vector3& v);

Vector4 MultiplyMat4x4ByVector4(const Mat4x4& M, const Vector4& v);

Mat4x4 TransposeMat4x4(const Mat4x4& M);

Mat4x4 MakeIdentityMatrix();