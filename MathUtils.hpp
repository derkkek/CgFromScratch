#pragma once
#include <corecrt_math.h>
#include <vector>

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

struct Sphere
{
    Vector3 center;
    float radius;
    Vector3 color;
};
struct Line
{
    std::vector<Vector3> points;
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
inline float Length(Vector3 v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

inline Vector3 MultiplyVectorByScalar(Vector3 v, float s)
{
    return Vector3{ s * v.x, s * v.y, s * v.z };
}

inline Line DrawLine(Vector3 P0, Vector3 P1, Vector3 color)
{
    Line line{};
    float dx = P1.x - P0.x;
    float dy = P1.y - P0.y;

    if (abs(dx) > abs(dy))
    {
        if (P0.x > P1.x)
        {
            Vector3 holder;

            holder = P1;
            P1 = P0;
            P0 = holder;
        }
        float dx = P1.x - P0.x;
        float dy = P1.y - P0.y;

        float a = dy / dx;
        float y = P0.y;

        for (float x = P0.x; x <= dx; x++)
        {
            line.points.push_back({ x, y });
            y = y + a;
        }
    }
    else
    {
        if (P0.y > P1.y)
        {
            Vector3 holder;

            holder = P1;
            P1 = P0;
            P0 = holder;
        }
        float dx = P1.x - P0.x;
        float dy = P1.y - P0.y;

        float a = dx / dy;
        float x = P0.x;

        for (float y = P0.y; y <= dy; y++)
        {
            line.points.push_back({ x, y });
            x = x + a;
        }
    }

    line.color = color;

    return line;

}

/*the function returns constant 1 for z value(?) it's the distance between the camera and the projection plane.*/
inline Vector3 CanvasToViewport(float x, float y, float canvasWidth, float canvasHeight ,float viewportWidth, float viewportHeight)
{
    return Vector3{ x * viewportWidth / canvasWidth, y * viewportHeight / canvasHeight, 1 };
}
inline QuadraticEquationSolutions IntersectRaySphere(Vector3 O, Vector3 D, Sphere sphere)
{
    Vector3 C = sphere.center;
    float r = sphere.radius;

    Vector3 CO = O - C;  // O - C (not C - O)

    float a = D * D;
    float b = 2 * (CO * D);
    float c = (CO * CO) - r * r;

    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0)
    {
        return QuadraticEquationSolutions{ INFINITY, INFINITY };
    }

    float t1 = (-b + sqrt(discriminant)) / (2 * a);
    float t2 = (-b - sqrt(discriminant)) / (2 * a);
    return QuadraticEquationSolutions{ t1, t2 };
}
inline Vector3 TraceRay(Vector3 O, Vector3 D, float t_min, float t_max, std::vector<Sphere>& spheres)
{
    float closestT = INFINITY;
    Sphere* closestSphere = nullptr;

    for (Sphere& sphere : spheres)
    {
        QuadraticEquationSolutions solutions = IntersectRaySphere(O, D, sphere);

        if (solutions.t1 <= t_max && solutions.t1 >= t_min && solutions.t1 < closestT)
        {
            closestT = solutions.t1;
            closestSphere = &sphere;
        }
        if (solutions.t2 <= t_max && solutions.t2 >= t_min && solutions.t2 < closestT)
        {
            closestT = solutions.t2;
            closestSphere = &sphere;
        }
    }

    if (closestSphere == nullptr)
    {
        return Vector3(0, 0, 0);
    }
    return closestSphere->color;
}