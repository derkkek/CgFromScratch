#include "MathUtils.h"

Vector3 CrossProduct(Vector3 a, Vector3 b)
{
    //a × b = (a₂b₃ - a₃b₂, a₃b₁ - a₁b₃, a₁b₂ - a₂b₁)
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}
float Length(Vector3 v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vector3 MultiplyVectorByScalar(Vector3 v, float s)
{
    return Vector3{ s * v.x, s * v.y, s * v.z };
}

Line DrawLine(Vector2 P0, Vector2 P1, Vector3 color)
{
    Line line{};
    float dx = P1.x - P0.x;
    float dy = P1.y - P0.y;

    if (abs(dx) > abs(dy))
    {
        if (P0.x > P1.x)
        {
            Vector2 holder;

            holder = P1;
            P1 = P0;
            P0 = holder;
        }
        float dx = P1.x - P0.x;
        float dy = P1.y - P0.y;

        float a = dy / dx;
        float y = P0.y;

        for (float x = P0.x; x <= P1.x; x++)
        {
            line.points.push_back({ x, y });
            y = y + a;
        }
    }
    else
    {
        if (P0.y > P1.y)
        {
            Vector2 holder;

            holder = P1;
            P1 = P0;
            P0 = holder;
        }
        float dx = P1.x - P0.x;
        float dy = P1.y - P0.y;

        float a = dx / dy;
        float x = P0.x;

        for (float y = P0.y; y <= P1.y; y++)
        {
            line.points.push_back({ x, y });
            x = x + a;
        }
    }

    line.color = color;

    return line;

}

Vector3 CanvasToViewport(float x, float y, float canvasWidth, float canvasHeight, float viewportWidth, float viewportHeight)
{
    return Vector3{ x * viewportWidth / canvasWidth, y * viewportHeight / canvasHeight, 1 };
}

QuadraticEquationSolutions IntersectRaySphere(Vector3 O, Vector3 D, Sphere sphere)
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
Vector3 TraceRay(Vector3 O, Vector3 D, float t_min, float t_max, std::vector<Sphere>& spheres)
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