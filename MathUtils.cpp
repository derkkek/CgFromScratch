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

Mat4x4 MakeScale(float scale_x, float scale_y, float scale_z)
{
    return Mat4x4
    {
        {
            { scale_x, 0,       0,       0 },
            { 0,       scale_y, 0,       0 },
            { 0,       0,       scale_z, 0 },
            { 0,       0,       0,       1 }
        }   
    };
    
}
Mat4x4 MakeTranslation(float Tx, float Ty, float Tz)
{
    return Mat4x4
    {
        {
            { 1, 0, 0, Tx },
            { 0, 1, 0, Ty },
            { 0, 0, 1, Tz },
            { 0, 0, 0, 1  }
        }
    };
}

Mat4x4 MakeRotationAboutX(float angle)
{
    float radian = angle * PI / 180;

    return Mat4x4{
    {
        { 1, 0,           0,            0 },
        { 0, cosf(radian), -sinf(radian), 0 },
        { 0, sinf(radian),  cosf(radian), 0 },
        { 0, 0,           0,            1 }
    } };
}
Mat4x4 MakeRotationAboutY(float angle)
{
    float radian = angle * PI / 180;

    return Mat4x4{
    {
        { cosf(radian), 0,  -sinf(radian),0},
        { 0,            1,   0,           0 },
        { sinf(radian), 0,  cosf(radian), 0},
        { 0,            0,  0,            1 }
    } };
}

Mat4x4 MakeRotationAboutZ(float angle)
{
    float radian = angle * PI / 180;

    return Mat4x4{
    {
        { cosf(radian), -sinf(radian), 0, 0},
        { sinf(radian), cosf(radian), 0, 0},
        { 0,            0,            1, 0 },
        { 0,            0,            0, 1 }
    } };
}
Mat3x4 Make3DToCanvas(float camDistanceToViewport, float canvasWidth, float viewportWidth, float canvasHeight, float viewportHeight)
{
    float d = camDistanceToViewport;

    return Mat3x4{
    {
        { d * canvasWidth / viewportWidth, 0, 0, 0},
        { 0, d * canvasHeight / viewportHeight, 0, 0},
        { 0, 0, 1, 0 }
    } };
}

Mat4x4 MultiplyMat4x4(const Mat4x4& A, const Mat4x4& B)
{
    Mat4x4 result = {};
    for (int i = 0; i < 4; i++) 
    {
        for (int j = 0; j < 4; j++) 
        {
            result.cells[i][j] = 0;
            for (int k = 0; k < 4; k++) 
            {
                result.cells[i][j] += A.cells[i][k] * B.cells[k][j];
            }
        }
    }
    return result;
}

Vector3 MultiplyMat4x4ByVector3(const Mat4x4& M, const Vector3& v)
{
    // Treat Vector3 as homogeneous coordinate (x, y, z, 1)
    float x = M.cells[0][0] * v.x + M.cells[0][1] * v.y + M.cells[0][2] * v.z + M.cells[0][3];
    float y = M.cells[1][0] * v.x + M.cells[1][1] * v.y + M.cells[1][2] * v.z + M.cells[1][3];
    float z = M.cells[2][0] * v.x + M.cells[2][1] * v.y + M.cells[2][2] * v.z + M.cells[2][3];
    float w = M.cells[3][0] * v.x + M.cells[3][1] * v.y + M.cells[3][2] * v.z + M.cells[3][3];
    
    // Perspective divide (if w != 1)
    if (w != 1.0f && w != 0.0f) 
    {
        return Vector3{ x / w, y / w, z / w };
    }
    return Vector3{ x, y, z };
}

Vector4 MultiplyMat4x4ByVector4(const Mat4x4& M, const Vector4& v)
{
    float x = M.cells[0][0] * v.x + M.cells[0][1] * v.y + M.cells[0][2] * v.z + M.cells[0][3] * v.w;
    float y = M.cells[1][0] * v.x + M.cells[1][1] * v.y + M.cells[1][2] * v.z + M.cells[1][3] * v.w;
    float z = M.cells[2][0] * v.x + M.cells[2][1] * v.y + M.cells[2][2] * v.z + M.cells[2][3] * v.w;
    float w = M.cells[3][0] * v.x + M.cells[3][1] * v.y + M.cells[3][2] * v.z + M.cells[3][3] * v.w;
    
    return Vector4{ x, y, z, w };
}

Mat4x4 TransposeMat4x4(const Mat4x4& M)
{
    Mat4x4 result = {};
    for (int i = 0; i < 4; i++) 
    {
        for (int j = 0; j < 4; j++) 
        {
            result.cells[i][j] = M.cells[j][i];
        }
    }
    return result;
}

Mat4x4 MakeIdentityMatrix()
{
    return Mat4x4{
        {
            { 1, 0, 0, 0 },
            { 0, 1, 0, 0 },
            { 0, 0, 1, 0 },
            { 0, 0, 0, 1 }
        }
    };
}


