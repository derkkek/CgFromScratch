#include "GraphicsUtils.h"

Vector3 WorldToViewportProjection(Vector3 worldPosition, float cameraToViewportDistance)
{
    float d = cameraToViewportDistance;
    return Vector3{ worldPosition.x * d / worldPosition.z, worldPosition.y * d / worldPosition.z, d };
}
Vector2 ViewPortToScreenProjection(Vector3 viewportPos, RenderContext renderContext)
{
    return Vector2{ viewportPos.x * renderContext.windowWidth / renderContext.viewportWidth, viewportPos.y * renderContext.windowHeight / renderContext.viewportHeight };
}

Vector2 ProjectVertex(Vector3 vertex, RenderContext context)
{
    Vector3 worldPos = Vector3{ vertex.x, vertex.y, vertex.z };
    Vector3 viewPortPos = WorldToViewportProjection(worldPos, 1);
    Vector2 screenPos = ViewPortToScreenProjection(viewPortPos, context);
    return screenPos;
}

int InitSDL(RenderContext& renderContext)
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
void PutPixel(SDL_Surface* surface, float windowWidth,float windowHeight,int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
    SDL_WriteSurfacePixel(surface, windowWidth / 2 + x, windowHeight / 2 - y - 1, r, g, b, a);
}

/*First constructs a line from start to end after that renders it on a sdl surface context.*/
void RenderLine(RenderContext context, Vector2 start, Vector2 end, Vector3 color)
{
    Line line = DrawLine(start, end, color);

    for (Vector2 point : line.points)
    {
        PutPixel(context.surface, context.windowWidth, context.windowHeight, point.x, point.y, line.color.x, line.color.y, line.color.z, 255);
    }
}

void DrawWireframeTriangle(RenderContext context, Vector2 P0, Vector2 P1, Vector2 P2, Vector3 color)
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

void DrawFilledTriangle(RenderContext context, Vector3 P0, Vector3 P1, Vector3 P2, Vector3 color)
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

    // Set bounding sphere (center at origin, radius = sqrt(3) for unit cube)
    cube.bounds_center = Vector3{0, 0, 0};
    cube.bounds_radius = sqrtf(3.0f);

    return cube;
}

Mat4x4 ComputeInstanceTransform(const ModelInstance& instance)
{
    // Build transformation from right to left: T × R × S
    Mat4x4 S = MakeScale(instance.scale, instance.scale, instance.scale);
    Mat4x4 R = instance.orientation;
    Mat4x4 T = MakeTranslation(instance.position.x, instance.position.y, instance.position.z);
    
    return MultiplyMat4x4(T, MultiplyMat4x4(R, S));
}

Mat4x4 ComputeCameraMatrix(const Camera& camera)
{
    // Inverse translation: move camera position to origin
    Mat4x4 invTranslation = MakeTranslation(
        -camera.position.x,
        -camera.position.y,
        -camera.position.z
    );

    // Transpose = inverse for orthogonal rotation matrices
    Mat4x4 invRotation = TransposeMat4x4(camera.orientation);

    // Camera matrix = R^T × T^-1
    return MultiplyMat4x4(invRotation, invTranslation);
}

void RenderModelInstance(ModelInstance& instance, Camera& camera, RenderContext context)
{
    // Compute transforms ONCE per instance
    Mat4x4 cameraMatrix = ComputeCameraMatrix(camera);
    Mat4x4 finalTransform = MultiplyMat4x4(cameraMatrix, instance.transform);
    
    // Transform and clip the model
    Model* clipped = TransformAndClip(camera.clipping_planes, instance.model, instance.scale, finalTransform);
    
    if (clipped == nullptr) {
        // Model was completely clipped out
        return;
    }
    
    // Project all vertices from the clipped model
    std::vector<Vector2> projected;
    for (const Vector3& vertex : clipped->vertices)
    {
        // Vertex is already in camera space, just project to screen
        Vector2 screenPos = ProjectVertex(vertex, context);
        projected.push_back(screenPos);
    }
    
    // Draw triangles using projected vertices
    for (const Triangle& tri : clipped->triangles)
    {
        DrawWireframeTriangle(context,
            projected[tri.v0],
            projected[tri.v1],
            projected[tri.v2],
            Color::Blue);
    }
    
    // Clean up the clipped model
    delete clipped;
}

// Old vertex transformation functions removed - now using matrix pipeline
// These functions were incorrectly modifying the shared model vertices
// The new system uses pre-computed transformation matrices instead
void TranslateCamera(Camera& cam, Vector3 translation)
{
    cam.position = cam.position + translation;
}

// Computes the intersection of a segment and a plane
// Returns the interpolation parameter t where intersection = v0 + t * (v1 - v0)
float ComputeIntersection(const Vector3& v0, const Vector3& v1, const Plane& plane)
{
    float d0 = plane.normal * v0 + plane.distance;
    float d1 = plane.normal * v1 + plane.distance;
    float t = d0 / (d0 - d1);
    return t;
}

// Clips a triangle against a plane. Adds output to triangles and vertices.
void ClipTriangle(const Triangle& triangle, const Plane& plane, 
                  std::vector<Triangle>& triangles, std::vector<Vector3>& vertices)
{
    const Vector3& v0 = vertices[triangle.v0];
    const Vector3& v1 = vertices[triangle.v1];
    const Vector3& v2 = vertices[triangle.v2];

    // Check which vertices are in front of the plane (distance > 0 means inside)
    bool in0 = (plane.normal * v0 + plane.distance) > 0;
    bool in1 = (plane.normal * v1 + plane.distance) > 0;
    bool in2 = (plane.normal * v2 + plane.distance) > 0;

    int in_count = (in0 ? 1 : 0) + (in1 ? 1 : 0) + (in2 ? 1 : 0);

    if (in_count == 0) {
        // Nothing to do - the triangle is fully clipped out.
        return;
    } 
    else if (in_count == 3) {
        // The triangle is fully in front of the plane.
        triangles.push_back(triangle);
    } 
    else if (in_count == 1) {
        // The triangle has one vertex in. Output is one clipped triangle.
        // Find which vertex is inside
        int in_idx, out_idx1, out_idx2;
        Vector3 in_vertex, out_vertex1, out_vertex2;
        
        if (in0) {
            in_idx = triangle.v0; in_vertex = v0;
            out_idx1 = triangle.v1; out_vertex1 = v1;
            out_idx2 = triangle.v2; out_vertex2 = v2;
        } else if (in1) {
            in_idx = triangle.v1; in_vertex = v1;
            out_idx1 = triangle.v0; out_vertex1 = v0;
            out_idx2 = triangle.v2; out_vertex2 = v2;
        } else {
            in_idx = triangle.v2; in_vertex = v2;
            out_idx1 = triangle.v0; out_vertex1 = v0;
            out_idx2 = triangle.v1; out_vertex2 = v1;
        }

        // Compute intersection points
        float t1 = ComputeIntersection(in_vertex, out_vertex1, plane);
        float t2 = ComputeIntersection(in_vertex, out_vertex2, plane);

        Vector3 new_v1 = in_vertex + (out_vertex1 - in_vertex) * t1;
        Vector3 new_v2 = in_vertex + (out_vertex2 - in_vertex) * t2;

        // Add new vertices
        int new_idx1 = vertices.size();
        vertices.push_back(new_v1);
        int new_idx2 = vertices.size();
        vertices.push_back(new_v2);

        // Add clipped triangle
        triangles.push_back(Triangle{in_idx, new_idx1, new_idx2});
    } 
    else if (in_count == 2) {
        // The triangle has two vertices in. Output is two clipped triangles (a quad split into 2 triangles).
        // Find which vertices are inside
        int in_idx1, in_idx2, out_idx;
        Vector3 in_vertex1, in_vertex2, out_vertex;
        
        if (!in0) {
            out_idx = triangle.v0; out_vertex = v0;
            in_idx1 = triangle.v1; in_vertex1 = v1;
            in_idx2 = triangle.v2; in_vertex2 = v2;
        } else if (!in1) {
            out_idx = triangle.v1; out_vertex = v1;
            in_idx1 = triangle.v0; in_vertex1 = v0;
            in_idx2 = triangle.v2; in_vertex2 = v2;
        } else {
            out_idx = triangle.v2; out_vertex = v2;
            in_idx1 = triangle.v0; in_vertex1 = v0;
            in_idx2 = triangle.v1; in_vertex2 = v1;
        }

        // Compute intersection points
        float t1 = ComputeIntersection(in_vertex1, out_vertex, plane);
        float t2 = ComputeIntersection(in_vertex2, out_vertex, plane);

        Vector3 new_v1 = in_vertex1 + (out_vertex - in_vertex1) * t1;
        Vector3 new_v2 = in_vertex2 + (out_vertex - in_vertex2) * t2;

        // Add new vertices
        int new_idx1 = vertices.size();
        vertices.push_back(new_v1);
        int new_idx2 = vertices.size();
        vertices.push_back(new_v2);

        // Add two triangles forming a quad
        triangles.push_back(Triangle{in_idx1, in_idx2, new_idx1});
        triangles.push_back(Triangle{new_idx1, in_idx2, new_idx2});
    }
}

// Transform and clip a model against clipping planes
Model* TransformAndClip(const std::vector<Plane>& clipping_planes, 
                        const Model& model, float scale, const Mat4x4& transform)
{
    // Transform the bounding sphere, and attempt early discard.
    Vector4 center_homo = {model.bounds_center.x, model.bounds_center.y, model.bounds_center.z, 1.0f};
    Vector4 center_transformed = MultiplyMat4x4ByVector4(transform, center_homo);
    Vector3 center = {center_transformed.x / center_transformed.w, 
                      center_transformed.y / center_transformed.w, 
                      center_transformed.z / center_transformed.w};
    
    float radius = model.bounds_radius * scale;

    // Early discard if bounding sphere is completely outside any plane
    for (const Plane& plane : clipping_planes) {
        float distance = plane.normal * center + plane.distance;
        if (distance < -radius) {
            return nullptr;  // Completely clipped
        }
    }

    // Apply modelview transform to all vertices
    std::vector<Vector3> vertices;
    for (const Vector3& vertex : model.vertices) {
        Vector3 transformed = MultiplyMat4x4ByVector3(transform, vertex);
        vertices.push_back(transformed);
    }

    // Clip the entire model against each successive plane
    std::vector<Triangle> triangles = model.triangles;  // Copy triangles
    
    for (const Plane& plane : clipping_planes) {
        std::vector<Triangle> new_triangles;
        for (const Triangle& tri : triangles) {
            ClipTriangle(tri, plane, new_triangles, vertices);
        }
        triangles = new_triangles;
    }

    // Create and return the clipped model
    Model* clipped = new Model();
    clipped->vertices = vertices;
    clipped->triangles = triangles;
    clipped->bounds_center = center;
    clipped->bounds_radius = radius;
    
    return clipped;
}

