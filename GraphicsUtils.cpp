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
    // Initialize depth buffer to 0 since we're using inverse z (1/z)
    // Larger 1/z = closer, so we start with the smallest value (0)
    renderContext.depthBuffer = std::vector<float>(renderContext.windowWidth * renderContext.windowHeight, 0.0f);
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

// Helper function to interpolate Vector3 values (for normals)
std::vector<Vector3> InterpolateVector3(float i0, Vector3 d0, float i1, Vector3 d1)
{
    std::vector<Vector3> values;
    if (i0 == i1)
    {
        values.push_back(d0);
        return values;
    }
    float a = 1.0f / (i1 - i0);
    for (int i = i0; i <= i1; i++)
    {
        float t = (i - i0) * a;
        values.push_back(d0 + (d1 - d0) * t);
    }
    return values;
}

Vector3 CalculatePhongLighting(Vector3 position, Vector3 normal, Vector3 cameraPos, Light light, Material material)
{
    // Normalize the normal
    normal = Normalize(normal);
    
    // Light direction (from surface to light)
    Vector3 lightDir = light.position - position;
    float lightDistance = Length(lightDir);
    if (lightDistance > 0.0001f) {
        lightDir = lightDir / lightDistance;
    } else {
        // Light is at the same position, return only ambient
        Vector3 ambient = material.ambient * (1.0f / 255.0f);
        Vector3 lightColor = light.color * (1.0f / 255.0f) * light.intensity;

        Vector3 finalColor = {ambient.x * lightColor.x, ambient.y * lightColor.y, ambient.z * lightColor.z};
        finalColor.x = finalColor.x * 255.0f;
        finalColor.y = finalColor.y * 255.0f;
        finalColor.z = finalColor.z * 255.0f;
        return finalColor;
    }
    
    // View direction (from surface to camera)
    Vector3 viewDir = cameraPos - position;
    float viewDistance = Length(viewDir);
    if (viewDistance > 0.0001f) {
        viewDir = viewDir / viewDistance;
    } else {
        viewDir = Vector3{0, 0, -1};  // Default view direction
    }
    
    // Ambient component
    Vector3 ambient = material.ambient * (1.0f / 255.0f);
    
    // Diffuse component: max(0, N · L)
    float NdotL = normal * lightDir;
    if (NdotL < 0.0f) NdotL = 0.0f;
    Vector3 diffuse = material.diffuse * (1.0f / 255.0f) * NdotL;
    
    // Specular component: (R · V)^shininess
    // R = 2 * (N · L) * N - L (reflection vector)
    Vector3 reflectDir = normal * (2.0f * NdotL) - lightDir;
    float RdotV = reflectDir * viewDir;
    if (RdotV < 0.0f) RdotV = 0.0f;
    float specularFactor = powf(RdotV, material.shininess);
    Vector3 specular = material.specular * (1.0f / 255.0f) * specularFactor;
    
    // Combine all components with light color and intensity
    Vector3 lightColor = light.color * (1.0f / 255.0f) * light.intensity;
    Vector3 combined = ambient + diffuse + specular;

    Vector3 finalColor = {combined.x * lightColor.x, combined.y * lightColor.y, combined.z * lightColor.z};
    
    // Clamp to 0-255 range
    finalColor.x = (finalColor.x > 1.0f) ? 255.0f : finalColor.x * 255.0f;
    finalColor.y = (finalColor.y > 1.0f) ? 255.0f : finalColor.y * 255.0f;
    finalColor.z = (finalColor.z > 1.0f) ? 255.0f : finalColor.z * 255.0f;
    
    return finalColor;
}

void DrawFilledTriangle(RenderContext& context, Vector3 P0, Vector3 P1, Vector3 P2,
                        Vector3 N0, Vector3 N1, Vector3 N2,
                        Vector3 cameraPos, Light light, Material material)
{
    // Step 1: Sort the points so that y0 <= y1 <= y2
    Vector3 tempP, tempN;
    if (P1.y < P0.y) {
        tempP = P1; tempN = N1;
        P1 = P0; N1 = N0;
        P0 = tempP; N0 = tempN;
    }
    if (P2.y < P0.y) {
        tempP = P2; tempN = N2;
        P2 = P0; N2 = N0;
        P0 = tempP; N0 = tempN;
    }
    if (P2.y < P1.y) {
        tempP = P2; tempN = N2;
        P2 = P1; N2 = N1;
        P1 = tempP; N1 = tempN;
    }

    // Step 2: Convert z values to inverse z (1/z) for perspective-correct interpolation
    // Skip triangles with invalid depth values (z <= 0 means behind camera, should be clipped)
    if (P0.z <= 0.0f || P1.z <= 0.0f || P2.z <= 0.0f) {
        return;  // Skip triangles behind the camera
    }

    float inv_z0 = 1.0f / P0.z;
    float inv_z1 = 1.0f / P1.z;
    float inv_z2 = 1.0f / P2.z;
    
    // For perspective-correct normal interpolation, we need to interpolate N/z and then normalize
    // N0/z0, N1/z1, N2/z2
    Vector3 N0_over_z = N0 * inv_z0;
    Vector3 N1_over_z = N1 * inv_z1;
    Vector3 N2_over_z = N2 * inv_z2;

    // Step 3: Compute the x coordinates, inverse z values, and normals of the triangle edges
    std::vector<float> x01 = Interpolate(P0.y, P0.x, P1.y, P1.x);
    std::vector<float> inv_z01 = Interpolate(P0.y, inv_z0, P1.y, inv_z1);
    std::vector<Vector3> N01_over_z = InterpolateVector3(P0.y, N0_over_z, P1.y, N1_over_z);

    std::vector<float> x12 = Interpolate(P1.y, P1.x, P2.y, P2.x);
    std::vector<float> inv_z12 = Interpolate(P1.y, inv_z1, P2.y, inv_z2);
    std::vector<Vector3> N12_over_z = InterpolateVector3(P1.y, N1_over_z, P2.y, N2_over_z);

    std::vector<float> x02 = Interpolate(P0.y, P0.x, P2.y, P2.x);
    std::vector<float> inv_z02 = Interpolate(P0.y, inv_z0, P2.y, inv_z2);
    std::vector<Vector3> N02_over_z = InterpolateVector3(P0.y, N0_over_z, P2.y, N2_over_z);

    // Step 4: Concatenate the short sides
    x01.pop_back();
    std::vector<float> x012 = x01;
    x012.insert(x012.end(), x12.begin(), x12.end());

    inv_z01.pop_back();
    std::vector<float> inv_z012 = inv_z01;
    inv_z012.insert(inv_z012.end(), inv_z12.begin(), inv_z12.end());
    
    N01_over_z.pop_back();
    std::vector<Vector3> N012_over_z = N01_over_z;
    N012_over_z.insert(N012_over_z.end(), N12_over_z.begin(), N12_over_z.end());

    // Step 5: Determine which is left and which is right
    int m = x012.size() / 2;
    std::vector<float> x_left, x_right, inv_z_left, inv_z_right;
    std::vector<Vector3> N_left_over_z, N_right_over_z;

    if (x02[m] < x012[m])
    {
        x_left = x02;
        inv_z_left = inv_z02;
        N_left_over_z = N02_over_z;
        x_right = x012;
        inv_z_right = inv_z012;
        N_right_over_z = N012_over_z;
    }
    else {
        x_left = x012;
        inv_z_left = inv_z012;
        N_left_over_z = N012_over_z;
        x_right = x02;
        inv_z_right = inv_z02;
        N_right_over_z = N02_over_z;
    }

    // Step 6: Draw the horizontal segments
    for (int y = P0.y; y <= P2.y; y++)
    {
        int index = y - (int)P0.y;

        if (index >= 0 && index < x_left.size() && index < x_right.size())
        {
            int xl = (int)x_left[index];
            int xr = (int)x_right[index];

            // Interpolate inverse z values and normals across this scanline
            std::vector<float> inv_z_segment = Interpolate(xl, inv_z_left[index], xr, inv_z_right[index]);
            std::vector<Vector3> N_segment_over_z = InterpolateVector3(xl, N_left_over_z[index], xr, N_right_over_z[index]);

            for (int x = xl; x <= xr; x++)
            {
                int x_index = x - xl;  // Convert to 0-based index

                if (x_index >= 0 && x_index < inv_z_segment.size() && x_index < N_segment_over_z.size()) 
                {
                    float inv_z = inv_z_segment[x_index];  // This is the interpolated inverse depth value
                    Vector3 N_over_z = N_segment_over_z[x_index];
                    
                    // Recover the normal: N = (N/z) / (1/z) = (N/z) * z
                    float z = 1.0f / inv_z;
                    Vector3 normal = N_over_z * z;
                    normal = Normalize(normal);

                    // Calculate depth buffer index
                    int screen_x = (int)context.windowWidth / 2 + x;
                    int screen_y = (int)context.windowHeight / 2 - y - 1;
                    int depthIndex = screen_y * (int)context.windowWidth + screen_x;
                    
                    // Bounds check for depth buffer
                    if (screen_x >= 0 && screen_x < (int)context.windowWidth && 
                        screen_y >= 0 && screen_y < (int)context.windowHeight &&
                        depthIndex >= 0 && depthIndex < (int)context.depthBuffer.size()) 
                    {
                        // Depth test: only draw if this pixel is closer (larger 1/z = closer)
                        if (inv_z > context.depthBuffer[depthIndex]) 
                        {
                            // Calculate 3D position at this pixel (for lighting calculation)
                            // Reconstruct camera-space position from screen coordinates and depth
                            // Screen coordinates x,y are centered at origin
                            // Reverse the projection: viewport = screen * viewportSize / windowSize
                            // Then: x_cam = viewport_x * z_cam / d, y_cam = viewport_y * z_cam / d (where d=1)
                            float viewport_x = (float)x * context.viewportWidth / context.windowWidth;
                            float viewport_y = (float)y * context.viewportHeight / context.windowHeight;
                            float d = context.cameraToViewportDistance;  // Should be 1.0
                            Vector3 pixelPos = Vector3{viewport_x * z / d, viewport_y * z / d, z};
                            
                            // Calculate Phong lighting
                            Vector3 litColor = CalculatePhongLighting(pixelPos, normal, cameraPos, light, material);
                            
                            uint8_t r = (uint8_t)litColor.x;
                            uint8_t g = (uint8_t)litColor.y;
                            uint8_t b = (uint8_t)litColor.z;

                            PutPixel(context.surface, context.windowWidth, context.windowHeight,
                                x, y, r, g, b, 255);

                            // Update depth buffer with inverse z
                            context.depthBuffer[depthIndex] = inv_z;
                        }
                    }
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

void RenderModelInstance(ModelInstance& instance, Camera& camera, RenderContext& context, 
                        Light light, Material material)
{
    // Compute transforms ONCE per instance
    Mat4x4 cameraMatrix = ComputeCameraMatrix(camera);
    Mat4x4 finalTransform = MultiplyMat4x4(cameraMatrix, instance.transform);
    
    // Transform light position to camera space
    Vector4 lightPosHomo = {light.position.x, light.position.y, light.position.z, 1.0f};
    Vector4 lightPosCam = MultiplyMat4x4ByVector4(cameraMatrix, lightPosHomo);
    Light lightCamSpace = light;
    lightCamSpace.position = {lightPosCam.x, lightPosCam.y, lightPosCam.z};
    
    // Camera position in camera space is always (0,0,0)
    Vector3 cameraPos = Vector3{0, 0, 0};
    
    // Transform and clip the model
    Model* clipped = TransformAndClip(camera.clipping_planes, instance.model, instance.scale, finalTransform);
    
    if (clipped == nullptr) {
        // Model was completely clipped out
        return;
    }
    
    // Project all vertices from the clipped model
    std::vector<Vector2> projected;
    std::vector<float> depths;
    for (const Vector3& vertex : clipped->vertices)
    {
        depths.push_back(vertex.z);
        // Vertex is already in camera space, just project to screen
        Vector2 screenPos = ProjectVertex(vertex, context);
        projected.push_back(screenPos);
    }
    
    // Draw triangles using projected vertices with depth values
    for (const Triangle& tri : clipped->triangles)
    {
        // Back face culling: check in camera space before projecting
        // Get the actual camera-space vertices (not screen coordinates)
        const Vector3& v0 = clipped->vertices[tri.v0];
        const Vector3& v1 = clipped->vertices[tri.v1];
        const Vector3& v2 = clipped->vertices[tri.v2];
        
        // Calculate normal using camera-space vertices
        // Normal = (v1 - v0) × (v2 - v0)
        Vector3 edge1 = v1 - v0;
        Vector3 edge2 = v2 - v0;
        Vector3 faceNormal = CrossProduct(edge1, edge2);
        faceNormal = Normalize(faceNormal);
        
        // In camera space, camera is at origin (0,0,0)
        // Calculate view vector from triangle center to camera
        Vector3 triangleCenter = (v0 + v1 + v2) * (1.0f / 3.0f);
        Vector3 viewVector = Vector3{0, 0, 0} - triangleCenter;  // From triangle to camera
        
        // Dot product: normal · viewVector
        // If > 0: normal points towards camera (front face) - keep it
        // If <= 0: normal points away from camera (back face) - cull it
        float dotProduct = faceNormal * viewVector;
        
        if (dotProduct <= 0.0f) {
            continue;  // Skip back-facing triangles
        }
        
        // For Phong shading, we use the face normal for all vertices (flat shading)
        // In a full implementation, you'd have per-vertex normals
        Vector3 N0 = faceNormal;
        Vector3 N1 = faceNormal;
        Vector3 N2 = faceNormal;
        
        // Construct Vector3 inputs: x,y are screen coordinates, z is camera-space depth
        Vector3 P0 = {projected[tri.v0].x, projected[tri.v0].y, depths[tri.v0]};
        Vector3 P1 = {projected[tri.v1].x, projected[tri.v1].y, depths[tri.v1]};
        Vector3 P2 = {projected[tri.v2].x, projected[tri.v2].y, depths[tri.v2]};
        
        DrawFilledTriangle(context, P0, P1, P2, N0, N1, N2, cameraPos, lightCamSpace, material);
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

void ClearDepthBuffer(RenderContext& context)
{
    // Initialize depth buffer to 0 (or negative infinity) since we're using inverse z
    // Larger 1/z = closer, so we want to start with the smallest value (0 or negative)
    context.depthBuffer = std::vector<float>(context.windowWidth * context.windowHeight, 0.0f);
}

