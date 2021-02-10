#define M_PI 3.1415926535
#define M_EPSILON 0.0001

#define MAXIters 16
#define SKYColor vec3(0.85, 1.05, 1.20)
#define FOGColor vec3(0.70, 0.80, 1.00)


// ------------------ Miscellaneous ----------------------
float deg2rad(float deg)
{
    return(deg * M_PI / 180.0);
}

float infinite2Unit(float x)
{
    x = abs(x);
    return(sqrt(x / (1.0 + x)));
}

// Viridis approximation - https://www.shadertoy.com/view/XtGGzG
vec3 plasmaQuintic(float x)
{
	vec4 x1 = vec4( 1.0, x, x * x, x * x * x ); // 1 x x2 x3
	vec4 x2 = x1 * x1.w * x; // x4 x5 x6 x7
	return vec3(
		dot( x1.xyzw, vec4( +0.063861086, +1.992659096, -1.023901152, -0.490832805 ) ) + dot( x2.xy, vec2( +1.308442123, -0.914547012 ) ),
		dot( x1.xyzw, vec4( +0.049718590, -0.791144343, +2.892305078, +0.811726816 ) ) + dot( x2.xy, vec2( -4.686502417, +2.717794514 ) ),
		dot( x1.xyzw, vec4( +0.513275779, +1.580255060, -5.164414457, +4.559573646 ) ) + dot( x2.xy, vec2( -1.916810682, +0.570638854 ) ) );
}

// Narkowicz 2015, "ACES Filmic Tone Mapping Curve"
vec3 ACESFilm(vec3 x)
{
    float a = 2.51;
    float b = 0.03;
    float c = 2.43;
    float d = 0.59;
    float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

vec3 Linear2sRGB(const vec3 linear)
{
    return pow(linear, vec3(1.0 / 2.2));
}
// ------------------ Miscellaneous ----------------------


// ------------------ BLAS/LAPACK elements ----------------------
mat4 QuaternionToMatrix(vec4 q)
{
    mat4 m1 = mat4( q.w,  q.z, -q.y, q.x,
                   -q.z,  q.w,  q.x, q.y,
                    q.y, -q.x,  q.w, q.z,
                   -q.x, -q.y, -q.z, q.w);

    mat4 m2 = mat4( q.w,  q.z, -q.y, -q.x,
                   -q.z,  q.w,  q.x, -q.y,
                    q.y, -q.x,  q.w, -q.z,
                    q.x,  q.y,  q.z,  q.w);

    mat4 m = m1 * m2;

    return(m);
}

mat4 AxisAngleToMatrix(vec3 axis, float angle)
{
    float s = sin(angle / 2.0);
    float c = cos(angle / 2.0);

    vec4 q = vec4(s, s, s, c);
    q.x *= axis.x;
    q.y *= axis.y;
    q.z *= axis.z;

    mat4 m = QuaternionToMatrix(q);
    return(m);
}

// pitch (attitude) - rotation around X-axis
// yaw (heading)    - rotation around Y-axis
// roll (bank)      - rotation around Z-axis
mat4 EulerToMatrix(float pitch, float yaw, float roll)
{
    // The definition can be found in glm/gtc/quaternion.hpp
    vec3 c = vec3(cos(pitch / 2.0), cos(yaw / 2.0), cos(roll / 2.0));
    vec3 s = vec3(sin(pitch / 2.0), sin(yaw / 2.0), sin(roll / 2.0));

    vec4 q = vec4(0.0);

    // XYZ ordering
    q.x = s.x * c.y * c.z - c.x * s.y * s.z;
    q.y = c.x * s.y * c.z + s.x * c.y * s.z;
    q.z = c.x * c.y * s.z - s.x * s.y * c.z;
    q.w = c.x * c.y * c.z + s.x * s.y * s.z;

    //q = normalize(q);
    mat4 m = QuaternionToMatrix(q);
    return(m);
}
// ------------------ BLAS/LAPACK elements ----------------------


// ------------------ Ray ------------------
struct Ray
{
	vec3 Origin;
	vec3 Direction;
};

Ray rayConstruct(vec2 uv, mat4 invProj, mat4 invView)
{
    // Ray in screen space
    vec2 sXY = 2.0 * uv - 1.0;
    vec4 sP0 = vec4(sXY, -1.0, 1.0);
    vec4 sP1 = vec4(sXY,  1.0, 1.0);

    // Ray in world space
    vec4 wP0 = invProj * sP0; wP0 /= wP0.w;
    vec4 wP1 = invProj * sP1; wP1 /= wP1.w;
   
    wP0 = invView * wP0;
    wP1 = invView * wP1;
    
    Ray ray = Ray(invView[3].xyz, normalize(wP1.xyz - wP0.xyz));
    return(ray);
}
// ------------------ Ray ------------------


// ------------------ Plane ------------------
struct Plane
{
	vec3 M;
	vec3 Normal;
};

Plane planeConstruct(vec3 point, vec3 normal)
{
    Plane pl;
    pl.M = point;
    pl.Normal = normal;
   
    return(pl);
}

bool planeIntersect(Plane pl, Ray ray)
{
    float denom = dot(ray.Direction, pl.Normal);
    if(abs(denom) < M_EPSILON)
    {
        return(false);
    }

    float t = dot((pl.M - ray.Origin), pl.Normal) / denom;
    if(t < 0.0)
    {
        return(false);
    }

    return(true);
}

bool planeIntersect(Plane pl, Ray ray, out vec3 ipos, out vec3 norm)
{
    float denom = dot(ray.Direction, pl.Normal);
    if(abs(denom) < M_EPSILON)
    {
        return(false);
    }

    float t = dot((pl.M - ray.Origin), pl.Normal) / denom;
    if(t < 0.0)
    {
        return(false);
    }

    ipos = ray.Origin + t * ray.Direction;
    norm = pl.Normal;

    return(true);
}
// ------------------ Plane ------------------


// ------------------ Superellipsoid ------------------
struct Superellipsoid
{
	vec3 Center;
	vec3 Radius;
    vec2 Exponent;
    mat3 Orientation; // Orientation for finding intersection
};

Superellipsoid superellipsoidConstruct(vec3 pos, vec3 radius)
{
    //arg parameter for rotation about x, y, z axis
    //division by vec3 to slow down time 
    vec3 arg = 0.5 + 0.5 * sin(vec3(1. *iTime) / vec3(2.0, 4.0, 3.0));
    
    // 0.1 - near to square form, 2.00 - diamond form
    
    //e determines nature of shape (exponent for superquad)
    vec2 e = mix(vec2(0.1), vec2(2.0), arg.xy);
    
    vec3 axis0 = vec3(1.0, 0.0, 0.0);
    vec3 axis1 = vec3(0.0, 1.0, 1.0);
    vec3 axis2 = vec3(0.0, 0.0, 1.0);
    vec3 axis = mix(axis0, mix(axis1, axis2, max(0.0, 2.0 * arg.z - 1.0)), min(1.0, 2.0 * arg.z));
    mat4 o = AxisAngleToMatrix(normalize(axis), deg2rad(360.0 * mod(0.05 * iTime, 1.0)));
    
    Superellipsoid se;
    se.Center = pos;
    se.Radius = radius;
    se.Exponent = e;
    se.Orientation = mat3(o);
    
    // mat3 o = mat3( 1.0,  0.0,  1.0,
    //                0.0,  1.0,  1.0,
    //                0.0,  0.0,  1.0);
    // se.Orientation = mat3(o);

    return(se);
}

// Superellipsoid Inside-Outside Function
float superellipsoidIOF(vec3 pos, vec3 dir, float t, Superellipsoid se)
{
    vec3 e = vec3(vec2(1.0) / se.Exponent.xy, se.Exponent.x / se.Exponent.y);
    vec3 invr = vec3(1.0) / se.Radius;
    vec3 p = pos + t * dir;

    vec3 A = p * invr;
    vec3 B = pow(A * A, e.xxy);
    float E = B.x + B.y;
    float F = pow(E, e.z);
    float P = F + B.z;

    float K = pow(P, se.Exponent.y) - 1.0;
    return(K);
}

vec3 superellipsoidNormal(vec3 p, Superellipsoid se)
{
    vec3 e = vec3(vec2(1.0) / se.Exponent.xy, se.Exponent.x / se.Exponent.y);
    vec3 g = 2.0 * e;
    vec3 invr = vec3(1.0) / se.Radius;

    vec3 A = p * invr;
    vec3 B = pow(A * A, e.xxy);
    vec3 C = B / A;

    float E = B.x + B.y;
    float F = pow(E, e.z);
    float G = e.z * (F / E);

    vec3 n = g.xxy * C * invr;
    n.xy *= G;

    n = normalize(n);
    return(n);
}

// Ron Goldman "Curvature formulas for implicit curves and surfaces"
float superellipsoidGaussianCurvature(Superellipsoid se, vec3 pos)
{
    vec3 e = vec3(vec2(1.0) / se.Exponent.xy, se.Exponent.x / se.Exponent.y);
    vec3 invr = vec3(1.0) / se.Radius;
    
    vec3 p = (pos - se.Center) * se.Orientation;
    
    vec3 A = p * invr;
    vec3 B = pow(A * A, e.xxy);
    float E = B.x + B.y;
    float F = pow(E, e.z);
    float P = F + B.z;

    float Fx = e.z * (F / E) * (2.0 * e.x * (B.x / A.x) * invr.x);
    float Fy = e.z * (F / E) * (2.0 * e.x * (B.y / A.y) * invr.y);
    float Fz = 2.0 * e.y * (B.z / A.z) * invr.z;

    float Fxx = Fx * (e.z - 1.0) * (1.0 / E) * (2.0 * e.x * (B.x / A.x) * invr.x) + Fx * (2.0 * e.x - 1.0) * (1.0 / A.x) * invr.x;
    float Fyy = Fy * (e.z - 1.0) * (1.0 / E) * (2.0 * e.x * (B.y / A.y) * invr.y) + Fy * (2.0 * e.x - 1.0) * (1.0 / A.y) * invr.y;
    float Fzz = Fz * (2.0 * e.y - 1.0) * (1.0 / A.z) * invr.z;

    float Fyx = Fx * (e.z - 1.0) * (1.0 / E) * (2.0 * e.x * (B.y / A.y) * invr.y);
    float Fxy = Fyx;

    float Fxz = 0.0;
    float Fzx = 0.0;

    float Fyz = 0.0;
    float Fzy = 0.0;
    
    vec3 nf = vec3(Fx, Fy, Fz);
    mat3 hs = mat3(vec3(Fyy * Fzz - Fyz * Fzy, Fxz * Fzy - Fxy * Fzz, Fxy * Fyz - Fxz * Fyy),
                   vec3(Fyz * Fzx - Fyx * Fzz, Fxx * Fzz - Fxz * Fzx, Fyx * Fxz - Fxx * Fyz),
                   vec3(Fyx * Fzy - Fyy * Fzx, Fxy * Fzx - Fxx * Fzy, Fxx * Fyy - Fxy * Fyx));
    
    float D = dot(nf, nf);
    float Kg = dot(nf, hs * nf) / (D * D);

    return(Kg);   
}

bool superellipsoidIntersect(Superellipsoid se, Ray ray, float tolerance)
{
    // OBB -> AABB
    vec3 vmin = -se.Radius;
    vec3 vmax =  se.Radius;

    // Ray vs OBB -> Ray vs AABB
    mat3 invm = transpose(se.Orientation);
    vec3 pos = invm * (ray.Origin - se.Center);
    vec3 dir = invm * ray.Direction;
    
    // Hit points with AABB
    vec3 v1 = (vmin - pos) / dir;
    vec3 v2 = (vmax - pos) / dir;
    vec3 n = min(v1, v2);
    vec3 f = max(v1, v2);

    float tn = max(n.x, max(n.y, n.z));
    float tf = min(f.x, min(f.y, f.z));
    if(tf < 0.0 || tn > tf)
    {
        return(false);
    }
    
    // Iterative proceduare of finding intersection point with superellipsoid
    bool success = false;
    
    float dt = (tf - tn) / 128.0;
    float t0 = tn - dt;
    float t1 = tn;

    float S0 = superellipsoidIOF(pos, dir, t0, se);
    float S1 = superellipsoidIOF(pos, dir, t1, se);

    // secant method of root refinement
    for(int i = 0; i < MAXIters; i++)
    {
        float t = t0 - S0 * (t1 - t0) / (S1 - S0);

        t0 = t1;
        t1 = t;

        S0 = S1;
        S1 = superellipsoidIOF(pos, dir, t1, se);

        float error = abs(t1 - t0) / max(10.0 * tolerance, max(t0, t1));
        if(error < tolerance)
        {
            success = true;
            break;
        }
    }
    
    return(success);
}

bool superellipsoidIntersect(Superellipsoid se, Ray ray, float tolerance, out vec3 ipos, out vec3 norm)
{
    // OBB -> AABB
    vec3 vmin = -se.Radius;
    vec3 vmax =  se.Radius;

    // Ray vs OBB -> Ray vs AABB
    mat3 invm = transpose(se.Orientation);
    vec3 pos = invm * (ray.Origin - se.Center);
    vec3 dir = invm * ray.Direction;
    
    // Hit points with AABB
    vec3 v1 = (vmin - pos) / dir;
    vec3 v2 = (vmax - pos) / dir;
    vec3 n = min(v1, v2);
    vec3 f = max(v1, v2);

    float tn = max(n.x, max(n.y, n.z));
    float tf = min(f.x, min(f.y, f.z));
    if(tf < 0.0 || tn > tf)
    {
        return(false);
    }
    
    // Iterative proceduare of finding intersection point with superellipsoid
    bool success = false;
    
    float dt = (tf - tn) / 128.0;

    float t0 = tn - dt;
    float t1 = tn;

    float S0 = superellipsoidIOF(pos, dir, t0, se);
    float S1 = superellipsoidIOF(pos, dir, t1, se);

    // secant method of root refinement
    for(int i = 0; i < MAXIters; i++)
    {
        float t = t0 - S0 * (t1 - t0) / (S1 - S0);

        t0 = t1;
        t1 = t;

        S0 = S1;
        S1 = superellipsoidIOF(pos, dir, t1, se);

        float error = abs(t1 - t0) / max(10.0 * tolerance, max(t0, t1));
        if(error < tolerance)
        {
            success = true;
        
            vec3 lpos = pos + t1 * dir;
            norm = superellipsoidNormal(lpos, se);
            ipos = se.Orientation * lpos + se.Center;
            norm = se.Orientation * norm;
            break;
        }
    }

    return(success);
}
// ------------------ Superellipsoid ------------------


// ------------------ Camera ----------------------
struct Camera
{
    mat4 invProj;
    mat4 invView;
};

Camera cameraConstruct(float fovy, float aspect, float near, float far)
{
    fovy = deg2rad(fovy);

    Camera camera;
    camera.invView = mat4(1.0);

    float d = 1.0 / tan(0.5 * fovy);
    camera.invProj = mat4(aspect / d, 0.0,      0.0, 0.0,
                          0.0,   1.0 / d,  0.0, 0.0,
                          0.0,   0.0,      0.0, (near - far) / (2.0 * near * far),
                          0.0,   0.0,     -1.0, (near + far) / (2.0 * near * far));

    return(camera);
}

void cameraSetOrientation(inout Camera camera, float pitch, float yaw, float roll)
{
    pitch = deg2rad(pitch);
    yaw = deg2rad(yaw);
    roll = deg2rad(roll);
    
    mat4 m = camera.invView;
    camera.invView = EulerToMatrix(pitch, yaw, roll);
    camera.invView[3] = m[3];
}

void cameraSetPosition(inout Camera camera, vec3 origin)
{
    camera.invView[3] = vec4(origin, 1.0);
}

Ray cameraGetRay(Camera camera, vec2 uv)
{
    Ray ray = rayConstruct(uv, camera.invProj, camera.invView);
    return(ray);
}
// ------------------ Camera ----------------------


// ------------------ Lighting ------------------
struct Light
{
	vec3 Color;
	vec3 Position;
	vec3 Direction;
};

Light constructLight(vec3 c, vec3 o, float theta, float phi)
{
    // https://en.wikipedia.org/wiki/Spherical_coordinate_system
    float ct = cos(theta);
    float st = sin(theta);

    float cp = cos(phi);
    float sp = sin(phi);
    
    float r = 1.0;
    float x = r * st * cp;
    float y = r * st * sp;
    float z = r * ct;

    Light l;
    l.Color = c;
	l.Position = o;
	l.Direction = vec3(x, y, z);
    
    return(l);
}

// Ashikhmin Shirley 2000 (isotropic case)
vec3 calculateLighting(vec3 I, vec3 L, vec3 V, vec3 N, float Rd, float Rs, float exponent)
{
    vec3 H = normalize(L + V);
    float HdotV = dot(H, V);
    float NdotH = dot(N, H);
    float NdotV = dot(N, V);
    float NdotL = dot(N, L);

    float rho_d = 28.0 / (23.0 * M_PI) * Rd * (1.0 - pow(1.0 - NdotV / 2.0, 5.0)) * (1.0 - pow(1.0 - NdotL / 2.0, 5.0));
    rho_d *= (1.0 - Rs); // coupled diffuse

    float F = Rs + (1.0 - Rs) * pow(1.0 - HdotV, 5.0);
    float rho_s = ((exponent + 1.0) / (8.0 * M_PI)) * F * pow(max(NdotH, 0.0), exponent) / (HdotV * max(NdotV, NdotL));

    vec3 brightness = max(0.0, NdotL) * I * (rho_d + rho_s);
    return(brightness);
}

float calcShadowAttenuation(vec3 ipoint, Light dl, Superellipsoid se)
{
    Ray ray = Ray(ipoint, dl.Direction);
    bool isIntersect = superellipsoidIntersect(se, ray, 1.0e-06);
    
    return((isIntersect)? 0.25: 1.0);
}
// ------------------ Lighting ------------------


void mainImage( out vec4 fragColor, in vec2 fragCoord )
{
    vec3 P = vec3(0.0, 0.0, 0.0); // default viewer position
    float aspect = iResolution.x / iResolution.y;
    float near = 0.1;
    float far = 32.0;

    Camera camera = cameraConstruct(45.0, aspect, near, far);
    cameraSetOrientation(camera, 0.0, 0.0, 0.0);
    cameraSetPosition(camera, P);

    vec2 uv = fragCoord / iResolution.xy;
    Ray ray = cameraGetRay(camera, uv);

    // scene rendering
    vec3 pos = vec3(0.0, 1.0, 0.0);
    vec3 normal = vec3(0.0, -1.0, 0.0);
    Plane spl = planeConstruct(pos, normal); // sky plane

    pos = vec3(0.0, -1.0, 0.0);
    normal = vec3(0.0, 1.0, 0.0);
    Plane gpl = planeConstruct(pos, normal); // ground plane

    pos = vec3(0.0, 0.0, -2.5);
    vec3 radius = vec3(1.0 / 3.0, 1.0 / 3.0, 1.0 / 2.0);
    Superellipsoid se = superellipsoidConstruct(pos, radius); // object

    vec3 ipoint = ray.Direction * far;
    vec3 color = SKYColor + ray.Direction.y * 0.72;
    
    Light sun;
    sun.Color = 1.85 * vec3(1.0, 1.0, 1.0);
    sun.Direction = normalize(vec3(0.5, 0.5, 1.0));

    // ray vs superellipsoid
    normal = vec3(0.0);
    bool isIntersect = superellipsoidIntersect(se, ray, 1.0e-06, ipoint, normal);
    if(isIntersect)
    {
        // float gc = superellipsoidGaussianCurvature(se, ipoint);
        // vec3 albedo = plasmaQuintic(infinite2Unit(gc));

        // account sun lighting 
        vec3 brightness = calculateLighting(sun.Color, sun.Direction, -ray.Direction, normal, 1.0, 0.25, 128.0);

        // Ray ray2 = Ray(ipoint, reflect(ray.Direction, normal));
        // vec3 ipoint2 = vec3(0.0);
        // vec3 normal2 = vec3(0.0);

        // account sky plane lighting 
        // isIntersect = planeIntersect(spl, ray2, ipoint2, normal2);
        // if(isIntersect)
        // {
        //     brightness += calculateLighting(SKYColor, -spl.Normal, ray2.Direction, normal, 1.0, 0.25, 32.0);
        // }

        // account ground plane lighting (used fake ground colour)
        // isIntersect = planeIntersect(gpl, ray2, ipoint2, normal2);
        // if(isIntersect)
        // {
        //     float f = mod(floor(6.0 * ipoint2.z) + floor(6.0 * ipoint2.x), 2.0);
        //     vec3 GNDColor = 0.4 + f * vec3(0.6);
        //     brightness += calculateLighting(SKYColor * GNDColor, -gpl.Normal, ray2.Direction, normal, 1.0, 0.25, 32.0);
        // }

        // color = albedo * brightness;
        color = brightness;
    }
    // else
    // {
    //     //ray vs ground plane
    //     isIntersect = planeIntersect(gpl, ray, ipoint, normal);
    //     if(isIntersect)
    //     {
    //         // account sun lighting 
    //         vec3 brightness = calculateLighting(sun.Color, sun.Direction, -ray.Direction, normal, 1.0, 0.0, 32.0);

    //         // account sky plane lighting 
    //         brightness += calculateLighting(SKYColor, -spl.Normal, -ray.Direction, normal, 1.0, 0.0, 32.0);

    //         // calculate albedo of ground plane
    //         float f = mod(floor(6.0 * ipoint.z) + floor(6.0 * ipoint.x), 2.0);
    //         vec3 albedo = 0.4 + f * vec3(0.6);

    //         color = albedo * brightness;

    //         // shadow
    //         float attenuation = calcShadowAttenuation(ipoint, sun, se);
    //         color *= attenuation;
    //     }
    // }

    // Tone mapping
    // color = ACESFilm(color);
    
    // Exponential distance fog
    // float distance = length(ipoint - P);
    // color = mix(color, 0.85 * FOGColor, 1.0 - exp2(-0.0055 * distance * distance));

    // Gamma correction
    // color = Linear2sRGB(color);

    // float vignette = pow(32.0 * uv.x * uv.y * (1.0 - uv.x) * (1.0 - uv.y), 0.05);
    // fragColor = vec4(color * vignette, 1.0);
    fragColor = vec4(color , 1.0);
}
