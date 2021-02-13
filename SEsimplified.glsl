#define M_PI 3.1415926535
#define MAXIters 16


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
    vec3 arg = 0.5 + 0.5 * sin(vec3(0. *iTime) / vec3(2.0, 4.0, 3.0));
    
    // 0.1 - near to square form, 2.00 - diamond form
    
    //e determines nature of shape (exponent for superquad)
    // vec2 e = mix(vec2(0.1), vec2(2.0), arg.xy);
    vec2 e = vec2(0.1);
    
    vec3 axis0 = vec3(1.0, 0.0, 0.0);
    vec3 axis1 = vec3(-1.0, 1.0, 1.0);
    vec3 axis2 = vec3(0.0, 0.0, 1.0);
    // vec3 axis = mix(axis0, mix(axis1, axis2, max(0.0, 2.0 * arg.z - 1.0)), min(1.0, 2.0 * arg.z));
    vec3 axis = axis1;
    // mat4 o = AxisAngleToMatrix(normalize(axis), deg2rad(360.0 * mod(0.0 * iTime, 1.0)));
    mat4 o = AxisAngleToMatrix(normalize(axis), deg2rad(180.0));
    
    Superellipsoid se;
    se.Center = pos;
    se.Radius = radius;
    se.Exponent = e;
    se.Orientation = mat3(o);

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


// ------------------ Lighting ------------------


// void mainImage( out vec4 fragColor, in vec2 fragCoord )
void main()
{
    vec3 P = vec3(0.0, 0.0, 0.0); // default viewer position
    float aspect = iResolution.x / iResolution.y;
    float near = 0.1;
    float far = 32.0;

    Camera camera = cameraConstruct(45.0, aspect, near, far);

    vec2 uv = gl_FragCoord.xy / iResolution.xy;
    Ray ray = cameraGetRay(camera, uv);


    vec3 pos = vec3(0.0, 0.0, -2.5);
    vec3 radius = vec3(1.0 / 3.0, 1.0 / 3.0, 1.0 / 2.0);
    Superellipsoid se = superellipsoidConstruct(pos, radius); // object

    vec3 ipoint = ray.Direction * far;
    vec3 color = vec3(1.0);
    
    Light sun;
    sun.Color = 1.85 * vec3(1.0, 1.0, 1.0);
    sun.Direction = normalize(vec3(0.5, 0.5, 1.0));

    // ray vs superellipsoid
    vec3 normal = vec3(0.0);
    bool isIntersect = superellipsoidIntersect(se, ray, 1.0e-06, ipoint, normal);
    if(isIntersect)
    {
        vec3 brightness = calculateLighting(sun.Color, sun.Direction, -ray.Direction, normal, 1.0, 0.25, 128.0);
        color = brightness;
    }
    // color *= abs(sin(iTime));
    gl_FragColor = vec4(color , 1.0);
}
