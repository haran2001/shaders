#define MAX_STEPS 100
#define MAX_DIST 100.
#define SURF_DIST .01

float sdSphere(vec3 p, vec4 s){
    float t = .25 * (sin(2.*iTime));
    // s += t;
    return length(p - s.xyz) - s.w;
}

float sdCapsule(vec3 p, vec3 a, vec3 b, float r){
    vec3 ab = b - a; 
    vec3 ap = p - a;

    float t = dot(ap, ab) / dot(ab, ab);
    t = clamp(t, 0.0, 1.0);
    vec3 c = a + t*ab;

    return length(p - c) - r;
}


float GetDist(vec3 p) {
    float sdf =  sdSphere(p, vec4(1. + 2.*sin(2. * iTime), 1.+abs(sin(5.*iTime)), 6. + 2.*cos(2. * iTime), 1));
    // float sdf =  sdBox(p, vec3(1.0, 1.0, 1.0));
    // float sdf =  sdVerticalCapsule(p, 1., 1.);
    // float sdf =  sdCapsule(p, vec3(-1, 1, 6), vec3(0, 2, 6), 0.2);
    // float sdf =  sdCapsule(p, vec3(-sin(iTime), 1, 6. + cos(iTime)), vec3(sin(iTime), 2, 6. - cos(iTime)), 0.2);
    // float sdf =  sdCapsule(p, vec3(-sin(iTime), 0.5*abs(cos(iTime)), 6), vec3(sin(iTime), 2.*abs(cos(iTime)), 6), 0.2);
    float planeDist = p.y;
    float d = min(sdf, planeDist);
    return d;
}


float RayMarch(vec3 ro, vec3 rd) {
	float dO=0.;
    
    for(int i=0; i<MAX_STEPS; i++) {
    	vec3 p = ro + rd*dO;
        float dS = GetDist(p);
        dO += dS;
        if(dO>MAX_DIST || dS<SURF_DIST) break;
    }
    
    return dO;
}

vec3 GetNormal(vec3 p) {
	float d = GetDist(p);
    vec2 e = vec2(.01, 0);
    
    vec3 n = d - vec3(
        GetDist(p-e.xyy),
        GetDist(p-e.yxy),
        GetDist(p-e.yyx));
    
    return normalize(n);
}

float GetLight(vec3 p) {
    vec3 lightPos = vec3(0, 5, 4);
    // lightPos.xz += vec2(sin(iTime), cos(iTime))*2.;
    // lightPos.xz += vec2(1, 0.0*cos(1.57))*2.;
    vec3 l = normalize(lightPos-p);
    vec3 n = GetNormal(p);
    
    float dif = clamp(dot(n, l), 0., 1.);
    float d = RayMarch(p+n*SURF_DIST*2., l);
    if(d<length(lightPos-p)) dif *= .1;
    
    return dif;
}

void mainImage( out vec4 fragColor, in vec2 fragCoord )
{
    vec2 uv = (fragCoord-.5*iResolution.xy)/iResolution.y;

    vec3 col = vec3(0);
    
    vec3 ro = vec3(0, 1, 0);
    vec3 rd = normalize(vec3(uv.x, uv.y, 1));

    float d = RayMarch(ro, rd);
    
    vec3 p = ro + rd * d;
    
    float dif = GetLight(p);
    col = vec3(dif);
    
    col = pow(col, vec3(.4545));	// gamma correction
    
    fragColor = vec4(col,1.0);
}