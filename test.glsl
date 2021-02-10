#define MAX_STEPS 100
#define MAX_DIST 100.
#define SURF_DIST 0.01



float sdSphere(vec3 p, vec4 s){
    return length(p - s.xyz) - s.w;
}

float sdCapsule(vec3 p, vec3 a, vec3 b, float r){
    vec3 ab = vec3(b - a);
    vec3 ap = vec3(p - a);

    float t = clamp(dot(ab, ap) / dot(ab, ab), 0.0, 1.0);
    vec3 c = a + t*ab;
    return length(p-c) - r;
}

float sdTorus(vec3 p, vec2 r){
    float x = length(p.xz) - r.x;
    return length(vec2(x, p.y)) - r.y;
}


float sdBox(vec3 p, vec3 b){
    return length(max(abs(p) - b, 0.0));
}


float getDist(vec3 p){
    vec4 s = vec4(0 ,1 ,6, 1);
    // float sd = sdSphere(p, s);
    float sd1 = sdCapsule(p, vec3(0, 1, 6), vec3(1, 2, 6), 0.2);
    float sd2 = sdTorus(p - vec3(0, 0.5, 6), vec2(1, .2));
    float sd3 = sdBox(p - vec3(-2, 0.5, 6), vec3(.5, .5, .5));
    float planeDist = p.y;
    float sd = min(sd1, planeDist);
    sd = min(sd, sd2);
    sd = min(sd, sd3);
    return sd;  
}

float rayMarch(vec3 ro, vec3 rd){
    float d0 = 0.0;

    for(int i=0; i<MAX_STEPS; i++){
        vec3 p = ro + rd * d0;
        float dS = getDist(p);
        d0 += dS;
        if(d0 > MAX_DIST || dS < SURF_DIST) break;
    }

    return d0;
}

vec3 getNormal(vec3 p){
    float d = getDist(p);
    vec2 e = vec2(0.01, 0.0);
    vec3 n = d - vec3(
        getDist(p - e.xyy),
        getDist(p - e.yxy),
        getDist(p - e.yyx)
    );
    return normalize(n);
}

float getLight(vec3 p){
    // vec3 lightPos = vec3(0, 5, 6);
    vec3 lightPos = vec3(0.  + 2.*(cos(-2.*iTime)), 5., 6. + 2.*(sin(-2.*iTime)));
    vec3 l = normalize(lightPos - p);
    vec3 n = getNormal(p);

    float dif = clamp(dot(n, l), 0.0, 1.0);
    float d = rayMarch(p + 2.*SURF_DIST*n, l);
    if(d < length(lightPos - p)) dif *= 0.1;

    return dif;
}

void main(){
    vec2 uv = (gl_FragCoord.xy - iResolution.xy*0.5) / iResolution.y;  
    vec3 col = vec3(0.0);
    // vec3 ro = vec3(0, 1, 0);
    vec3 ro = vec3(0, 2, 0);
    vec3 rd = vec3(normalize(vec3(uv.x, uv.y-.1, 1.0)));
    // vec3 rd = vec3(normalize(vec3(uv.x, uv.y, 1.0)));
    float d = rayMarch(ro, rd);
    vec3 p = ro + rd*d;
    float dif = getLight(p);
    col = vec3(dif);

    gl_FragColor = vec4(col, 1.0);
}
