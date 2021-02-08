#define MAX_STEPS 100
#define MAX_DIST 100.
#define SURF_DIST .01

float GetDist(vec3 p){
    vec4 s = vec4(0, 1, 6, 1);
    float sphereDist = length(p - s.xyz) - s.w;
    float planeDist = p.y;

    float d = min(sphereDist, planeDist);
    return d;
}

float RayMarch(vec3 ro, vec3 rd){
    float d0= 0.;

    for(int i=0; i<MAX_STEPS; i++){
        vec3 p = ro + rd * d0;
        float dS = GetDist(p);
        d0 += dS;
        if(d0 > MAX_DIST || dS < SURF_DIST) break;
    }

    return d0;
}

vec3 GetNormal(vec3 p){
    float d = GetDist(p);
    vec2 e = vec2(0.01, 0);
    
    vec3 n = d - vec3(
        GetDist(p - e.xyy),
        GetDist(p - e.yxy),
        GetDist(p - e.yyx)
    );

    return normalize(n);
}

float GetLight(vec3 p){
    vec3 lightPos = vec3(sin(iTime), 5. + cos(iTime), 6);
    vec3 l = normalize(lightPos-p);
    vec3 n = GetNormal(p);
    
    float dif = clamp(dot(n, l), 0., 1.);
    float d = RayMarch(p+n*SURF_DIST*2., l);
    if(d<length(lightPos-p)) dif *= .1;
    return dif; 
}

void main(){
    vec2 uv = (gl_FragCoord.xy - 0.5*iResolution.xy) / iResolution.y;

    vec3 col = vec3(0.);

    vec3 ro = vec3(0.0, 1.0, 0.0);
    vec3 rd = normalize(vec3(uv.x, uv.y, 1));
    float d = RayMarch(ro, rd);
    vec3 p = ro + rd * d;
    float diff = GetLight(p);
    col = vec3(diff);
    gl_FragColor = vec4(col, 1.0); 

}