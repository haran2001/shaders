#define AA 2

float sdSphere(in vec3 p, in float r){
    return length(p)-r;
}

vec4 map(in vec3 p, float time){
    float d = sdSphere(p, 0.2);
    return vec4(d, p);
}

#define ZERO min(iFrame, 0)

vec3 calcNormal(in vec3 pos, in float time){
    vec3 n = vec3(0.0);
    for(int i=ZERO; i<4; i++){
        
    }
}