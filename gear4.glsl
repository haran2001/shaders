// credits inigo quilez

#define AA 2

float sdSphere( in vec3 p, in float r )
{
    return length(p)-r; 
}

float sdBox( in vec2 p, in vec2 r )
{
    return length( max(abs(p)-r,0.0) );
}

float smax( float a, float b, float k )
{
    float h = max(k-abs(a-b),0.0);
    return max(a, b) + h*h*0.25/k;
}

float sdCross( in vec3 p, in vec3 b ) 
{
    vec3 q = abs(p);
    q.xz = (q.z>q.x) ? q.zx : q.xz;
    return length(max(q-b,0.0));
}


vec4 gear( in vec3 p, in float time )
{
    {
    float an = time*sign(p.y);
    float co = cos(an), si = sin(an);
    p.xz = mat2(co,-si,si,co)*p.xz;
    }
    
    p.y=abs(p.y);

    float d = 0.0;
    {
    const float an = 6.283185/12.0;
    float sector = round(atan(p.z,p.x)/an);
    float angrot = sector*an;
    vec3 q = p;

    q.xz = mat2(cos(angrot),-sin(angrot),
                sin(angrot), cos(angrot))*q.xz;    
    
    d = sdBox( q.xz - vec2(0.17,0.0), vec2(0.04,0.02) ) - 0.005;
    }
    
    // ring
    float d2 = abs(length(p.xz) - 0.155) - 0.018;
    d = min(d,d2);

    // slice
    float r = length(p);
    d = smax( d, abs(r-0.5)-0.03, 0.005 );
    
    d2 = sdCross( p+vec3(0.0,-0.48,0.0), vec3(0.15,0.003,0.003) )-0.003;
    d = min(d,d2);
    
    // pivot
    d2 = length(vec3(p.x,max(p.y-0.5,0.0),p.z)) - 0.01;        
    d = min(d,d2);
    
	return vec4( d, p );
}
vec4 map( in vec3 p, float time )
{
	vec4 d1 = gear(p.xyz,time);
    vec4 d2 = gear(p.yzx,time); d1 = (d1.x<d2.x) ? d1 : d2;
         d2 = gear(p.zxy,time); d1 = (d1.x<d2.x) ? d1 : d2;

    float f = sdSphere(p,0.1);
    d1 = (f<d1.x) ? vec4(f,p) : d1;
    
    return d1;
}

#define ZERO min(iFrame,0)

vec3 calcNormal( in vec3 pos, in float time )
{
    vec3 n = vec3(0.0);
    for( int i=ZERO; i<4; i++ )
    {
        vec3 e = 0.5773*(2.0*vec3((((i+3)>>1)&1),((i>>1)&1),(i&1))-1.0);
        n += e*map(pos+0.0005*e,time).x;
    }
    return normalize(n);
}

float calcAO( in vec3 pos, in vec3 nor, in float time )
{
	float occ = 0.0;
    float sca = 1.0;
    for( int i=ZERO; i<5; i++ )
    {
        float h = 0.01 + 0.12*float(i)/4.0;
        float d = map( pos+h*nor, time ).x;
        occ += (h-d)*sca;
        sca *= 0.95;
    }
    return clamp( 1.0 - 3.0*occ, 0.0, 1.0 );
}

float calcSoftshadow( in vec3 ro, in vec3 rd, in float k, in float time )
{
    float res = 1.0;
    
    float tmax = 2.0;
    float t    = 0.001;
    for( int i=0; i<64; i++ )
    {
        float h = map( ro + rd*t, time ).x;
        res = min( res, k*h/t );
        t += clamp( h, 0.012, 0.2 );
        if( res<0.001 || t>tmax ) break;
    }
    
    return clamp( res, 0.0, 1.0 );
}

vec4 intersect( in vec3 ro, in vec3 rd, in float time )
{
    vec4 res = vec4(-1.0);
    
    float t = 0.001;
    float tmax = 5.0;
    for( int i=0; i<128 && t<tmax; i++ )
    {
        vec4 h = map(ro+t*rd,time);
        if( h.x<0.001 ) { res=vec4(t,h.yzw); break; }
        t += h.x;
    }
    
    return res;
}

mat3 setCamera( in vec3 ro, in vec3 ta, float cr )
{
	vec3 cw = normalize(ta-ro);
	vec3 cp = vec3(sin(cr), cos(cr),0.0);
	vec3 cu = normalize( cross(cw,cp) );
	vec3 cv =          ( cross(cu,cw) );
    return mat3( cu, cv, cw );
}

void mainImage( out vec4 fragColor, in vec2 fragCoord )
{
    vec3 tot = vec3(0.0);
    
    #if AA>1
    for( int m=ZERO; m<AA; m++ )
    for( int n=ZERO; n<AA; n++ )
    {
        // pixel coordinates
        vec2 o = vec2(float(m),float(n)) / float(AA) - 0.5;
        vec2 p = (2.0*(fragCoord+o)-iResolution.xy)/iResolution.y;
        float d = 0.5*sin(fragCoord.x*147.0)*sin(fragCoord.y*131.0);
        float time = iTime;
        #else    
        vec2 p = (2.0*fragCoord-iResolution.xy)/iResolution.y;
        float time = iTime;
        #endif

	    // camera	
        float an = 6.2831*time/40.0;
        vec3 ta = vec3( 0.0, 0.0, 0.0 );
        vec3 ro = ta + vec3( 1.2*cos(an), 0.5, 1.2*sin(an) );
        
        // camera-to-world transformation
        mat3 ca = setCamera( ro, ta, 0.0 );
        
        // ray direction
        float fl = 2.0;
        vec3 rd = ca * normalize( vec3(p,fl) );

        // background
        vec3 col = vec3(1.0+rd.y)*0.03;
        
        // raymarch geometry
        vec4 tuvw = intersect( ro, rd, time );
        if( tuvw.x>0.0 )
        {
            // shading/lighting	
            vec3 pos = ro + tuvw.x*rd;
            vec3 nor = calcNormal(pos, time);
                        
            col = 0.5 + 0.5*nor;
        }
        
        
        // gamma        
	    tot += pow(col,vec3(0.45) );
    #if AA>1
    }
    tot /= float(AA*AA);
    #endif

    // cheap dithering
    tot += sin(fragCoord.x*114.0)*sin(fragCoord.y*211.1)/512.0;

    fragColor = vec4( tot, 1.0 );
}