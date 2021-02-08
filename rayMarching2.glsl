// #iChannel1 'file://duck.png'

// ray marching
const int max_iterations = 512;
const float stop_threshold = 0.001;
const float grad_step = 0.02;
const float clip_far = 1000.0;

// math
const float PI = 3.14159265359;
const float DEG_TO_RAD = PI / 180.0;

// iq's distance function
float sdSphere( vec3 pos, float r ) {
	return length( pos ) - r;
}

float sdBox( vec3 p, vec3 b ) {
  vec3 d = abs(p) - b;
  return min(max(d.x,max(d.y,d.z)),0.0) + length(max(d,0.0));
}


float sdUnion( float d0, float d1 ) {
    return min( d0, d1 );
}

float sdInter( float d0, float d1 ) {
    return max( d0, d1 );
}

float sdSub( float d0, float d1 ) {
    return max( d0, -d1 );
}

float sdUnion_s( float a, float b, float k ) {
    float h = clamp( 0.5+0.5*(b-a)/k, 0.0, 1.0 );
    return mix( b, a, h ) - k*h*(1.0-h);
}

float sfDisp( vec3 p ) {
    return sin(p.x)*sin(p.y)*sin(p.z) ;
}

vec3 sdTwist( vec3 p, float a ) {
    float c = cos(a*p.y);
    float s = sin(a*p.y);
    mat2  m = mat2(c,-s,s,c);
    return vec3(m*p.xz,p.y);
}

vec3 sdRep( vec3 p, vec3 c ) {
    return mod(p,c)-0.5*c;
}

// get distance in the world
float dist_field( vec3 p ) {
//  p = sdRep( p, vec3( 4.0 ) );
//  p = sdTwist( p, 3.0 );
    
    float d0 = sdBox( p, vec3(0.5) );
    float d1 = sdSphere( p, 0.6 );
    
    float d = sdInter( d1, d0 );

    return d;
    //return d + sfDisp( p * 2.5 );
    //return sdUnion_s( d + sfDisp( p * 2.5 * sin( iTime * 1.01 ) ), d1, 0.1 );
}

// get gradient in the world
vec3 gradient( vec3 pos ) {
	const vec3 dx = vec3( grad_step, 0.0, 0.0 );
	const vec3 dy = vec3( 0.0, grad_step, 0.0 );
	const vec3 dz = vec3( 0.0, 0.0, grad_step );
	return normalize (
		vec3(
			dist_field( pos + dx ) - dist_field( pos - dx ),
			dist_field( pos + dy ) - dist_field( pos - dy ),
			dist_field( pos + dz ) - dist_field( pos - dz )			
		)
	);
}

vec3 fresnel( vec3 F0, vec3 h, vec3 l ) {
	return F0 + ( 1.0 - F0 ) * pow( clamp( 1.0 - dot( h, l ), 0.0, 1.0 ), 5.0 );
}

// phong shading
vec3 shading( vec3 v, vec3 n, vec3 dir, vec3 eye ) {
	// ...add lights here...
	
	float shininess = 16.0;
	
	vec3 final = vec3( 0.0 );
	
	vec3 ref = reflect( dir, n );
    
    vec3 Ks = vec3( 0.5 );
    vec3 Kd = vec3( 1.0 );
	
	// light 0
	{
		vec3 light_pos   = vec3( 20.0, 20.0, 20.0 );
		vec3 light_color = vec3( 1.0, 0.7, 0.7 );
	
		vec3 vl = normalize( light_pos - v );
	
		vec3 diffuse  = Kd * vec3( max( 0.0, dot( vl, n ) ) );
		vec3 specular = vec3( max( 0.0, dot( vl, ref ) ) );
		
        vec3 F = fresnel( Ks, normalize( vl - dir ), vl );
		specular = pow( specular, vec3( shininess ) );
		
		final += light_color * mix( diffuse, specular, F ); 
	}
	
	// light 1
	{
		vec3 light_pos   = vec3( -20.0, -20.0, -30.0 );
		vec3 light_color = vec3( 0.5, 0.7, 1.0 );
	
		vec3 vl = normalize( light_pos - v );
	
		vec3 diffuse  = Kd * vec3( max( 0.0, dot( vl, n ) ) );
		vec3 specular = vec3( max( 0.0, dot( vl, ref ) ) );
        
        vec3 F = fresnel( Ks, normalize( vl - dir ), vl );
		specular = pow( specular, vec3( shininess ) );
		
		final += light_color * mix( diffuse, specular, F );
	}

    // final += texture( iChannel1, ref ).rgb * fresnel( Ks, n, -dir );
    
	return final;
}


bool ray_vs_aabb(vec3 o, vec3 dir, vec3 bmin, vec3 bmax, inout vec2 e ) {
    vec3 a = ( bmin - o ) / dir;
    vec3 b = ( bmax - o ) / dir;
    
    vec3 s = min( a, b );
    vec3 t = max( a, b );
    
    e.x = max( max( s.x, s.y ), max( s.z, e.x ) );
    e.y = max( min( t.x, t.y ), max( t.z, e.y ) );
    
    return e.x < e.y;
}

// ray marching
bool ray_marching( vec3 o, vec3 dir, inout float depth, inout vec3 n ) {
	float t = 0.0;
    float d = 10000.0;
    float dt = 0.0;
    for ( int i = 0; i < 128; i++ ) {
        vec3 v = o + dir * t;
        d = dist_field( v );
        if ( d < 0.001 ) {
            break;
        }
        dt = min( abs(d), 0.1 );
        t += dt;
        if ( t > depth ) {
            break;
        }
    }
    
    if ( d >= 0.001 ) {
        return false;
    }
    
    t -= dt;
    for ( int i = 0; i < 4; i++ ) {
        dt *= 0.5;
        
        vec3 v = o + dir * ( t + dt );
        if ( dist_field( v ) >= 0.001 ) {
            t += dt;
        }
    }
    
    depth = t;
    n = normalize( gradient( o + dir * t ) );
    return true;
    
    return true;
}

// get ray direction
vec3 ray_dir( float fov, vec2 size, vec2 pos ) {
	vec2 xy = pos - size * 0.5;

	float cot_half_fov = tan( ( 90.0 - fov * 0.5 ) * DEG_TO_RAD );	
	float z = size.y * 0.5 * cot_half_fov;
	
	return normalize( vec3( xy, -z ) );
}

// camera rotation : pitch, yaw
mat3 rotationXY( vec2 angle ) {
	vec2 c = cos( angle );
	vec2 s = sin( angle );
	
	return mat3(
		c.y      ,  0.0, -s.y,
		s.y * s.x,  c.x,  c.y * s.x,
		s.y * c.x, -s.x,  c.y * c.x
	);
}

void mainImage( out vec4 fragColor, in vec2 fragCoord )
{
	// default ray dir
	vec3 dir = ray_dir( 45.0, iResolution.xy, fragCoord.xy );
	
	// default ray origin
	vec3 eye = vec3( 0.0, 0.0, 3.5 );

	// rotate camera
	mat3 rot = rotationXY( ( iMouse.xy - iResolution.xy * 0.5 ).yx * vec2( 0.01, -0.01 ) );
	dir = rot * dir;
	eye = rot * eye;
	
	// ray marching
    float depth = clip_far;
    vec3 n = vec3( 0.0 );
	if ( !ray_marching( eye, dir, depth, n ) ) {
		// fragColor = texture( iChannel1, dir );
        return;
	}
	
	// shading
	vec3 pos = eye + dir * depth;
    
    vec3 color = shading( pos, n, dir, eye );
    color = color * color;
	fragColor = vec4( pow( color, vec3(1.0/1.2) ), 1.0 );
}