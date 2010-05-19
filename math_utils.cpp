#include <math.h>
#include "math_utils.h"

double nan() {
	double f = 0.0;
	return f/f;
}
double distance_squared_d(const double a[2], const double b[2]) {
	double x = a[0]-b[0];
	double y = a[1]-b[1];
	return x*x + y*y;
}

double distance_d(const double a[2], const double b[2]) {
	return sqrt(distance_squared_d(a,b));
}

double dot_d(const double p[2], const double q[2]) {
	return p[0]*q[0] + p[1]*q[1];
}

/* Executes ray tracing for a segment. p0 and p1 are the segments extrema, eye is the position
of the eye, and direction is the direction of the ray coming out of the eye. Returns true
if the ray intersects the segment, and in that case *range contains the length of the ray. */
int segment_ray_tracing(const double p0[2], const double p1[2], const double eye[2], const double direction, double*range, double * coord) {
	
	*range = NAN;
	*coord = NAN;
	
	// p0 - p1
	double arrow[2] = {p1[0]-p0[0],p1[1]-p0[1]};
	// Normal to segment line
	double S[2] = { -arrow[1], arrow[0]};
	// Viewing direction
	double N[2] = { cos(direction), sin(direction)};
	// If S*N = 0 then they cannot cross
	double S_dot_N = dot_d(S,N);
	if( S_dot_N == 0) return 0;
	// Rho of the line in polar coordinates (multiplied by |S|)
	double line_rho = dot_d(p0,S);
	// Rho of the eye  (multiplied by |S|)
	double eye_rho = dot_d(eye,S);
	// Black magic
	double dist = (line_rho - eye_rho) / S_dot_N;
	if(dist<=0) return 0;
	
	// Now we check whether the crossing point
	// with the line lies within the segment
	
	// Crossing point
	double crossing[2] = {eye[0] + N[0]*dist, eye[1]+N[1]*dist};
	// Half of the segment
	double midpoint[2] = { 0.5*(p1[0]+p0[0]),0.5*(p1[1]+p0[1])};
	
	double seg_size = distance_d(p0, p1);
	double dist_to_midpoint = distance_d(crossing, midpoint);
	
	if(dist_to_midpoint > seg_size/2 )
		return 0;
	
	if(coord != 0) {
		*coord = distance_d(crossing, p0) / seg_size;
	}
	
	*range = dist;
	return 1;
}

double segment_alpha(const double p0[2], const double p1[2]) {
	double arrow[2] = {p1[0]-p0[0],p1[1]-p0[1]};
	// Normal to segment line
	double S[2] = { -arrow[1], arrow[0]};
	return atan2(S[1], S[0]);
}

double normalize_0_2PI(double t) {
//	if(is_nan(t)) {
//		sm_error("Passed NAN to normalize_0_2PI().\n");
//		return GSL_NAN;
//	}
	while(t<0) t+=2*M_PI;
	while(t>=2*M_PI) t-=2*M_PI;
	return t;
}


int is_nan(double v) {
	return v == v ? 0 : 1;
}

int any_nan(const double *d, int n) {
	int i; for(i=0;i<n;i++) 
		if(is_nan(d[i]))
			return 1;
	return 0;
}




void projection_on_line_d(const double a[2],
	const double b[2],
	const double p[2],
	double res[2], double *distance)
{
	double t0 = a[0]-b[0];
	double t1 = a[1]-b[1];
	double one_on_r = 1 / sqrt(t0*t0+t1*t1);
	/* normal */
	double nx = t1  * one_on_r ;
	double ny = -t0 * one_on_r ;
	double c= nx, s = ny; 
	double rho = c*a[0]+s*a[1];

	res[0] =   c*rho + s*s*p[0] - c*s*p[1] ;
	res[1] =   s*rho - c*s*p[0] + c*c*p[1] ;	
	
	if(distance)
		*distance = fabs(rho-(c*p[0]+s*p[1]));
}

void copy_d(const double*from, int n, double*to) {
	int i; for(i=0;i<n;i++) to[i] = from[i];
}


void projection_on_segment_d(
	const double a[2],
	const double b[2],
	const double x[2],
	double proj[2]) 
{
	projection_on_line_d(a,b,x,proj,0);
	if ((proj[0]-a[0])*(proj[0]-b[0]) +
	    (proj[1]-a[1])*(proj[1]-b[1]) < 0 ) {
		/* the projection is inside the segment */
	} else 
		if(distance_squared_d(a,x) < distance_squared_d(b,x)) 
			copy_d(a,2,proj);
		else
			copy_d(b,2,proj);
}

