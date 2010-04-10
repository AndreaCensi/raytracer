#include <math.h>
#include <assert.h>
#include "simplemap.h"
#include "math_utils.h"

int sign(double x) {
	return x > 0 ? 1 : (x==0 )? 0 : -1;
}

int intersect_ray_and_circle(const double center_in[2], const double radius, const double eye_position_in[2], const double eye_orientation, double * reading, double * coord) {
	*reading = NAN;
	if(coord) *coord = NAN;
	
	// First of all, translate coordinates such that the circle is at (0,0)
	double eye_position[2];
	eye_position[0] = eye_position_in[0]-center_in[0];
	eye_position[1] = eye_position_in[0]-center_in[0];
	double center[2] = {0,0};
	
	// next follows the algorithm from 
	// http://mathworld.wolfram.com/Circle-LineIntersection.html	
	
	double x_1 = eye_position[0];
	double y_1 = eye_position[1];
	double x_2 = eye_position[0] + cos(eye_orientation);
	double y_2 = eye_position[1] + sin(eye_orientation);
	double D = x_1 * y_2 - x_2 * y_1;
	double d_x = x_2 - x_1;
	double d_y = y_2 - y_1;
	double d_r = sqrt( d_x * d_x + d_y * d_y);
	
	double delta = radius* radius * d_r * d_r - D * D;
	if (delta < 0)
		return 0;
	
	
	double p1[2],p2[2];
	
	p1[0] = (D * d_y + sign(d_y) * d_x * sqrt(delta)) / (d_r*d_r);
	p2[0] = (D * d_y - sign(d_y) * d_x * sqrt(delta)) / (d_r*d_r);
	p1[1] = (-D * d_x + fabs(d_y) * sqrt(delta)) / (d_r*d_r);
	p2[1] = (-D * d_x - fabs(d_y) * sqrt(delta)) / (d_r*d_r);

	// check they are on the circle
	double tolerance = 1e-5;
	assert( fabs(distance_d(p1, center) - radius) < tolerance );
	assert( fabs(distance_d(p2, center) - radius) < tolerance );
	
	// check that they are on the line as well
	double direction[2] = {cos(eye_orientation), sin(eye_orientation)};
	double normal[2] = {- direction[1], direction[0] };
	#define DOT(a,b) (a[0]*b[0]+a[1]*b[1])
	assert( fabs( DOT(eye_position, normal) - DOT(p1, normal) ) < tolerance );
	assert( fabs( DOT(eye_position, normal) - DOT(p2, normal) ) < tolerance );
	
	// let's choose between the two
	double d1 = distance_d(p1, eye_position);
	double d2 = distance_d(p2, eye_position);
	// let's see if they are in front
	int ok1 = DOT(direction,p1) > DOT(direction, eye_position);
	int ok2 = DOT(direction,p2) > DOT(direction, eye_position);
	
	double * p = (!ok1 & !ok2) ? 0 : (ok1 & !ok2) ? p1 : (ok2 & !ok1) ? p2 : (d1<d2) ? p1 : p2;

	if(!p)
		return 0;
	
	*reading = distance_d(p, eye_position);
	*coord = atan2( p[1], p[0] );
	return 1;
}

namespace RayTracer {
	using namespace std;
	
	bool Circle::ray_tracing(const double p[2], const double direction,  double& out_distance, double &out_alpha, int&region, double&out_coord) const {
		double distance, angle; 
		if(intersect_ray_and_circle(this->center, this->radius, p, direction, &distance, &angle)) {
			out_distance = distance;
			out_coord = angle * this->radius;
			out_alpha = angle;
			region = 0;
			return true;
		} else {
			out_distance = NAN;
			out_coord = NAN;
			region = -1;
			return false;
		}
	}
	

	bool Segment::ray_tracing(const double p[2], const double direction, 
	 	double& range, double &alpha, int&region, double&coord) const {

		double seg_coord;
		int found = segment_ray_tracing(this->p0, this->p1, p, direction, &range, &seg_coord);

		if(found) {
			alpha = segment_alpha(this->p0, this->p1);

			if( cos(alpha-direction) < 1 )
				alpha = alpha + M_PI;

			// XXX change this
			alpha = normalize_0_2PI(alpha);
			region = this->region_id;
			coord = (1-seg_coord)*this->t0 + seg_coord*this->t1;
			return true;
		} else {
			alpha = NAN;
			return false;
		}
	};

	double  Segment::getSegmentLength() {
		return distance_d(p0,p1);
	}
	
	bool Environment::ray_tracing(const double p[2], const double direction,  double& out_distance, double &out_alpha, int& surface, int&region, double&coord) const {
	
		int champion = -1;
		double champion_range, champion_alpha;
		int champion_region; double champion_coord;
		
		for(size_t i=0;i<stuff.size();i++) {
			Stuff * s = stuff.at(i);
			
			double range, alpha;
			int region; double coord;
			if(s->ray_tracing(p,direction,range,alpha, region, coord)){
				if(range<champion_range || champion==-1) {
					champion = i;
					champion_range = range;
					champion_alpha = alpha;
					champion_region = region;
					champion_coord = coord;
				}
			}
			
		}
		
		if(champion != -1) {
			out_distance = champion_range;
			out_alpha = champion_alpha;
			region = champion_region;
			coord = champion_coord;
			surface = stuff[champion]->surface_id;
			return true;
		} else {
			return false;
		}
	}
	
} // namespace SimpleMap
