#ifndef H_MATH_UTILS
#define H_MATH_UTILS

/* Executes ray tracing for a segment. p0 and p1 are the segments extrema, eye is the position
of the eye, and direction is the direction of the ray coming out of the eye. Returns true
if the ray intersects the segment, and in that case *range contains the length of the ray.
coord is between 0, 1 (0: hit p0, 1: hit p1)
 */
int segment_ray_tracing(const double p0[2], const double p1[2], const double eye[2], const double direction, double*range, double*coord);

/** Returns the orientation of the normal for the line passing through p0-p1 */
double segment_alpha(const double p0[2], const double p1[2]);

/** Normalizes an angle in the 0-2PI range */
double normalize_0_2PI(double angle);

double distance_d(const double a[2], const double b[2]);


/** Returns true v is NAN */
int is_nan(double v);

/** Returns true if any value in d is NAN */
int any_nan(const double *d, int n);

/** Projection of P on the SEGMENT A-B */
void projection_on_segment_d(
	const double a[2],
	const double b[2],
	const double P[2],
   double proj[2]);

#endif

