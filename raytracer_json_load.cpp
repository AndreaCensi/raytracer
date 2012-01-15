#include "json-c/json.h"
#include <math.h>
#include "simplemap.h"
#include "math_utils.h"
#include "macros.h"

using namespace RayTracer;
using namespace std;

int add_polyline(Environment&env, JO jo) {
	int surface;
	expect(jo_read_int(jo, "surface", &surface));
	/* TODO: remove surface if already present. */
	env.remove(surface);
	
	JO points = jo_get(jo, "points");
	expect(jo_expect_array_size_min(points, 2));
	
	double current_coord = 0;
	for(int p=0;p<jo_array_length(points)-1;p++) {
		Segment * s = new Segment();
		s->surface_id = surface;
		s->region_id = 0;
		expect(jo_read_from_double_array (jo_array_get(points, p  ), s->p0, 2, NAN));
		expect(jo_read_from_double_array (jo_array_get(points, p+1), s->p1, 2, NAN));
		double seg_size = s->getSegmentLength();
		s->t0 = current_coord;
		s->t1 = current_coord + seg_size;
		current_coord += seg_size;
		env.stuff.push_back(s);
	}
	return 0;
}


int add_circle(Environment&env, JO jo) {
	int surface;
	expect(jo_read_int(jo, "surface", &surface));
	env.remove(surface);
	
	double position[2]; double radius; int solid_inside;
	expect(jo_read_int(jo, "solid_inside", &solid_inside));
	expect(jo_read_double(jo, "radius", &radius));
	expect(jo_read_double_array(jo, "center", position, 2, NAN));
	Circle *c = new Circle();
	c->surface_id = surface;
	c->center[0] = position[0];
	c->center[1] = position[1];
	c->radius = radius;
	c->solid_inside = solid_inside;
	env.stuff.push_back(c);

	//fprintf(stderr, "Added surface %d center %f %f; total is %d.\n",
	//		surface, position[0],position[1], env.stuff.size() );
	return 0;
}

/** Returns 0 for success */
int load_env_from_json(Environment& env, JO jo_map) {
	
	jo_expect_object(jo_map);
	
	JO jo_objects = json_object_object_get(jo_map, "objects");
		expect(jo_expect_array(jo_objects));
	
	for(int i=0; i < jo_array_length(jo_objects); i++) {
		JO jo = jo_array_get(jo_objects, i);
			expect(jo_expect_object(jo));
		
		JO jo_type = jo_get(jo, "class"); 
			expect(jo_expect_string(jo_type));
		string class_name = jo_get_string(jo_type);
			
		if(class_name == "polyline") {
			add_polyline(env, jo);
			
		} else if( class_name == "circle" ) {
			add_circle(env, jo);
		} else {
			sm_error("unknown object type: '%s'\n", jo_get_string(jo_type));
			return -1;
		}
	}
	
	return 0;
}
