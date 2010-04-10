#include "simplemap.h"
#include <json-c/json.h>
#include <csm/csm_all.h>
using namespace CSM;
using namespace RayTracer;

bool load_env_from_json(Environment& env, JO jo);

#define jo_expect_array(a) (a!=0 && json_object_is_type(a, json_type_array))
#define jo_expect_object(a) (a!=0 &&  json_object_is_type(a, json_type_object))
#define jo_expect_string(a) (a!=0 && json_object_is_type(a, json_type_string))
#define jo_expect_double(a) (a!=0 && json_object_is_type(a, json_type_double))
#define jo_expect_array_size(a,n) ( (a!=0) && (json_object_is_type(a, json_type_array)&& (jo_array_length(a)==n)))
#define jo_expect_array_size_min(a,n) ( (a!=0) && (json_object_is_type(a, json_type_array)&& (jo_array_length(a)>=n)))


#define expect(a) if(!a) { \
		sm_error("Invalid format: \n\t %s \n", #a); \
			return -3; \
		}

#define expect_s(a, s) if(!a) { \
		sm_error("Invalid format: %s \n\t %s \n", s, #a); \
			return -2; \
		}
		
using namespace std;

JO query_environment(Environment&env, double position[2], double orientation, vector<double>& directions);

int main(int argc, const char** argv)
{
	Environment env;
	vector<double> directions;
	
	FILE * file_input = stdin;
	
	while(1) { 
		JO jo = json_read_stream(file_input); 
		if(!jo) {
			if(feof(file_input)) break;
			sm_error("Could not read JSON.\n");
			return -3;
		}
		JO jo_class = json_object_object_get(jo, "class");
			expect(jo_expect_string(jo_class));
		string class_name = jo_get_string(jo_class);
		
		if(class_name == "map") {
			if(!load_env_from_json(env, jo)) {
				sm_error("Could not properly load map.\n");
				return -4;
			}
		} else if(class_name == "sensor") {
			JO jo_directions = json_object_object_get(jo, "directions");
				expect(jo_expect_array(jo_directions));
				
			int num_directions = jo_array_length(jo_directions);
			double directions_d[num_directions];
			expect(jo_read_from_double_array(jo_directions, directions_d, num_directions, NAN));
			directions = vector<double>(directions_d, directions_d + num_directions);
			
		} else if(class_name == "query") {
			double position[2]; double orientation;
			expect(jo_read_double(jo, "orientation", &orientation));
			expect(jo_read_double_array(jo, "position", position, 2, NAN));
			
			if(is_nan(orientation) || any_nan(position, 2)) {
				sm_error("Invalid pose (%f,%f,%f)\n", position[0],position[1],orientation);
				return -5;
			}
			
			JO response = query_environment(env, position, orientation, directions);
			std::cout << json_object_to_json_string(response) << std::endl;
			std::cout.flush();
			jo_free(response);

		} else {
			sm_error("Uknown class %s\n", class_name.c_str());
			return -3;
		}
		
		jo_free(jo);
	}
	
	return 0;
	
}

JO query_environment(Environment&env, double position[2], double orientation, vector<double>& directions) {
	int n = directions.size();
	double readings[n];
	double normal[n];
	double texture_u[n];
	int valid[n];
	int surface[n];
	int region[n];
	
	for(size_t i=0;i<directions.size();i++) {
		double rho, alpha;
		int surface_i, region_i; double texture;
		if(env.ray_tracing(position, orientation + directions[i], rho, alpha, surface_i, region_i, texture)) {
			valid[i] = 1;
			readings[i] = rho;
			normal[i] = normalize_0_2PI(alpha-orientation);// XXX change this
			surface[i] = surface_i;
			region[i] = region_i;
			texture_u[i] = texture;
		} else {
			valid[i] = 0;
			readings[i] = NAN;
			normal[i] = NAN;
			surface[i] = -1;
			region[i] = -1;
			texture_u[i] = NAN;
		}
	}

	JO response = json_object_new_object();
	jo_add_double_array(response, "readings", readings, n);
	jo_add_double_array(response, "normal",  normal, n);
	jo_add_int_array(response, "valid",  valid, n);
	jo_add_int_array(response, "surface",  surface, n);
	jo_add_int_array(response, "region",  region, n);
	jo_add_double_array(response, "curvilinear_coordinate",  texture_u, n);
	return response;
}
 

bool load_env_from_json(Environment& env, JO jo_map) {
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
			
			int surface;
			expect(jo_read_int(jo, "surface", &surface));
			
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
		} else if( class_name == "circle" ) {
			double position[2]; double radius;
			expect(jo_read_double(jo, "radius", &radius));
			expect(jo_read_double_array(jo, "center", position, 2, NAN));
			int surface;
			expect(jo_read_int(jo, "surface", &surface));
			Circle *c = new Circle();
			c->surface_id = surface;
			c->center[0] = position[0];
			c->center[1] = position[1];
			c->radius = radius;
			env.stuff.push_back(c);
		} else {
			sm_error("unknown object type: '%s'\n", jo_get_string(jo_type));
			return false;
		}
	}
	
	return true;
}

