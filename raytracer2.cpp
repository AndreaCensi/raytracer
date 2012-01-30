#include "json-c/json.h"
#include <math.h>
#include "simplemap.h"
#include "math_utils.h"
#include "macros.h"
#include <ctime>

using namespace RayTracer;
using namespace std;


/* 0 for success */
int load_env_from_json(Environment& env, JO jo);
int add_circle(Environment& env, JO jo);
int add_polyline(Environment& env, JO jo);

JO query_environment(Environment&env, 
	double position[2], double orientation, vector<double>& directions);

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
	
		clock_t c0 = clock();
		time_t t0;  time(&t0);
		
	
		JO jo_class = json_object_object_get(jo, "class");
			expect(jo_expect_string(jo_class));
		string class_name = jo_get_string(jo_class);
		
		if(class_name == "map") {
			env = Environment();
			if(0 != load_env_from_json(env, jo)) {
				sm_error("Could not properly load map.\n");
				return -4;
			}
		} else if(class_name == "add_circle") {
			// XXX: error handling
			add_circle(env, jo);
		} else if(class_name == "add_polyline") {
			// XXX: error handling
			add_polyline(env, jo);
		} else if(class_name == "sensor") {
			JO jo_directions = json_object_object_get(jo, "directions");
				expect(jo_expect_array(jo_directions));
				
			int num_directions = jo_array_length(jo_directions);
			double directions_d[num_directions];
			expect(jo_read_from_double_array(jo_directions, directions_d, num_directions, NAN));
			directions = vector<double>(directions_d, directions_d + num_directions);
			
		} else if(class_name == "query_sensor") {
			double position[2]; double orientation;
			expect(jo_read_double(jo, "orientation", &orientation));
			expect(jo_read_double_array(jo, "position", position, 2, NAN));
			
			if(is_nan(orientation) || any_nan(position, 2)) {
				sm_error("Invalid pose (%f,%f,%f)\n", position[0],position[1],orientation);
				return -5;
			}
			
			JO response = query_environment(env, position, orientation,
					 						directions);
			jo_add_string(response, "class",  "query_sensor_response");
			
			std::cout << json_object_to_json_string(response) << std::endl;
			std::cout.flush();
			jo_free(response);
		} else if(class_name == "query_circle") {
			double center[2]; double radius;
			expect(jo_read_double(jo, "radius", &radius));
			expect(jo_read_double_array(jo, "center", center, 2, NAN));

			if(is_nan(radius) || any_nan(center, 2)) {
				sm_error("Invalid pose (%f,%f,%f)\n", center[0],center[1],radius);
				return -5;
			}

			int surface_id = -1;
			bool intersect = env.check_circle_intersection(center, radius, surface_id);
			
			JO response = json_object_new_object();
			jo_add_string(response, "class",  "query_circle_response");
			jo_add_int(response, "intersects",  intersect ? 1 : 0);
			jo_add_int(response, "surface",  surface_id);
			
			std::cout << json_object_to_json_string(response) << std::endl;
			std::cout.flush();
			jo_free(response);

		} else {
			sm_error("Unknown message '%s'.\n", class_name.c_str());
			return -3;
		}
		
		clock_t c1 = clock();
		time_t t1;  time(&t1);
		double deltat_s = difftime(t1,t0);
		
		double deltac_s = ((double)(c1-c0)) / CLOCKS_PER_SEC;

		if(0) {
			fprintf(stderr, "Answered in %.2f ms (%.2f clock)\n", 
				deltat_s*1000, deltac_s*1000);
		}
		//for(int a =0; a<env.stuff.size();a++) {
		//	fprintf(stderr, " stuff[%d] id %d\n", a, env.stuff[a]->surface_id);
		//}
		
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
 

