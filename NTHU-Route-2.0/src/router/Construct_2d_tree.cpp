#include "Construct_2d_tree.h"

#include "parameter.h"

#include "Route_2pinnets.h"

#include "flute/flute4nthuroute.h"

#include "grdb/RoutingComponent.h"

#include "util/traversemap.h"

#include "misc/geometry.h"

#include <climits>

#include <cmath>

#include <algorithm>

#include <unordered_map>

using namespace std;

using namespace Jm;

// #define GARY

/***********************

 * Global Variable Begin

 * ********************/

int par_ind = 0;

ParameterSet *parameter_set;

RoutingParameters *routing_parameter;

const int dir_array[4][2] = {{0, 1}, {0, -1}, {-1, 0}, {1, 0}}; // FRONT,BACK,LEFT,RIGHT

const int Jr2JmDirArray[4] = {0, 1, 3, 2}; // FRONT,BACK,LEFT,RIGHT <-> North, South, East, West

EdgePlane<Edge_2d> *congestionMap2d;

vector<Two_pin_element_2d *> two_pin_list;

int two_pin_list_size;

int flute_mode;

Monotonic_element **cong_monotonic; // store max congestion during monotonic path

int **parent_monotonic; // record parent (x,y) during finding monotonic path

Coordinate_2d **coor_array;

Vertex_3d ***cur_map_3d;

vector<Two_pin_element *> all_two_pin_list;

std::vector<std::vector<double>> discountRatio;

std::vector<std::vector<double>> discountRatioV;

std::vector<std::vector<double>> discountRatioH;

std::vector<std::vector<double>> discountRatioNonZeroV;
std::vector<std::vector<double>> discountRatioNonZeroH;

RoutingRegion *rr_map;

int cur_iter;

int done_iter;

double alpha;

int total_overflow;

int used_cost_flag;

int BOXSIZE_INC = 10;

Multisource_multisink_mazeroute *mazeroute_in_range = NULL;

int via_cost = 3;

double max_congestion_factor = 1.0;

Tree *net_flutetree;

EdgePlane<CacheEdge> *cache = NULL;

extern std::string TESTCASE_NAME;
extern std::chrono::high_resolution_clock::time_point pattern_start;
extern std::chrono::high_resolution_clock::time_point pattern_end;
extern std::chrono::high_resolution_clock::time_point main_start;
extern std::chrono::high_resolution_clock::time_point main_end;

/***********************

 * Global Variable End

 * ********************/

static vector<Two_pin_list_2d *> net_2pin_list; // store 2pin list of each net

static Tree global_flutetree;

static vector<Two_pin_list_2d *> bbox_2pin_list; // store bbox 2pin list of each net

static vector<Vertex_flute_ptr> vertex_fl;

static EdgeColorMap<int> *bboxRouteStateMap;

static double factor = 1.0;

static double exponent = 5.0;

static double WL_Cost = 1.0;

static double adjust_value = 0;

extern int maxTurns;

Edge_2d::Edge_2d()

	: cur_cap(0.),

	  max_cap(0.),

	  history(1),

	  used_net(128)

{
}

Edge_3d::Edge_3d()

	: max_cap(0),

	  cur_cap(0),

	  used_net(5)

{
}

Edge_3d_ptr Create_Edge_3d()

{

	return new Edge_3d;
}

int sign(double x) {
	const double EPS = 1e-5;
	if (fabs(x) < EPS)
		return 0;
	if (x + EPS < 0)
		return -1;
	return 1;
}

/*==================DEBUG FUNCTION================================*/

// Obtain the max. overflow and total overflowed value of edges of every gCell

int cal_max_overflow()

{

	int max_2d_of = 0; // max. overflow (2D)

	int dif_curmax = 0;

	// obtain the max. overflow and total overflowed value of RIGHT edge of every gCell

	for (int i = rr_map->get_gridx() - 2; i >= 0; --i)

	{

		for (int j = rr_map->get_gridy() - 1; j >= 0; --j)

		{

			if (congestionMap2d->edge(i, j, DIR_EAST).isOverflow()) // overflow occur

			{

				max_2d_of = max(max_2d_of, congestionMap2d->edge(i, j, DIR_EAST).overUsage());
				dif_curmax += congestionMap2d->edge(i, j, DIR_EAST).overUsage();
			}
		}
	}

	// obtain the max. overflow and total overflowed value of FRONT edge of every gCell

	for (int i = rr_map->get_gridx() - 1; i >= 0; --i)

	{

		for (int j = rr_map->get_gridy() - 2; j >= 0; --j)

		{

			if (congestionMap2d->edge(i, j, DIR_NORTH).isOverflow()) // overflow occur

			{

				max_2d_of = max(max_2d_of, congestionMap2d->edge(i, j, DIR_NORTH).overUsage());
				
				dif_curmax += congestionMap2d->edge(i, j, DIR_NORTH).overUsage();
			}
		}
	}

	printf("\033[32mcal max overflow=%d   cur_cap-max_cap=%d\033[m\n", max_2d_of, dif_curmax);

	return dif_curmax;
}

// Sum all demand value on every edge

// So if demand vlaue = wire length, this function can be used

int cal_total_wirelength()

{

	int total_wl = 0;

	for (int i = rr_map->get_gridx() - 2; i >= 0; --i)

	{

		for (int j = rr_map->get_gridy() - 1; j >= 0; --j)

		{

			total_wl += (int)congestionMap2d->edge(i, j, RIGHT).cur_cap;
		}
	}

	for (int i = rr_map->get_gridx() - 1; i >= 0; --i)

	{

		for (int j = rr_map->get_gridy() - 2; j >= 0; --j)

		{

			total_wl += (int)congestionMap2d->edge(i, j, FRONT).cur_cap;
		}
	}

	printf("total wirelengh:%d\n", total_wl);

	return total_wl;
}

/*sort bbox in ascending order, then pin_num in descending order*/

bool comp_net(const Net *a, const Net *b)

{

	if (a->get_bboxSize() > b->get_bboxSize())

	{

		return true;
	}

	else if (a->get_bboxSize() < b->get_bboxSize())

	{

		return false;
	}

	else

	{

		return (a->get_pinNumber() < b->get_pinNumber());
	}
}

bool comp_2pin_net(Two_pin_element *a, Two_pin_element *b)

{

	int a_bbox_size = abs(a->pin1.x - a->pin2.x) + abs(a->pin1.y - a->pin2.y);

	int b_bbox_size = abs(b->pin1.x - b->pin2.x) + abs(b->pin1.y - b->pin2.y);

	return (a_bbox_size < b_bbox_size);
}

bool comp_2pin_net_from_path(Two_pin_element_2d *a, Two_pin_element_2d *b)

{

	int a_bbox_size = abs(a->pin1.x - a->pin2.x) + abs(a->pin1.y - a->pin2.y);

	int b_bbox_size = abs(b->pin1.x - b->pin2.x) + abs(b->pin1.y - b->pin2.y);

	return (a_bbox_size < b_bbox_size);
}

// sort by x,y,pin,steiner

bool comp_vertex_fl(Vertex_flute_ptr a, Vertex_flute_ptr b)

{

	if (a->x < b->x)

		return true;

	else if (a->x > b->x)

		return false;

	else if (a->y < b->y)

		return true;

	else if (a->y > b->y)

		return false;

	else if (a->type == PIN)

		return true;

	else

		return false;
}

void setup_flute_order(int *order)

{

	for (int i = global_flutetree.number - 1; i >= 0; --i)

	{

		order[i] = i;
	}
}

/*assign the estimated track# to each edge*/

void init_2d_map()

{

	congestionMap2d = new EdgePlane<Edge_2d>(rr_map->get_gridx(),

											 rr_map->get_gridy(),

											 Edge_2d());

	for (int x = rr_map->get_gridx() - 2; x >= 0; --x)

	{

		for (int y = rr_map->get_gridy() - 1; y >= 0; --y)

		{

			for (int layer = rr_map->get_layerNumber() - 1; layer >= 0; --layer)

			{

#ifdef IBM_CASE

				// There is no wire spacing, so

				// the edge capacity on congestion map = edge capacity on every layer

				congestionMap2d->edge(x, y, DIR_EAST).max_cap += rr_map->capacity(layer, x, y, x + 1, y);

#else

				// Because the wire width = 1 and wire spaceing = 1,

				// the edge capacity on congestion map = edge capacity on every layer /2.

				congestionMap2d->edge(x, y, DIR_EAST).max_cap += (rr_map->capacity(layer, x, y, x + 1, y));

#endif
			}
		}
	}

	for (int x = rr_map->get_gridx() - 1; x >= 0; --x)

	{

		for (int y = rr_map->get_gridy() - 2; y >= 0; --y)

		{

			for (int layer = rr_map->get_layerNumber() - 1; layer >= 0; --layer)

			{

#ifdef IBM_CASE

				// There is no wire spacing, so

				// the edge capacity on congestion map = edge capacity on every layer

				congestionMap2d->edge(x, y, DIR_NORTH).max_cap += rr_map->capacity(layer, x, y, x, y + 1);

#else

				// Because the wire width = 1 and wire spaceing = 1,

				// the edge capacity on congestion map = edge capacity on every layer /2.

				congestionMap2d->edge(x, y, DIR_NORTH).max_cap += (rr_map->capacity(layer, x, y, x, y + 1));

#endif
			}
		}
	}
}

// Make an coordinate array which contains the (x, y) information

void allocate_coor_array()

{

	int i, j;

	Coordinate_2d *tmp_data;

	coor_array = (Coordinate_2d **)malloc(rr_map->get_gridx() * sizeof(Coordinate_2d *));

	tmp_data = (Coordinate_2d *)malloc(rr_map->get_gridx() * rr_map->get_gridy() * sizeof(Coordinate_2d));

	for (i = 0; i < rr_map->get_gridx(); ++i, tmp_data += rr_map->get_gridy())

		coor_array[i] = tmp_data;

	for (i = 0; i < rr_map->get_gridx(); ++i)

	{

		for (j = 0; j < rr_map->get_gridy(); ++j)

		{

			coor_array[i][j].x = i;

			coor_array[i][j].y = j;
		}
	}
}

void init_3d_map()

{

	int i, j, k;

	Vertex_3d **tmp_data, *tmp_data2;

	Edge_3d_ptr newedge;

	/*allocate space for cur_map_3d*/

	cur_map_3d = (Vertex_3d ***)malloc(rr_map->get_gridx() * sizeof(Vertex_3d **));

	tmp_data = (Vertex_3d **)malloc(rr_map->get_gridx() * rr_map->get_gridy() * sizeof(Vertex_3d *));

	for (i = 0; i < rr_map->get_gridx(); ++i, tmp_data += rr_map->get_gridy())

		cur_map_3d[i] = tmp_data;

	tmp_data2 = (Vertex_3d *)malloc(rr_map->get_gridx() * rr_map->get_gridy() * rr_map->get_layerNumber() * sizeof(Vertex_3d));

	for (i = 0; i < rr_map->get_gridx(); ++i)

		for (j = 0; j < rr_map->get_gridy(); ++j, tmp_data2 += rr_map->get_layerNumber())

			cur_map_3d[i][j] = tmp_data2;

	// initialize capacity

	for (i = 0; i < rr_map->get_gridx() - 1; ++i)

		for (j = 0; j < rr_map->get_gridy(); ++j)

			for (k = 0; k < rr_map->get_layerNumber(); ++k)

			{

				newedge = Create_Edge_3d(); /*allocate space for edge_list without initialization*/
#ifdef GARY				
				if (k == 0)

					cur_map_3d[i][j][k].discountRatio = discountRatio[i][j];

				else if (database.getLayerDir(k) == X && k <= 2) // k = 2
				{
					cur_map_3d[i][j][k].discountRatio = discountRatioV[i][j];
				}
				else if (database.getLayerDir(k) == Y && k <= 2) // k = 1
				{
					cur_map_3d[i][j][k].discountRatio = discountRatioH[i][j];
				}
				else if (database.getLayerDir(k) == X) // k = 4
				{
					cur_map_3d[i][j][k].discountRatio = discountRatioNonZeroV[i][j];
				}
				else if (database.getLayerDir(k) == Y) // k = 3
				{
					cur_map_3d[i][j][k].discountRatio = discountRatioNonZeroH[i][j];
				} 
#endif
				cur_map_3d[i][j][k].edge_list[RIGHT] = newedge;

				cur_map_3d[i + 1][j][k].edge_list[LEFT] = newedge;
			}

	for (i = 0; i < rr_map->get_gridx(); ++i)

		for (j = 0; j < rr_map->get_gridy() - 1; ++j)

			for (k = 0; k < rr_map->get_layerNumber(); ++k)

			{

				newedge = Create_Edge_3d(); /*allocate space for edge_list without initialization*/

				cur_map_3d[i][j][k].edge_list[FRONT] = newedge;

				cur_map_3d[i][j + 1][k].edge_list[BACK] = newedge;
			}

	for (i = 0; i < rr_map->get_gridx(); ++i)

		for (j = 0; j < rr_map->get_gridy(); ++j)

			for (k = 0; k < rr_map->get_layerNumber() - 1; ++k)

			{

				newedge = Create_Edge_3d(); /*allocate space for edge_list without initialization*/

				cur_map_3d[i][j][k].edge_list[UP] = newedge;

				cur_map_3d[i][j][k + 1].edge_list[DOWN] = newedge;
			}

	for (j = 0; j < rr_map->get_gridy(); ++j)

		for (k = 0; k < rr_map->get_layerNumber(); ++k)

			cur_map_3d[0][j][k].edge_list[LEFT] = cur_map_3d[rr_map->get_gridx() - 1][j][k].edge_list[RIGHT] = NULL;

	for (i = 0; i < rr_map->get_gridx(); ++i)

		for (k = 0; k < rr_map->get_layerNumber(); ++k)

			cur_map_3d[i][0][k].edge_list[BACK] = cur_map_3d[i][rr_map->get_gridy() - 1][k].edge_list[FRONT] = NULL;

	for (i = 0; i < rr_map->get_gridx(); ++i)

		for (j = 0; j < rr_map->get_gridy(); ++j)

			cur_map_3d[i][j][0].edge_list[DOWN] = cur_map_3d[i][j][rr_map->get_layerNumber() - 1].edge_list[UP] = NULL;
}

void init_2pin_list()

{

	int i, netnum;

	netnum = rr_map->get_netNumber();

	for (i = 0; i < netnum; ++i)

	{

		// Two pin nets group by net id. So for fetching the 2nd net's 2-pin net,

		// you can fetch by net_2pin_list[2][i], where i is the id of 2-pin net.

		net_2pin_list.push_back(new Two_pin_list_2d);

		bbox_2pin_list.push_back(new Two_pin_list_2d);
	}
}

void init_flute()

{

	net_flutetree = (Tree *)malloc(rr_map->get_netNumber() * sizeof(Tree));
}

void free_memory_con2d()

{

	for (vector<Two_pin_list_2d *>::iterator it = bbox_2pin_list.begin(); it != bbox_2pin_list.end(); ++it)

		delete (*it);

	bbox_2pin_list.clear();
}

void bbox_route(Two_pin_list_2d *list, const double value)

{

	int i, x1, y1, x2, y2;

	double u_value;

	if (value > 0)

		u_value = 1;

	else

		u_value = -1;

	for (vector<Two_pin_element_2d *>::iterator it = list->begin();

		 it != list->end();

		 ++it)

	{

		// to create the bounding box
		if ((*it)->pin1.x > (*it)->pin2.x)

		{

			swap((*it)->pin1.x, (*it)->pin2.x);
		}

		if ((*it)->pin1.y > (*it)->pin2.y)

		{

			swap((*it)->pin1.y, (*it)->pin2.y);
		}

		x1 = (*it)->pin1.x;

		y1 = (*it)->pin1.y;

		x2 = (*it)->pin2.x;

		y2 = (*it)->pin2.y;

#ifdef DEBUG_BBOX

		printf("(%d %d) (%d %d)\n", x1, y1, x2, y2);

#endif

		// to connect the net
		if (x1 == x2) // vertical edge

		{

			for (i = y1; i < y2; ++i)

			{

				bboxRouteStateMap->color(x1, i, DIR_NORTH) = 1;
			}
		}

		else if (y1 == y2) // horizontal edge

		{

			for (i = x1; i < x2; ++i)

			{

				bboxRouteStateMap->color(i, y1, DIR_EAST) = 1;
			}
		}

		else // box (L-shape routing need

		{

			for (i = y1; i < y2; ++i)

			{

				bboxRouteStateMap->color(x1, i, DIR_NORTH) = 1;

				bboxRouteStateMap->color(x2, i, DIR_NORTH) = 1;
			}

			for (i = x1; i < x2; ++i)

			{

				bboxRouteStateMap->color(i, y1, DIR_EAST) = 1;

				bboxRouteStateMap->color(i, y2, DIR_EAST) = 1;
			}
		}
	}

	// check the flag of edges, if it is set to 1, then add demand on it.
	// to add the demand of the wire

	for (vector<Two_pin_element_2d *>::iterator it = list->begin();

		 it != list->end();

		 ++it)

	{

		// x1, y1 is the smaller set

		// NOTE: they have been sorted in the last for loop

		x1 = (*it)->pin1.x;

		y1 = (*it)->pin1.y;

		x2 = (*it)->pin2.x;

		y2 = (*it)->pin2.y;

		if (x1 == x2) // vertical edge

		{

			for (i = y1; i < y2; ++i)

				if (bboxRouteStateMap->color(x1, i, DIR_NORTH) == 1)

				{

					congestionMap2d->edge(x1, i, DIR_NORTH).cur_cap += u_value;

					bboxRouteStateMap->color(x1, i, DIR_NORTH) = 0;
				}
		}

		else if (y1 == y2) // horizontal edge

		{

			for (i = x1; i < x2; ++i)

				if (bboxRouteStateMap->color(i, y1, DIR_EAST) == 1)

				{
					congestionMap2d->edge(i, y1, DIR_EAST).cur_cap += u_value;

					bboxRouteStateMap->color(i, y1, DIR_EAST) = 0;
				}
		}

		else // box

		{

			for (i = y1; i < y2; ++i)

			{

				if (bboxRouteStateMap->color(x1, i, DIR_NORTH) == 1)

				{

					congestionMap2d->edge(x1, i, DIR_NORTH).cur_cap += value;

					bboxRouteStateMap->color(x1, i, DIR_NORTH) = 0;
				}

				if (bboxRouteStateMap->color(x2, i, DIR_NORTH) == 1)

				{

					congestionMap2d->edge(x2, i, DIR_NORTH).cur_cap += value;

					bboxRouteStateMap->color(x2, i, DIR_NORTH) = 0;
				}
			}

			for (i = x1; i < x2; ++i)

			{

				if (bboxRouteStateMap->color(i, y1, DIR_EAST) == 1)

				{

					congestionMap2d->edge(i, y1, DIR_EAST).cur_cap += value;

					bboxRouteStateMap->color(i, y1, DIR_EAST) = 0;
				}

				if (bboxRouteStateMap->color(i, y2, DIR_EAST) == 1)

				{

					congestionMap2d->edge(i, y2, DIR_EAST).cur_cap += value;

					bboxRouteStateMap->color(i, y2, DIR_EAST) = 0;
				}
			}
		}
	}

#ifdef DEBUG_BBOX

	print_cap("cur");

#endif
}

void insert_all_two_pin_list(Two_pin_element_2d *mn_path_2d)

{

	Two_pin_element *mn_path;

	mn_path = new Two_pin_element();

	mn_path->pin1.x = mn_path_2d->pin1.x;

	mn_path->pin1.y = mn_path_2d->pin1.y;

	mn_path->pin2.x = mn_path_2d->pin2.x;

	mn_path->pin2.y = mn_path_2d->pin2.y;

	mn_path->pin1.z = mn_path->pin2.z = 0;

	mn_path->net_id = mn_path_2d->net_id;

	all_two_pin_list.push_back(mn_path);
}

// importance: overflow > WL > #via
inline bool smaller_than_lower_bound(

	double total_cost, int distance, int via_num,

	double bound_cost, int bound_distance, int bound_via_num)

{

	if ((total_cost - bound_cost) < neg_error_bound)

		return true;

	else if ((total_cost - bound_cost) > error_bound)

		return false;

	else

	{
		// std::cout << "Tie\n";

		if (distance < bound_distance)

			return true;

		else if (distance > bound_distance)

			return false;

		else

		{

			return (via_num < bound_via_num);
		}
	}
}

/* *NOTICE*

 * You can create many different cost function for difference case easily,

 * just reassign function ponter pre_evaluate_congestion_cost_fp to your

 * function in *route/route.cpp* .                                        */

// Here, pre_evaluate_congestion_cost_fp = pre_evaluate_congestion_cost_all
void (*pre_evaluate_congestion_cost_fp)(int i, int j, int dir);

void pre_evaluate_congestion_cost_all(int i, int j, int dir)

{

	static const int inc = 1;

	double cong;

	DirectionType dirType = static_cast<DirectionType>(Jr2JmDirArray[dir]);

	// if capacity == 0, then the cost is very large
	if (sign(congestionMap2d->edge(i, j, dirType).max_cap) == 0) {
		cache->edge(i, j, dirType).cost = 1e9;
		return;
	}

	if (used_cost_flag == HISTORY_COST)		// main stage for MM routing

	{

		cong = (congestionMap2d->edge(i, j, dirType).cur_cap + inc) /

			   (congestionMap2d->edge(i, j, dirType).max_cap *

				(1.0 - ((congestionMap2d->edge(i, j, dirType).history - 1) /

						(cur_iter * (1.5 + 3 * factor)))));

		cache->edge(i, j, dirType).cost = WL_Cost +

										  (congestionMap2d->edge(i, j, dirType).history) * pow(cong, exponent);
	}

	else	// refinement stage

	{
		// cache->edge(i, j, dirType).cost = (congestionMap2d->edge(i, j, dirType).isFull()) 
		// ? 1 + congestionMap2d->edge(i, j, dirType).overUsage() : 0;
		if (congestionMap2d->edge(i, j, dirType).isFull())

			cache->edge(i, j, dirType).cost = 1.0;

		else

			cache->edge(i, j, dirType).cost = 0.0;
	}
	assert(sign(cache->edge(i, j, dirType).cost) >= 0);

}

void pre_evaluate_congestion_cost()

{

	for (int i = rr_map->get_gridx() - 1; i >= 0; --i)

	{

		for (int j = rr_map->get_gridy() - 2; j >= 0; --j)

		{

			pre_evaluate_congestion_cost_fp(i, j, FRONT); // Function Pointer to Cost function

			if (congestionMap2d->edge(i, j, DIR_NORTH).isOverflow())

			{

				++congestionMap2d->edge(i, j, DIR_NORTH).history;
			}
		}
	}

	for (int i = rr_map->get_gridx() - 2; i >= 0; --i)

	{

		for (int j = rr_map->get_gridy() - 1; j >= 0; --j)

		{

			pre_evaluate_congestion_cost_fp(i, j, RIGHT);

			if (congestionMap2d->edge(i, j, DIR_EAST).isOverflow())

			{

				++congestionMap2d->edge(i, j, DIR_EAST).history;
			}
		}
	}
}

// get edge cost on a 2D layer

double get_cost_2d(int i, int j, int dir, int net_id, int *distance)

{

	DirectionType dirType = static_cast<DirectionType>(Jr2JmDirArray[dir]);

	// Check if the specified net pass the edge.

	// If it have passed the edge before, then the cost is 0.

	if (sign(congestionMap2d->edge(i, j, dirType).max_cap) == 0) {
		(*distance) = 1e5;
		return 1e9;
	}

	if (congestionMap2d->edge(i, j, dirType).lookupNet(net_id) == false)

	{

		(*distance) = 1;

		// Used in part II

		if (used_cost_flag == HISTORY_COST)

		{

			return cache->edge(i, j, dirType).cost;
		}

		// Used in part III: Post processing

		else if (used_cost_flag == MADEOF_COST)

		{
			// return (congestionMap2d->edge(i, j, dirType).overUsage());
			return (congestionMap2d->edge(i, j, dirType).isFull());
		}

		// Used in part I: Initial routing

		else if (used_cost_flag == FASTROUTE_COST)

		{

			return 1 + parameter_h /

						   (1 + exp((-1) * parameter_k *

									(congestionMap2d->edge(i, j, dirType).cur_cap + 1 -

									 congestionMap2d->edge(i, j, dirType).max_cap)));
		}

		return 0;
	}

	else

	{

		(*distance) = 0;

		return 0;
	}
}

/*

Compare two cost and return a pointer to the Monotonici_element which has smaller cost

*/

Monotonic_element *compare_cost(Monotonic_element *m1, Monotonic_element *m2)

{

	if ((m1->total_cost - m2->total_cost) < (neg_error_bound))

		return m1;

	else if ((m1->total_cost - m2->total_cost) > (neg_error_bound))

		return m2;

	else

	{

		if ((m1->max_cost - m2->max_cost) < (neg_error_bound))

			return m1;

		else if ((m1->max_cost - m2->max_cost) > (neg_error_bound))

			return m2;

		else

		{

			if (m1->distance < m2->distance)

				return m1;

			else if (m1->distance > m2->distance)

				return m2;

			else

			{

				if (m1->via_num <= m2->via_num)

					return m1;

				else

					return m2;
			}
		}
	}
}

/*

input: start coor and end coor, and directions of L

output: record the best L pattern into two_pin_L_path_global, and return the min max congestion value

*/

Monotonic_element L_pattern_max_cong(int x1, int y1, int x2, int y2, int dir1, int dir2, Two_pin_element_2d *two_pin_L_path, int net_id)

{

	int i, j;

	double temp;

	int dir[2], dir_index;

	Monotonic_element max_path;

	int distance;

	dir[0] = dir1;

	dir[1] = dir2;

	i = x1;

	j = y1;

	max_path.max_cost = -1000000;

	max_path.total_cost = 0;

	max_path.net_cost = 0;

	max_path.distance = 0;

	max_path.via_num = 1;

	for (dir_index = 0; dir_index < 2; dir_index++)

	{

		if (dir[dir_index] == RIGHT) // search in horizontal direction: RIGHT

		{

			// for loop from the left boundary to the right boundary

			for (i = x1; i < x2; ++i)

			{

				temp = get_cost_2d(i, j, dir[dir_index], net_id, &distance);

				max_path.total_cost += max(static_cast<double>(0), temp);

				max_path.distance += distance;

				(*two_pin_L_path).path.push_back(&coor_array[i][j]);

				if (temp > max_path.max_cost)

					max_path.max_cost = temp;
			}

			i = x2;
		}

		else // search in vertical direction

		{

			for (j = y1; (j < y2 && dir[dir_index] == FRONT) || (j > y2 && dir[dir_index] == BACK);)

			{

				temp = get_cost_2d(i, j, dir[dir_index], net_id, &distance);

				max_path.total_cost += max(static_cast<double>(0), temp);

				max_path.distance += distance;

				(*two_pin_L_path).path.push_back(&coor_array[i][j]);

				if (temp > max_path.max_cost)

					max_path.max_cost = temp;

				if (dir[dir_index] == FRONT)

					++j;

				else if (dir[dir_index] == BACK)

					--j;
			}

			j = y2;
		}
	}

	(*two_pin_L_path).path.push_back(&coor_array[x2][y2]);

	return max_path;
}

/*

input: two coordinates

output: record the L_pattern in the path, and the path is min max congestioned

*/

void L_pattern_route(int x1, int y1, int x2, int y2, Two_pin_element_2d *two_pin_L_path, int net_id)

{

	Two_pin_element_2d path1, path2;

	Monotonic_element max_cong_path1, max_cong_path2;

	if (x1 > x2)

	{

		swap(x1, x2);

		swap(y1, y2);
	}

	if (x1 < x2 && y1 < y2) // two points are left_back and right_front

	{

		// FRONT and RIGHT L pattern from (x1 y1)

		max_cong_path1 = L_pattern_max_cong(x1, y1, x2, y2, FRONT, RIGHT, &path1, net_id);

		// RIGTH and FRONT L pattern from (x1,y1)

		max_cong_path2 = L_pattern_max_cong(x1, y1, x2, y2, RIGHT, FRONT, &path2, net_id);

		if ((&max_cong_path1) == (compare_cost(&max_cong_path1, &max_cong_path2)))

			(*two_pin_L_path) = path1;

		else

			(*two_pin_L_path) = path2;
	}

	else if (x1 < x2 && y1 > y2) // two points are left_front and right_back

	{

		// BACK and RIGHT L pattern from (x1,y1)

		max_cong_path1 = L_pattern_max_cong(x1, y1, x2, y2, BACK, RIGHT, &path1, net_id);

		// RIGHT and BACK L pattern from (x1,y1)

		max_cong_path2 = L_pattern_max_cong(x1, y1, x2, y2, RIGHT, BACK, &path2, net_id);

		if ((&max_cong_path1) == (compare_cost(&max_cong_path1, &max_cong_path2)))

			(*two_pin_L_path) = path1;

		else

			(*two_pin_L_path) = path2;
	}

	else // vertical or horizontal line

	{

		if (y1 > y2)

		{

			swap(y1, y2);
		}

		max_cong_path1 = L_pattern_max_cong(x1, y1, x2, y2, FRONT, RIGHT, two_pin_L_path, net_id);
	}

	(*two_pin_L_path).pin1 = *((*two_pin_L_path).path[0]);

	(*two_pin_L_path).pin2 = *((*two_pin_L_path).path.back());

	(*two_pin_L_path).net_id = net_id;
}

/*allocate space for monotonic pattern routing*/

void allocate_monotonic()

{

	Monotonic_element *tmp_data;

	int *tmp_data2;

	/*allocate space for cong_monotonic*/

	cong_monotonic = (Monotonic_element **)malloc(rr_map->get_gridx() * sizeof(Monotonic_element *));

	tmp_data = (Monotonic_element *)malloc(rr_map->get_gridx() * rr_map->get_gridy() * sizeof(Monotonic_element));

	for (int i = 0; i < rr_map->get_gridx(); ++i, tmp_data += rr_map->get_gridy())

		cong_monotonic[i] = tmp_data;

	/*allocate space for parent_monotonic*/

	parent_monotonic = (int **)malloc(rr_map->get_gridx() * sizeof(int *));

	tmp_data2 = (int *)malloc(rr_map->get_gridx() * rr_map->get_gridy() * sizeof(int));

	for (int i = 0; i < rr_map->get_gridx(); ++i, tmp_data2 += rr_map->get_gridy())

		parent_monotonic[i] = tmp_data2;
}

void compare_two_direction_congestion(int i, int j, int dir1, int pre_i, int dir2, int pre_j, int net_id, double bound_cost, int bound_distance, int bound_via_num, bool bound_flag)

{

	Monotonic_element left_element, vertical_element, *choose_element;

	double cost;

	int distance = 1;

	bool left_flag, right_flag;

	int pre_dir;

	left_flag = right_flag = true;

	if (parent_monotonic[pre_i][j] != -2)

	{

		cost = get_cost_2d(i, j, dir1, net_id, &distance);

		left_element.max_cost = max(cost, cong_monotonic[pre_i][j].max_cost);

		left_element.total_cost = cong_monotonic[pre_i][j].total_cost +

								  max(static_cast<double>(0), cost);

		left_element.distance = cong_monotonic[pre_i][j].distance + distance;

		pre_dir = parent_monotonic[pre_i][j];

		if (pre_dir < 2)

		{

			left_element.via_num = cong_monotonic[pre_i][j].via_num + via_cost;

			if (distance != 0)

			{

				left_element.distance += via_cost;

				if (used_cost_flag == HISTORY_COST)

					left_element.total_cost += via_cost;
			}
		}

		else

			left_element.via_num = cong_monotonic[pre_i][j].via_num;

		if (!bound_flag || (bound_flag && smaller_than_lower_bound(left_element.total_cost, left_element.distance, left_element.via_num, bound_cost, bound_distance, bound_via_num)))

		{
			left_flag = true;
		}

		else

			left_flag = false;
	}

	else

		left_flag = false;

	if (parent_monotonic[i][pre_j] != -2)

	{

		cost = get_cost_2d(i, j, dir2, net_id, &distance);

		vertical_element.max_cost = max(cost, cong_monotonic[i][pre_j].max_cost);

		vertical_element.total_cost = cong_monotonic[i][pre_j].total_cost +

									  max(static_cast<double>(0), cost);

		vertical_element.distance = cong_monotonic[i][pre_j].distance + distance;

		pre_dir = parent_monotonic[i][pre_j];

		if (pre_dir >= 2)

		{

			vertical_element.via_num = cong_monotonic[i][pre_j].via_num + via_cost;

			if (distance != 0)

			{

				vertical_element.distance += via_cost;

				if (used_cost_flag == HISTORY_COST)

					vertical_element.total_cost += via_cost;
			}
		}

		else

			vertical_element.via_num = cong_monotonic[i][pre_j].via_num;

		if (!bound_flag || (bound_flag && smaller_than_lower_bound(vertical_element.total_cost, vertical_element.distance, vertical_element.via_num, bound_cost, bound_distance, bound_via_num)))

		{

			right_flag = true;
		}

		else

			right_flag = false;
	}

	else

		right_flag = false;

	if ((!left_flag) && (!right_flag))

	{

		parent_monotonic[i][j] = -2;

		return;
	}

	else if (left_flag && right_flag)

		choose_element = compare_cost(&left_element, &vertical_element);

	else if (left_flag)

		choose_element = &left_element;

	else

		choose_element = &vertical_element;

	cong_monotonic[i][j].max_cost = choose_element->max_cost;

	cong_monotonic[i][j].total_cost = choose_element->total_cost;

	cong_monotonic[i][j].net_cost = choose_element->net_cost;

	cong_monotonic[i][j].distance = choose_element->distance;

	cong_monotonic[i][j].via_num = choose_element->via_num;

	if (choose_element == (&left_element))

		parent_monotonic[i][j] = dir1;

	else if (choose_element == (&vertical_element))

		parent_monotonic[i][j] = dir2;

	else

	{

		puts("compare has problem!!!\n");

		exit(0);
	}
}

void monotonic_routing_algorithm(int x1, int y1, int x2, int y2, int dir, int net_id, double bound_cost, int bound_distance, int bound_via_num, bool bound_flag)

{

	int i, j;

	double cost;

	int distance = 1;

	// initialize cong_monotonic and parent_monotonic

	cong_monotonic[x1][y1].max_cost = -1000000;

	cong_monotonic[x1][y1].total_cost = 0;

	cong_monotonic[x1][y1].distance = 0;

	cong_monotonic[x1][y1].net_cost = 0;

	cong_monotonic[x1][y1].via_num = 0;

	parent_monotonic[x1][y1] = -1;

	// Update the cost of top boundary or bottom boundary, which draw with double line.

	// The source can in left-top corner or left-bottom corner

	for (i = x1 + 1; i <= x2; ++i)

	{

		if (parent_monotonic[i - 1][y1] != -2)

		{

			cost = get_cost_2d(i, y1, LEFT, net_id, &distance);

			cong_monotonic[i][y1].max_cost = max(cost, cong_monotonic[i - 1][y1].max_cost);

			cong_monotonic[i][y1].total_cost = cong_monotonic[i - 1][y1].total_cost + max(static_cast<double>(0), cost);

			cong_monotonic[i][y1].distance = cong_monotonic[i - 1][y1].distance + distance;

			cong_monotonic[i][y1].via_num = cong_monotonic[i - 1][y1].via_num;

			if (!bound_flag || (bound_flag && smaller_than_lower_bound(cong_monotonic[i][y1].total_cost, cong_monotonic[i][y1].distance, cong_monotonic[i][y1].via_num, bound_cost, bound_distance, bound_via_num)))

			{

				parent_monotonic[i][y1] = LEFT;
			}

			else

				parent_monotonic[i][y1] = -2;
		}

		else

			parent_monotonic[i][y1] = -2;
	}

	// If source is in the left-top corner

	if (dir == BACK)

	{

		for (j = y1 + 1; j <= y2; ++j)

		{

			if (parent_monotonic[x1][j - 1] != -2)

			{

				cost = get_cost_2d(x1, j, dir, net_id, &distance);

				cong_monotonic[x1][j].max_cost = max(cost, cong_monotonic[x1][j - 1].max_cost);

				cong_monotonic[x1][j].total_cost = cong_monotonic[x1][j - 1].total_cost + max(static_cast<double>(0), cost);

				cong_monotonic[x1][j].distance = cong_monotonic[x1][j - 1].distance + distance;

				cong_monotonic[x1][j].via_num = cong_monotonic[x1][j - 1].via_num;

				if (!bound_flag || (bound_flag && smaller_than_lower_bound(cong_monotonic[x1][j].total_cost, cong_monotonic[x1][j].distance, cong_monotonic[x1][j].via_num, bound_cost, bound_distance, bound_via_num)))

				{

					parent_monotonic[x1][j] = dir;
				}

				else

					parent_monotonic[x1][j] = dir;
			}

			else

				parent_monotonic[x1][j] = -2;
		}

		// If source is in the left-bottom corner
	}

	else if (dir == FRONT)

	{

		for (j = y1 - 1; j >= y2; --j)

		{

			if (parent_monotonic[x1][j + 1] != -2)

			{

				cost = get_cost_2d(x1, j, dir, net_id, &distance);

				cong_monotonic[x1][j].max_cost = max(cost, cong_monotonic[x1][j + 1].max_cost);

				cong_monotonic[x1][j].total_cost = cong_monotonic[x1][j + 1].total_cost + max(static_cast<double>(0), cost);

				cong_monotonic[x1][j].distance = cong_monotonic[x1][j + 1].distance + distance;

				cong_monotonic[x1][j].via_num = cong_monotonic[x1][j + 1].via_num;

				if (!bound_flag || (bound_flag && smaller_than_lower_bound(cong_monotonic[x1][j].total_cost, cong_monotonic[x1][j].distance, cong_monotonic[x1][j].via_num, bound_cost, bound_distance, bound_via_num)))

				{

					parent_monotonic[x1][j] = dir;
				}

				else

					parent_monotonic[x1][j] = -2;
			}

			else

				parent_monotonic[x1][j] = -2;
		}
	}

	// to judge whether go left or go top(down)
	for (i = x1 + 1; i <= x2; ++i)

	{

		// If source is in the left-bottom corner

		if (dir == BACK)

		{

			for (j = y1 + 1; j <= y2; ++j)

				compare_two_direction_congestion(i, j, LEFT, i - 1, dir, j - 1, net_id, bound_cost, bound_distance, bound_via_num, bound_flag);

			// If source is in the left-top corner
		}

		else

		{

			for (j = y1 - 1; j >= y2; --j)

				compare_two_direction_congestion(i, j, LEFT, i - 1, dir, j + 1, net_id, bound_cost, bound_distance, bound_via_num, bound_flag);
		}
	}
}

void traverse_parent_monotonic(int x1, int y1, int x2, int y2, Two_pin_element_2d *two_pin_monotonic_path)

{

	int i = x2;

	int j = y2;

	// Sink != Source

	while ((i != x1) || (j != y1))

	{

		// Push the path in to a list

		(*two_pin_monotonic_path).path.push_back(&coor_array[i][j]);

		// Update the coordinate of tracing cell

		if (parent_monotonic[i][j] == LEFT)

			--i;

		else if (parent_monotonic[i][j] == FRONT)

			++j;

		else

			--j;
	}

	// push the source to list

	(*two_pin_monotonic_path).path.push_back(&coor_array[i][j]);
}

// Try to obtain a monotonic routing path without cost over bounding cost

// Return true if there exist one such path

bool monotonic_pattern_route(int x1, int y1,

							 int x2, int y2,

							 Two_pin_element_2d *two_pin_monotonic_path,

							 int net_id,

							 double bound_cost,

							 int bound_distance,

							 int bound_via_num,

							 bool bound_flag)

{

	if (x1 > x2)

	{

		swap(x1, x2);

		swap(y1, y2);
	}

	if (y1 <= y2) // s->t RIGHT and FRONT (source is in the left-bottom corner)

		monotonic_routing_algorithm(x1, y1, x2, y2, BACK, net_id, bound_cost, bound_distance, bound_via_num, bound_flag);

	// use x1,y+1.edge_list[back]

	else if (y1 > y2) // s->t RIGHT and BACK (source is in the left-top corner)

		monotonic_routing_algorithm(x1, y1, x2, y2, FRONT, net_id, bound_cost, bound_distance, bound_via_num, bound_flag);

	// If there is no solution for this 2-pin net, return false

	if (parent_monotonic[x2][y2] == -2)

		return false;

	// travese parent_monotonic to find path in two_pin_monotonic_path

	traverse_parent_monotonic(x1, y1, x2, y2, two_pin_monotonic_path);

	(*two_pin_monotonic_path).pin1 = *((*two_pin_monotonic_path).path[0]);

	(*two_pin_monotonic_path).pin2 = *((*two_pin_monotonic_path).path.back());

	(*two_pin_monotonic_path).net_id = net_id;

	return true;
}

// Add the path of two pin element on to congestion map

// The congestion map record not only which net pass which edge,

// but also the number of a net pass through

void update_congestion_map_insert_two_pin_net(Two_pin_element_2d *element)

{

	int dir;

	NetDirtyBit[element->net_id] = true;

	for (int i = element->path.size() - 2; i >= 0; --i)

	{

		// get an edge from congestion map - c_map_2d

		dir = get_direction_2d(element->path[i], element->path[i + 1]);

		pair<RoutedNetTable::iterator, bool> insert_result =

			congestionMap2d->edge(element->path[i]->x, element->path[i]->y, dir).used_net.insert(pair<int, int>(element->net_id, 1));

		if (!insert_result.second)

			++((insert_result.first)->second);	// insert the routing edge

		else

		{

			++(congestionMap2d->edge(element->path[i]->x, element->path[i]->y, dir).cur_cap);

			if (used_cost_flag != FASTROUTE_COST)

			{

				pre_evaluate_congestion_cost_fp(element->path[i]->x, element->path[i]->y, dir);
			}
		}
	}
}

// Remove a net from an edge.

// If the net pass that edge more than once, this function will only decrease the counter.

void update_congestion_map_remove_two_pin_net(Two_pin_element_2d *element)

{

	int dir;

	NetDirtyBit[element->net_id] = true;

	for (int i = element->path.size() - 2; i >= 0; --i)

	{

		dir = get_direction_2d(element->path[i], element->path[i + 1]);

		RoutedNetTable::iterator find_result =

			congestionMap2d->edge(element->path[i]->x, element->path[i]->y, dir).used_net.find(element->net_id);

		--(find_result->second);	// delete the routing net

		if (find_result->second == 0)	// if the routing net number becomes zero -> recalculate the congestion cost

		{

			congestionMap2d->edge(element->path[i]->x, element->path[i]->y, dir).used_net.erase(element->net_id);

			--(congestionMap2d->edge(element->path[i]->x, element->path[i]->y, dir).cur_cap);

			if (used_cost_flag != FASTROUTE_COST)

			{

				pre_evaluate_congestion_cost_fp(element->path[i]->x, element->path[i]->y, dir);
			}
		}
	}
}

void update_congestion_map_remove_multipin_net(Two_pin_list_2d *list)

{

	for (vector<Two_pin_element_2d *>::iterator it = list->begin(); it != list->end(); ++it)

	{

		update_congestion_map_remove_two_pin_net(*it);
	}
}

void edge_shifting(Tree *t);

// generate the congestion map by Flute with wirelength driven mode.

int coord2Dto1D(int x, int y)
{

	return x + y * rr_map->get_gridx();
}

std::vector<int> coord1Dto2D(int coord)

{
	return std::vector<int>{coord % rr_map->get_gridx(), coord / rr_map->get_gridx()};
}

void gen_FR_congestion_map()

{

	Tree *flutetree; // a struct, defined by Flute library

	Two_pin_element_2d *L_path, *two_pin;

	int *flute_order;

	bboxRouteStateMap = new EdgeColorMap<int>(

		rr_map->get_gridx(),

		rr_map->get_gridy(),

		-1);

	init_2d_map(); // initial congestion map: calculating every edge's capacity

	init_2pin_list(); // initial 2-pin net container

	init_flute(); // initial the information of pin's coordinte and group by net for flute

	flute_mode = NORMAL; // wirelength driven	mode

	/*assign 0.7 demand to each net*/

#ifdef MESSAGE

	printf("bbox routing start...\n");

#endif

	// for storing the RSMT which retruned by flute

	Flute netRoutingTreeRouter;

	flutetree = (Tree *)calloc(rr_map->get_netNumber(), sizeof(Tree));

	// Get every net's possible RSMT by flute, then use it to calculate the possible congestion

	// In this section, we won't get a real routing result, but a possible congestion information.
	// std::set<tuple<int, int, int>> pinCheck;
	// std::set<tuple<int, int>> pinCheck2D, pinCheck2D_flute;
	for (int i = 0; i < rr_map->get_netNumber(); ++i) // i:net id

	{

#ifdef DEBUG_BBOX

		printf("bbox route net %d start...pin_num=%d\n", i, rr_map->get_netPinNumber(i));

#endif

		// call flute to gen steiner tree and put the result in flutetree[]

		netRoutingTreeRouter.routeNet(rr_map->get_nPin(i), flutetree[i]);
		
		// if (i == 892) {
		// 	for (auto &pin : rr_map->get_nPin(i)) {
		// 		pinCheck.insert({pin->get_tileX(), pin->get_tileY(), pin->get_layerId()});
		// 		pinCheck2D.insert({pin->get_tileX(), pin->get_tileY()});
		// 	}
		// 	std::cout << "Net " << rr_map->get_net_name(i) <<  " has " << pinCheck.size() << " pins" << std::endl;
		// }

		// The total node # in a tree, thoes nodes include pin and steinor point

		// And it is defined as ((2 * degree of a tree) - 2) by the authors of flute

		flutetree[i].number = 2 * flutetree[i].deg - 2; // add 0403

		/*2-pin bounding box assign demand 0.7, remember not to repeat the same net*/

		for (int j = 0; j < flutetree[i].number; ++j) // for all pins and steiner points

		{

			int x1 = (int)flutetree[i].branch[j].x;

			int y1 = (int)flutetree[i].branch[j].y;

			int x2 = (int)flutetree[i].branch[flutetree[i].branch[j].n].x;

			int y2 = (int)flutetree[i].branch[flutetree[i].branch[j].n].y;

			if (!(x1 == x2 && y1 == y2)) // start and end are not the same point

			{

				two_pin = new Two_pin_element_2d();

				two_pin->pin1.x = x1;

				two_pin->pin1.y = y1;

				two_pin->pin2.x = x2;

				two_pin->pin2.y = y2;

				two_pin->net_id = i;

				bbox_2pin_list[i]->push_back(two_pin);

				// if (i == 892) {
				// 	if (pinCheck2D.count({x1, y1})) {
				// 		pinCheck2D_flute.insert({x1, y1});
				// 	}
				// 	if (pinCheck2D.count({x2, y2})) {
				// 		pinCheck2D_flute.insert({x2, y2});

				// 	}
				// }
			}
		}
		// if (i == 892) {
		// 	std::cout << pinCheck2D_flute.size() << ' ' << pinCheck2D.size() << '\n';
		// }
#ifndef GARY
		bbox_route(bbox_2pin_list[i], 0.7);
#endif
	}
#ifdef GARY
	std::vector<std::vector<double>> rudyMap(rr_map->get_gridx(), std::vector<double>(rr_map->get_gridy(), 0.0));
	std::vector<std::vector<double>> rudyMapV(rr_map->get_gridx(), std::vector<double>(rr_map->get_gridy(), 0.0));
	std::vector<std::vector<double>> rudyMapH(rr_map->get_gridx(), std::vector<double>(rr_map->get_gridy(), 0.0));
	std::vector<std::vector<double>> rudyMapNonZeroPinV(rr_map->get_gridx(), std::vector<double>(rr_map->get_gridy(), 0.0));
	std::vector<std::vector<double>> rudyMapNonZeroPinH(rr_map->get_gridx(), std::vector<double>(rr_map->get_gridy(), 0.0));
	std::vector<std::vector<double>> pinDensity(rr_map->get_gridx(), std::vector<double>(rr_map->get_gridy(), 0.0));
	std::vector<std::vector<double>> pinDensityNonZeroPin(rr_map->get_gridx(), std::vector<double>(rr_map->get_gridy(), 0.0));

	int netNum = 0;

	double congSum = 0.0;
	double congSumV = 0.0;
	double congSumH = 0.0;
	double congSumNonZeroV = 0.0;
	double congSumNonZeroH = 0.0;
	std::unordered_map<int, int> pinMap;
	for (int i = 0; i < rr_map->get_netNumber(); ++i)

	{

		auto pinList = rr_map->get_nPin(i);

		for (auto &pin : pinList)
		{

			pinDensity[pin->get_tileX()][pin->get_tileY()]++;
			if (pin->get_layerId() > 0)
			{
				pinMap[coord2Dto1D(pin->get_tileX(), pin->get_tileY())] = pin->get_layerId();
				pinDensityNonZeroPin[pin->get_tileX()][pin->get_tileY()]++;
			}
		}
	}

	for (int x = 0; x < rr_map->get_gridx(); x++)

	{

		for (int y = 0; y < rr_map->get_gridy(); y++)

		{

			int cap = 0;

			if (y < rr_map->get_gridy() - 1)

				cap += congestionMap2d->edge(x, y, Jm::DIR_NORTH).max_cap;

			if (x < rr_map->get_gridx() - 1)

				cap += congestionMap2d->edge(x, y, Jm::DIR_EAST).max_cap;

			if (cap == 0)

				continue;

			pinDensity[x][y] /= cap;
			congSum += pinDensity[x][y];
			congSumV += pinDensity[x][y];
			congSumH += pinDensity[x][y];
			congSumNonZeroV += pinDensityNonZeroPin[x][y];
			congSumNonZeroH += pinDensityNonZeroPin[x][y];
		}
	}
	std::cout << "non zero = " << congSumNonZeroV << " , " << congSumNonZeroH << " ---------------------------------\n";
	for (int i = 0; i < rr_map->get_netNumber(); ++i)

	{

		for (auto &twoPin : (*bbox_2pin_list[i]))

		{

			int x1 = std::min(twoPin->pin1.x, twoPin->pin2.x);

			int x2 = std::max(twoPin->pin1.x, twoPin->pin2.x);

			int y1 = std::min(twoPin->pin1.y, twoPin->pin2.y);

			int y2 = std::max(twoPin->pin1.y, twoPin->pin2.y);

			double xedge = x2 - x1 + 1;

			double yedge = y2 - y1 + 1;

			double rudy = (xedge + yedge) / (xedge * yedge);

			double rudyV = yedge / (xedge * yedge);
			double rudyH = xedge / (xedge * yedge);
			double rudyNonZeroV = 0.0;
			double rudyNonZeroH = 0.0;
			if (pinMap[coord2Dto1D(twoPin->pin1.x, twoPin->pin1.y)] || pinMap[coord2Dto1D(twoPin->pin2.x, twoPin->pin2.y)])
			{
				rudyNonZeroV = yedge / (xedge * yedge);
				rudyNonZeroH = xedge / (xedge * yedge);
			}
			for (int x = x1; x <= x2; x++)
			{
				for (int y = y1; y <= y2; y++)
				{
					rudyMapV[x][y] += rudyV;
					rudyMapH[x][y] += rudyH;
					rudyMap[x][y] += rudy;
					congSum += rudy;
					congSumV += rudyV;
					congSumH += rudyH;
					if (pinMap[coord2Dto1D(twoPin->pin1.x, twoPin->pin1.y)] || pinMap[coord2Dto1D(twoPin->pin2.x, twoPin->pin2.y)])
					{
						rudyMapNonZeroPinV[x][y] += rudyNonZeroV;
						rudyMapNonZeroPinH[x][y] += rudyNonZeroH;
						congSumNonZeroV += rudyNonZeroV;
						congSumNonZeroH += rudyNonZeroH;
					}
				}
			}
		}
	}

	double congArea = congSum / (rr_map->get_gridx() * rr_map->get_gridy());
	double congAreaV = congSumV / (rr_map->get_gridx() * rr_map->get_gridy());
	double congAreaH = congSumH / (rr_map->get_gridx() * rr_map->get_gridy());
	double congAreaNonZeroV = congSumNonZeroV / (rr_map->get_gridx() * rr_map->get_gridy());
	double congAreaNonZeroH = congSumNonZeroH / (rr_map->get_gridx() * rr_map->get_gridy());

	std::cout << "non zero = " << congAreaNonZeroV << " , " << congAreaNonZeroH << " ---------------------------------\n";
	std::vector<std::vector<double>> congestedmap;

	congestedmap.resize(rr_map->get_gridx(), std::vector<double>(rr_map->get_gridy(), 0.0));

	// ofstream outPutFile;

	// outPutFile.open("congested.output");

	discountRatio.resize(rr_map->get_gridx(), std::vector<double>(rr_map->get_gridy(), 0.0));
	discountRatioV.resize(rr_map->get_gridx(), std::vector<double>(rr_map->get_gridy(), 0.0));
	discountRatioH.resize(rr_map->get_gridx(), std::vector<double>(rr_map->get_gridy(), 0.0));
	discountRatioNonZeroV.resize(rr_map->get_gridx(), std::vector<double>(rr_map->get_gridy(), 0.0));
	discountRatioNonZeroH.resize(rr_map->get_gridx(), std::vector<double>(rr_map->get_gridy(), 0.0));

	bool is_ispd18_test5_metal5 = (TESTCASE_NAME == "ispd18_test5_metal5");
	double slope = is_ispd18_test5_metal5 ?  0.2 : 0.3; // 0.2 produces DRC-clean result(1573...), 0.25 produces best score but has DRC(1572...)
	// std::cout << "is_ispd18_test5_metal5: " << is_ispd18_test5_metal5 << '\n';
	for (int x = 0; x < rr_map->get_gridx(); x++)
	{
		for (int y = 0; y < rr_map->get_gridy(); y++)
		{
			double cong = pinDensity[x][y] + rudyMap[x][y];
			double congV = pinDensity[x][y] + rudyMapV[x][y];
			double congH = pinDensity[x][y] + rudyMapH[x][y];
			double congNonZeroV = pinDensityNonZeroPin[x][y] + rudyMapNonZeroPinV[x][y];
			double congNonZeroH = pinDensityNonZeroPin[x][y] + rudyMapNonZeroPinH[x][y];

			double rcong = routing_parameter->get_rcong_min() + routing_parameter->get_rcong_diff() / (1 + std::exp((cong - congArea) * slope));
			double rcongV = routing_parameter->get_rcongV_min() + routing_parameter->get_rcongV_diff() / (1 + std::exp((congV - congAreaV) * slope));
			double rcongH = routing_parameter->get_rcongH_min() + routing_parameter->get_rcongH_diff() / (1 + std::exp((congH - congAreaH) * slope));
			double rcongNonZeroV = routing_parameter->get_NonZeroV_min() + routing_parameter->get_NonZeroV_diff() / (1 + std::exp((congNonZeroV - congAreaNonZeroV) * 0.5));
			double rcongNonZeroH = routing_parameter->get_NonZeroH_min() + routing_parameter->get_NonZeroH_diff() / (1 + std::exp((congNonZeroH - congAreaNonZeroH) * 0.5));
			discountRatio[x][y] = rcong;
			discountRatioV[x][y] = rcongV;
			discountRatioH[x][y] = rcongH;
			discountRatioNonZeroV[x][y] = rcongNonZeroV;
			discountRatioNonZeroH[x][y] = rcongNonZeroH;

			// if (y < rr_map->get_gridy() - 1)

			// 	congestionMap2d->edge(x, y, Jm::DIR_NORTH).max_cap -= (congestionMap2d->edge(x, y, Jm::DIR_NORTH).max_cap * (1 - rcong) * 0.5);

			// if (x < rr_map->get_gridx() - 1)

			// 	congestionMap2d->edge(x, y, Jm::DIR_EAST).max_cap -= (congestionMap2d->edge(x, y, Jm::DIR_EAST).max_cap * (1 - rcong) * 0.5);

			// if (y > 0)

			// 	congestionMap2d->edge(x, y - 1, Jm::DIR_NORTH).max_cap -= (congestionMap2d->edge(x, y - 1, Jm::DIR_NORTH).max_cap * (1 - rcong) * 0.5);

			// if (x > 0)

			// 	congestionMap2d->edge(x - 1, y, Jm::DIR_EAST).max_cap -= (congestionMap2d->edge(x - 1, y, Jm::DIR_EAST).max_cap * (1 - rcong) * 0.5);

			// outPutFile << rcong << " ";
		}

		// outPutFile << "\n";
	}

	for (int x=0; x<rr_map->get_gridx()-1; ++x) {
		for (int y=0; y<rr_map->get_gridy(); ++y) {
			double cong = (pinDensity[x][y] + rudyMap[x][y] + pinDensity[x+1][y] + rudyMap[x+1][y])/2;
			double ratio = routing_parameter->get_rcong_min() + routing_parameter->get_rcong_diff() / (1 + std::exp((cong - congArea) * slope));
			congestionMap2d->edge(x, y, Jm::DIR_EAST).max_cap = std::round(congestionMap2d->edge(x, y, Jm::DIR_EAST).max_cap * ratio);
			// congestionMap2d->edge(x, y, Jm::DIR_EAST).max_cap = std::round(congestionMap2d->edge(x, y, Jm::DIR_EAST).max_cap 
			// 															* ((discountRatio[x][y] + discountRatio[x+1][y])/2));
			// congestionMap2d->edge(x, y, Jm::DIR_EAST).max_cap = std::round(congestionMap2d->edge(x, y, Jm::DIR_EAST).max_cap);
			
		}
	}
	for (int x=0; x<rr_map->get_gridx(); ++x) {
		for (int y=0; y<rr_map->get_gridy()-1; ++y) {
			double cong = (pinDensity[x][y] + rudyMap[x][y] + pinDensity[x][y+1] + rudyMap[x][y+1])/2;
			double ratio = routing_parameter->get_rcong_min() + routing_parameter->get_rcong_diff() / (1 + std::exp((cong - congArea) * slope));
			congestionMap2d->edge(x, y, Jm::DIR_NORTH).max_cap = std::round(congestionMap2d->edge(x, y, Jm::DIR_NORTH).max_cap * ratio);
			// congestionMap2d->edge(x, y, Jm::DIR_NORTH).max_cap = std::round(congestionMap2d->edge(x, y, Jm::DIR_NORTH).max_cap 
			// 															* ((discountRatio[x][y] + discountRatio[x][y+1])/2));
			// congestionMap2d->edge(x, y, Jm::DIR_NORTH).max_cap = std::round(congestionMap2d->edge(x, y, Jm::DIR_NORTH).max_cap);
		}
	}

	for (int i = 0; i < rr_map->get_netNumber(); ++i)

	{

		bbox_route(bbox_2pin_list[i], 0.7);
	}
#endif

#ifdef DEBUG1

	printf("bbox routing complete\n");

	print_cap("max");

	print_cap("cur");

#endif

#ifdef MESSAGE

	printf("L-shaped pattern routing start...\n");

#endif

	// sort net by their bounding box size, then by their pin number
    pattern_start = std::chrono::high_resolution_clock::now();

	vector<const Net *> sort_net;

	for (int i = 0; i < rr_map->get_netNumber(); ++i)

	{

		sort_net.push_back(&rr_map->get_netList()[i]);
	}

	sort(sort_net.begin(), sort_net.end(), comp_net);

	// Now begins the initial routing by pattern routing

	// Edge shifting will also be applyed to the routing.
		// std::set<tuple<int, int>> pinCheckL;

	// bool do_edge_shifting = (TESTCASE_NAME != "ispd18_test5_metal5" && TESTCASE_NAME != "ispd19_test9_metal5"); // This is for Best Score Settings;
	bool do_edge_shifting = (TESTCASE_NAME != "ispd18_test5_metal5" && TESTCASE_NAME != "ispd19_test9_metal5" && TESTCASE_NAME != "ispd19_test7_metal5" && TESTCASE_NAME != "ispd19_test8_metal5"); // This is for DRC-clean settings
	
	
	for (vector<const Net *>::iterator it = sort_net.begin();

		 it != sort_net.end();

		 ++it)

	{

		int netId = (*it)->id;

		flute_order = (int *)malloc(sizeof(int) * (2 * flutetree[netId].deg - 2));

		if (do_edge_shifting) 
			edge_shifting(&flutetree[netId]);												  
			

		global_flutetree = flutetree[netId];

		setup_flute_order(flute_order);

		net_flutetree[netId] = flutetree[netId];

		/*remove demand*/

		bbox_route(bbox_2pin_list[netId], -0.7);
		for (int k = 0; k < flutetree[netId].number; ++k)

		{

			int j = flute_order[k];

			int x1 = (int)flutetree[netId].branch[j].x;

			int y1 = (int)flutetree[netId].branch[j].y;

			int x2 = (int)flutetree[netId].branch[flutetree[netId].branch[j].n].x;

			int y2 = (int)flutetree[netId].branch[flutetree[netId].branch[j].n].y;

			if (!(x1 == x2 && y1 == y2))

			{
				// if (netId == 892) {
				// 	if (pinCheck2D.count({x1, y1}))
				// 		pinCheckL.insert({x1, y1});
				// 	if (pinCheck2D.count({x2, y2}))
				// 		pinCheckL.insert({x2, y2});
				// }

				/*choose the L-shpae with lower congestion to assing new demand 1*/

				L_path = new Two_pin_element_2d();

				L_pattern_route(x1, y1, x2, y2, L_path, netId);

				/*insert 2pin_path into this net*/

				net_2pin_list[netId]->push_back(L_path);

				update_congestion_map_insert_two_pin_net(L_path);

#ifdef DEBUG_LROUTE

				print_path(*L_path);

#endif
			}
		}

#ifdef FREE

		free(flute_order);

#endif
	}
    pattern_end = std::chrono::high_resolution_clock::now();

		// std::cout << "After L-route " <<  pinCheckL.size() << ' ' << pinCheck2D.size() << '\n';


#ifdef MESSAGE

	printf("generate L-shape congestion map in stage1 successfully\n");

#endif

#ifdef DEBUG1

	print_cap("cur");

#endif

	cal_max_overflow();

	delete bboxRouteStateMap;
}

//=====================edge shifting=============================

// Return the smaller cost of L-shape path or the only one cost of flap path

double compute_L_pattern_cost(int x1, int y1, int x2, int y2, int net_id)

{

	Two_pin_element_2d path1, path2;

	Monotonic_element max_cong_path1, max_cong_path2;

	if (x1 > x2)

	{

		swap(x1, x2);

		swap(y1, y2);
	}

	if (x1 < x2 && y1 < y2)

	{

		max_cong_path1 = L_pattern_max_cong(x1, y1, x2, y2, FRONT, RIGHT, &path1, net_id);

		max_cong_path2 = L_pattern_max_cong(x1, y1, x2, y2, RIGHT, FRONT, &path2, net_id);

		if ((&max_cong_path1) == (compare_cost(&max_cong_path1, &max_cong_path2)))

			return max_cong_path1.total_cost;

		else

			return max_cong_path2.total_cost;
	}

	else if (x1 < x2 && y1 > y2)

	{

		max_cong_path1 = L_pattern_max_cong(x1, y1, x2, y2, BACK, RIGHT, &path1, net_id);

		max_cong_path2 = L_pattern_max_cong(x1, y1, x2, y2, RIGHT, BACK, &path2, net_id);

		if ((&max_cong_path1) == (compare_cost(&max_cong_path1, &max_cong_path2)))

			return max_cong_path1.total_cost;

		else

			return max_cong_path2.total_cost;
	}

	else // vertical or horizontal line

	{

		if (y1 > y2)

		{

			swap(y1, y2);
		}

		max_cong_path1 = L_pattern_max_cong(x1, y1, x2, y2, FRONT, RIGHT, &path1, net_id);

		return max_cong_path1.total_cost;
	}
}

void find_saferange(Vertex_flute_ptr a, Vertex_flute_ptr b, int *low, int *high, int dir)

{

	Vertex_flute_ptr cur, find;

	// Horizontal edge doing vertical shifting

	if (dir == HOR)

	{

		cur = a;

		while (cur->type != PIN)

		{

			find = *(cur->neighbor.begin());

			for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1;

				 nei != cur->neighbor.end();

				 ++nei)

			{

				if ((*nei)->y > find->y)

				{

					find = *nei;
				}
			}

			cur = find;

			if ((cur->y == a->y) || (cur->x != a->x))

				break;
		}

		*high = min(*high, cur->y);

		cur = b;

		while (cur->type != PIN)

		{

			find = *(cur->neighbor.begin());

			for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)

				if ((*nei)->y > find->y)

					find = *nei;

			cur = find;

			if ((cur->y == b->y) || (cur->x != b->x))

				break;
		}

		*high = min(*high, cur->y);

		cur = a;

		while (cur->type != PIN)

		{

			find = *(cur->neighbor.begin());

			for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)

				if ((*nei)->y < find->y)

					find = *nei;

			cur = find;

			if (cur->y == a->y || cur->x != a->x)

				break;
		}

		*low = max(*low, cur->y);

		cur = b;

		while (cur->type != PIN)

		{

			find = *(cur->neighbor.begin());

			for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)

				if ((*nei)->y < find->y)

					find = *nei;

			cur = find;

			if ((cur->y == b->y) || (cur->x != b->x))

				break;
		}

		*low = max(*low, cur->y);
	}

	else

	{

		cur = a;

		while (cur->type != PIN)

		{

			find = *(cur->neighbor.begin());

			for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)

				if ((*nei)->x > find->x)

					find = *nei;

			cur = find;

			if (cur->x == a->x || cur->y != a->y) // no neighboring vertex in the right of a

				break;
		}

		*high = min(*high, cur->x);

		cur = b;

		while (cur->type != PIN)

		{

			find = *(cur->neighbor.begin());

			for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)

				if ((*nei)->x > find->x)

					find = *nei;

			cur = find;

			if (cur->x == b->x || cur->y != b->y) // no neighboring vertex in the right of b

				break;
		}

		*high = min(*high, cur->x);

		cur = a;

		while (cur->type != PIN)

		{

			find = *(cur->neighbor.begin());

			for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)

				if ((*nei)->x < find->x)

					find = *nei;

			cur = find;

			if (cur->x == a->x || cur->y != a->y) // no neighboring vertex in the left of a

				break;
		}

		*low = max(*low, cur->x);

		cur = b;

		while (cur->type != PIN)

		{

			find = *(cur->neighbor.begin());

			for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)

				if ((*nei)->x < find->x)

					find = *nei;

			cur = find;

			if (cur->x == b->x || cur->y != b->y) // no neighboring vertex in the left of b

				break;
		}

		*low = max(*low, cur->x);
	}
}

void merge_vertex(Vertex_flute_ptr keep, Vertex_flute_ptr deleted)

{

	for (vector<Vertex_flute_ptr>::iterator nei = deleted->neighbor.begin(); nei != deleted->neighbor.end(); ++nei)

	{

		if ((*nei) != keep) // nei is not keep itself

		{

			keep->neighbor.push_back(*nei); // add deleted's neighbor to keep

			for (int find = 0;; ++find)

			{

				//#ifdef DEBUG_EDGESHIFT

				if (find == (int)(*nei)->neighbor.size())

				{

					printf("wrong in merge_vertex\n");

					exit(0);
				}

				//#endif

				if ((*nei)->neighbor[find]->x == deleted->x && (*nei)->neighbor[find]->y == deleted->y) // neighbor[find] equals to deleted

				{

					(*nei)->neighbor[find] = keep; // replace its neighbor as keep

					break;
				}
			}
		}
	}

	deleted->type = DELETED;
}

bool move_edge(Vertex_flute_ptr a, Vertex_flute_ptr b, int best_pos, int dir)

{

	vector<Vertex_flute_ptr> st_pt;

	Vertex_flute_ptr cur, find;

	Vertex_flute_ptr overlap_a, overlap_b;

	int ind1, ind2;

	overlap_a = overlap_b = NULL;

	if (dir == HOR)

	{

		if (best_pos > a->y) // move up

		{

			// find all steiner points between a and best_pos

			cur = a;

			while (cur->y < best_pos)

			{

				find = *(cur->neighbor.begin());

				for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)

					if ((*nei)->y > find->y)

						find = *nei;

				cur = find;

				st_pt.push_back(cur);
			}

			// exchange neighb

			for (int i = 0; i < (int)st_pt.size(); ++i)

			{

				if (st_pt[i]->y < best_pos)

				{

					for (ind1 = 0;; ++ind1)

					{

						if (!(((a->neighbor[ind1]->x == b->x) && (a->neighbor[ind1]->y == b->y)) ||

							  ((a->neighbor[ind1]->x == st_pt[i]->x) && (a->neighbor[ind1]->y == st_pt[i]->y))))

						{

							break;
						}
					}

					for (ind2 = 0;; ++ind2)

						if (st_pt[i]->neighbor[ind2]->y > st_pt[i]->y)

							break;

					for (int j = 0;; ++j)

						if (a->neighbor[ind1]->neighbor[j] == a)

						{

							a->neighbor[ind1]->neighbor[j] = st_pt[i];

							break;
						}

					for (int j = 0;; ++j)

						if (st_pt[i]->neighbor[ind2]->neighbor[j] == st_pt[i])

						{

							st_pt[i]->neighbor[ind2]->neighbor[j] = a;

							break;
						}

					swap(a->neighbor[ind1], st_pt[i]->neighbor[ind2]);

					a->y = st_pt[i]->y;
				}

				else if (st_pt[i]->x == a->x && st_pt[i]->y == best_pos)

					overlap_a = st_pt[i];

				else

					break;
			}

			st_pt.clear();

			cur = b;

			while (cur->y < best_pos)

			{

				find = *(cur->neighbor.begin());

				for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)

					if ((*nei)->y > find->y)

						find = *nei;

				cur = find;

				st_pt.push_back(cur);
			}

			for (int i = 0; i < (int)st_pt.size(); ++i)

			{

				if (st_pt[i]->y < best_pos)

				{

					for (ind1 = 0;; ++ind1)

						if (!((b->neighbor[ind1]->x == a->x && b->neighbor[ind1]->y == a->y) ||

							  (b->neighbor[ind1]->x == st_pt[i]->x && b->neighbor[ind1]->y == st_pt[i]->y)))

							break;

					for (ind2 = 0;; ++ind2)

						if (st_pt[i]->neighbor[ind2]->y > st_pt[i]->y)

							break;

					for (int j = 0;; ++j)

						if (b->neighbor[ind1]->neighbor[j] == b)

						{

							b->neighbor[ind1]->neighbor[j] = st_pt[i];

							break;
						}

					for (int j = 0;; ++j)

						if (st_pt[i]->neighbor[ind2]->neighbor[j] == st_pt[i])

						{

							st_pt[i]->neighbor[ind2]->neighbor[j] = b;

							break;
						}

					swap(b->neighbor[ind1], st_pt[i]->neighbor[ind2]);

					b->y = st_pt[i]->y;
				}

				else if (st_pt[i]->x == b->x && st_pt[i]->y == best_pos)

					overlap_b = st_pt[i];

				else

					break;
			}

			st_pt.clear();

			a->y = best_pos;

			b->y = best_pos;
		}

		else // move down

		{

			cur = a;

			while (cur->y > best_pos)

			{

				find = *(cur->neighbor.begin());

				for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)

					if ((*nei)->y < find->y)

						find = *nei;

				cur = find;

				st_pt.push_back(cur);
			}

			for (int i = 0; i < (int)st_pt.size(); ++i)

			{

				if (st_pt[i]->y > best_pos)

				{

					for (ind1 = 0;; ++ind1)

						if (!((a->neighbor[ind1]->x == b->x && a->neighbor[ind1]->y == b->y) || (a->neighbor[ind1]->x == st_pt[i]->x && a->neighbor[ind1]->y == st_pt[i]->y)))

							break;

					for (ind2 = 0;; ++ind2)

						if (st_pt[i]->neighbor[ind2]->y < st_pt[i]->y)

							break;

					for (int j = 0;; ++j)

						if (a->neighbor[ind1]->neighbor[j] == a)

						{

							a->neighbor[ind1]->neighbor[j] = st_pt[i];

							break;
						}

					for (int j = 0;; ++j)

						if (st_pt[i]->neighbor[ind2]->neighbor[j] == st_pt[i])

						{

							st_pt[i]->neighbor[ind2]->neighbor[j] = a;

							break;
						}

					swap(a->neighbor[ind1], st_pt[i]->neighbor[ind2]);

					a->y = st_pt[i]->y;
				}

				else if (st_pt[i]->x == a->x && st_pt[i]->y == best_pos)

					overlap_a = st_pt[i];

				else

					break;
			}

			st_pt.clear();

			cur = b;

			while (cur->y > best_pos)

			{

				find = *(cur->neighbor.begin());

				for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)

					if ((*nei)->y < find->y)

						find = *nei;

				cur = find;

				st_pt.push_back(cur);
			}

			for (int i = 0; i < (int)st_pt.size(); ++i)

			{

				if (st_pt[i]->y > best_pos)

				{

					for (ind1 = 0;; ++ind1)

						if (!((b->neighbor[ind1]->x == a->x && b->neighbor[ind1]->y == a->y) || (b->neighbor[ind1]->x == st_pt[i]->x && b->neighbor[ind1]->y == st_pt[i]->y)))

							break;

					for (ind2 = 0;; ++ind2)

						if (st_pt[i]->neighbor[ind2]->y < st_pt[i]->y)

							break;

					for (int j = 0;; ++j)

						if (b->neighbor[ind1]->neighbor[j] == b)

						{

							b->neighbor[ind1]->neighbor[j] = st_pt[i];

							break;
						}

					for (int j = 0;; ++j)

						if (st_pt[i]->neighbor[ind2]->neighbor[j] == st_pt[i])

						{

							st_pt[i]->neighbor[ind2]->neighbor[j] = b;

							break;
						}

					swap(b->neighbor[ind1], st_pt[i]->neighbor[ind2]);

					b->y = st_pt[i]->y;
				}

				else if (st_pt[i]->x == b->x && st_pt[i]->y == best_pos)

					overlap_b = st_pt[i];

				else

					break;
			}

			st_pt.clear();

			a->y = best_pos;

			b->y = best_pos;
		}
	}

	else // VER

	{

		if (best_pos > a->x) // move right

		{

			// find all steiner points between a and best_pos

			cur = a;

			while (cur->x < best_pos)

			{

				find = *(cur->neighbor.begin());

				for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)

					if ((*nei)->x > find->x)

						find = *nei;

				cur = find;

				st_pt.push_back(cur);
			}

			// exchange neighbor

			for (int i = 0; i < (int)st_pt.size(); ++i)

			{

				if (st_pt[i]->x < best_pos)

				{

					for (ind1 = 0;; ++ind1)

						if (!((a->neighbor[ind1]->x == b->x && a->neighbor[ind1]->y == b->y) || (a->neighbor[ind1]->x == st_pt[i]->x && a->neighbor[ind1]->y == st_pt[i]->y)))

							break;

					for (ind2 = 0;; ++ind2)

						if (st_pt[i]->neighbor[ind2]->x > st_pt[i]->x)

							break;

					for (int j = 0;; ++j)

						if (a->neighbor[ind1]->neighbor[j] == a)

						{

							a->neighbor[ind1]->neighbor[j] = st_pt[i];

							break;
						}

					for (int j = 0;; ++j)

						if (st_pt[i]->neighbor[ind2]->neighbor[j] == st_pt[i])

						{

							st_pt[i]->neighbor[ind2]->neighbor[j] = a;

							break;
						}

					swap(a->neighbor[ind1], st_pt[i]->neighbor[ind2]);

					a->x = st_pt[i]->x;
				}

				else if (st_pt[i]->x == best_pos && st_pt[i]->y == a->y)

					overlap_a = st_pt[i];

				else

					break;
			}

			st_pt.clear();

			cur = b;

			while (cur->x < best_pos)

			{

				find = *(cur->neighbor.begin());

				for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)

					if ((*nei)->x > find->x)

						find = *nei;

				cur = find;

				st_pt.push_back(cur);
			}

			// exchange neighbor

			for (int i = 0; i < (int)st_pt.size(); ++i)

			{

				if (st_pt[i]->x < best_pos)

				{

					for (ind1 = 0;; ++ind1)

						if (!((b->neighbor[ind1]->x == a->x && b->neighbor[ind1]->y == a->y) || (b->neighbor[ind1]->x == st_pt[i]->x && b->neighbor[ind1]->y == st_pt[i]->y)))

							break;

					for (ind2 = 0;; ++ind2)

						if (st_pt[i]->neighbor[ind2]->x > st_pt[i]->x)

							break;

					for (int j = 0;; ++j)

						if (b->neighbor[ind1]->neighbor[j] == b)

						{

							b->neighbor[ind1]->neighbor[j] = st_pt[i];

							break;
						}

					for (int j = 0;; ++j)

						if (st_pt[i]->neighbor[ind2]->neighbor[j] == st_pt[i])

						{

							st_pt[i]->neighbor[ind2]->neighbor[j] = b;

							break;
						}

					swap(b->neighbor[ind1], st_pt[i]->neighbor[ind2]);

					b->x = st_pt[i]->x;
				}

				else if (st_pt[i]->x == best_pos && st_pt[i]->y == b->y)

					overlap_b = st_pt[i];

				else

					break;
			}

			st_pt.clear();

			a->x = best_pos;

			b->x = best_pos;
		}

		else // move left

		{

			cur = a;

			while (cur->x > best_pos)

			{

				find = *(cur->neighbor.begin());

				for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)

					if ((*nei)->x < find->x)

						find = *nei;

				cur = find;

				st_pt.push_back(cur);
			}

			for (int i = 0; i < (int)st_pt.size(); ++i)

			{

				if (st_pt[i]->x > best_pos)

				{

					for (ind1 = 0;; ++ind1)

						if (!((a->neighbor[ind1]->x == b->x && a->neighbor[ind1]->y == b->y) || (a->neighbor[ind1]->x == st_pt[i]->x && a->neighbor[ind1]->y == st_pt[i]->y)))

							break;

					for (ind2 = 0;; ++ind2)

						if (st_pt[i]->neighbor[ind2]->x < st_pt[i]->x)

							break;

					for (int j = 0;; ++j)

						if (a->neighbor[ind1]->neighbor[j] == a)

						{

							a->neighbor[ind1]->neighbor[j] = st_pt[i];

							break;
						}

					for (int j = 0;; ++j)

						if (st_pt[i]->neighbor[ind2]->neighbor[j] == st_pt[i])

						{

							st_pt[i]->neighbor[ind2]->neighbor[j] = a;

							break;
						}

					swap(a->neighbor[ind1], st_pt[i]->neighbor[ind2]);

					a->x = st_pt[i]->x;
				}

				else if (st_pt[i]->x == best_pos && st_pt[i]->y == a->y)

					overlap_a = st_pt[i];

				else

					break;
			}

			st_pt.clear();

			cur = b;

			while (cur->x > best_pos)

			{

				find = *(cur->neighbor.begin());

				for (vector<Vertex_flute_ptr>::iterator nei = cur->neighbor.begin() + 1; nei != cur->neighbor.end(); ++nei)

					if ((*nei)->x < find->x)

						find = *nei;

				cur = find;

				st_pt.push_back(cur);
			}

			for (int i = 0; i < (int)st_pt.size(); ++i)

			{

				if (st_pt[i]->x > best_pos)

				{

					for (ind1 = 0;; ++ind1)

						if (!((b->neighbor[ind1]->x == a->x && b->neighbor[ind1]->y == a->y) || (b->neighbor[ind1]->x == st_pt[i]->x && b->neighbor[ind1]->y == st_pt[i]->y)))

							break;

					for (ind2 = 0;; ++ind2)

						if (st_pt[i]->neighbor[ind2]->x < st_pt[i]->x)

							break;

					for (int j = 0;; ++j)

						if (b->neighbor[ind1]->neighbor[j] == b)

						{

							b->neighbor[ind1]->neighbor[j] = st_pt[i];

							break;
						}

					for (int j = 0;; ++j)

						if (st_pt[i]->neighbor[ind2]->neighbor[j] == st_pt[i])

						{

							st_pt[i]->neighbor[ind2]->neighbor[j] = b;

							break;
						}

					swap(b->neighbor[ind1], st_pt[i]->neighbor[ind2]);

					b->x = st_pt[i]->x;
				}

				else if (st_pt[i]->x == best_pos && st_pt[i]->y == b->y)

					overlap_b = st_pt[i];

				else

					break;
			}

			st_pt.clear();

			a->x = best_pos;

			b->x = best_pos;
		}
	}

	bool ret = false;

	if (overlap_a != NULL)

	{

		merge_vertex(overlap_a, a);

		ret = true;
	}

	if (overlap_b != NULL)

		merge_vertex(overlap_b, b);

	return ret;
}

void traverse_tree(double *ori_cost)

{

	double cur_cost, tmp_cost, best_cost;

	int best_pos = 0;

	Vertex_flute_ptr node;

	for (vector<Vertex_flute_ptr>::iterator it_v = vertex_fl.begin();

		 it_v != vertex_fl.end();

		 ++it_v)

	{

		node = *it_v;

		node->visit = 1;

		if (node->type == STEINER && node->neighbor.size() <= 3) // remove3!!!

		{

			for (vector<Vertex_flute_ptr>::iterator it = node->neighbor.begin(); it != node->neighbor.end(); ++it)

				if ((*it)->visit == 0 && ((*it)->type == STEINER && (*it)->neighbor.size() <= 3)) //!!!remove3!!

				{

					int low = 0, high = INT_MAX;

					Vertex_flute_ptr a = node, b = (*it);

					if ((*it)->y == node->y) // horizontal edge (vertical shifting)

					{

						find_saferange(a, b, &low, &high, HOR); // compute safe range

						best_cost = *ori_cost;

						cur_cost = (*ori_cost) - compute_L_pattern_cost(a->x, a->y, b->x, b->y, -1);

						for (int pos = low; pos <= high; ++pos)

						{

							tmp_cost = cur_cost + compute_L_pattern_cost(a->x, pos, b->x, pos, -1);

							if (tmp_cost < best_cost)

							{

								best_cost = tmp_cost;

								best_pos = pos;
							}
						}

						if (best_cost < *ori_cost) // edge need shifting

						{

							move_edge(a, b, best_pos, HOR);

							// move to best position,exchange steiner points if needed

							*ori_cost = best_cost;
						}
					}

					else if ((*it)->x == node->x) // vertical edge (horizontal shifting)

					{

						find_saferange(a, b, &low, &high, VER); // compute safe range

						best_cost = *ori_cost;

						cur_cost = (*ori_cost) - compute_L_pattern_cost(a->x, a->y, b->x, b->y, -1);

						for (int pos = low; pos <= high; ++pos)

						{

							tmp_cost = cur_cost + compute_L_pattern_cost(pos, a->y, pos, b->y, -1);

							if (tmp_cost < best_cost)

							{

								best_cost = tmp_cost;

								best_pos = pos;
							}
						}

						if (best_cost < *ori_cost) // edge need shifting

						{

							move_edge(a, b, best_pos, VER);

							// move to best position,exchange steiner points if needed

							*ori_cost = best_cost;
						}
					}

					else

						continue;
				}
		}
	}
}

void dfs_output_tree(Vertex_flute_ptr node, Tree *t)

{

	node->visit = 1;

	t->branch[t->number].x = node->x;

	t->branch[t->number].y = node->y;

	node->index = t->number;

	(t->number) += 1;

	for (vector<Vertex_flute_ptr>::iterator it = node->neighbor.begin();

		 it != node->neighbor.end();

		 ++it)

	{

		if (((*it)->visit == 0) && ((*it)->type != DELETED))

		{

			dfs_output_tree((*it), t); // keep tracing deeper vertice

			t->branch[(*it)->index].n = node->index; // make parsent of target vertex point to current vertex
		}
	}
}

void edge_shifting(Tree *t)

{

	Vertex_flute_ptr new_v;

	double ori_cost; // the original cost without edge shifting

	ori_cost = 0;

	// creat vertex

	for (int i = 0; i < t->deg; ++i)

	{

		new_v = new Vertex_flute((int)t->branch[i].x, (int)t->branch[i].y);

		new_v->type = PIN;

		vertex_fl.push_back(new_v);
	}

	for (int i = t->deg; i < 2 * t->deg - 2; ++i)

	{

		new_v = new Vertex_flute((int)t->branch[i].x, (int)t->branch[i].y);

		new_v->type = STEINER;

		vertex_fl.push_back(new_v);
	}

	// creat edge

	for (int i = 0; i < 2 * (t->deg) - 2; ++i)

	{

		// skip the vertex if it is the same vertex with its neighbor

		if ((vertex_fl[i]->x == vertex_fl[t->branch[i].n]->x) && (vertex_fl[i]->y == vertex_fl[t->branch[i].n]->y))

			continue;

		vertex_fl[i]->neighbor.push_back(vertex_fl[t->branch[i].n]);

		vertex_fl[t->branch[i].n]->neighbor.push_back(vertex_fl[i]);

		// compute original tree cost

		ori_cost += compute_L_pattern_cost(vertex_fl[i]->x, vertex_fl[i]->y, vertex_fl[t->branch[i].n]->x, vertex_fl[t->branch[i].n]->y, -1);
	}

	sort(vertex_fl.begin(), vertex_fl.end(), comp_vertex_fl);

	for (int i = 0, j = 1; j < 2 * (t->deg) - 2; ++j)
	{
		if ((vertex_fl[i]->x == vertex_fl[j]->x) && (vertex_fl[i]->y == vertex_fl[j]->y)) // j is redundant
		{
			vertex_fl[j]->type = DELETED;
			for (vector<Vertex_flute_ptr>::iterator it = vertex_fl[j]->neighbor.begin();
				 it != vertex_fl[j]->neighbor.end();
				 ++it)
			{
				if (((*it)->x != vertex_fl[i]->x) || ((*it)->y != vertex_fl[i]->y)) // not i,add 0430
				{
					vertex_fl[i]->neighbor.push_back(*it);
					for (int k = 0; k < (int)(*it)->neighbor.size(); k++)
					{
						if ((*it)->neighbor[k]->x == vertex_fl[i]->x && (*it)->neighbor[k]->y == vertex_fl[i]->y)
						{
							(*it)->neighbor[k] = vertex_fl[i]; // modify (*it)'s neighbor to i
							break;
						}
					}
				}
			}
		}
		else
			i = j;
	}

	for (int i = 0; i < 2 * t->deg - 2; ++i)
		vertex_fl[i]->visit = 0;
	traverse_tree(&ori_cost); // dfs to find 2 adjacent steiner points(h or v edge) and do edge_shifhting

	// Output the result (2-pin lists) to a Tree structure in DFS order
	// 1. make sure every 2-pin list have not been visited
	for (int i = 0; i < 2 * t->deg - 2; ++i)
	{
		vertex_fl[i]->visit = 0;
	}
	t->number = 0;

	// 2. begin to out put the 2-pin lists to a Tree strucuture
	dfs_output_tree(vertex_fl[0], t);
	t->branch[0].n = 0; // because neighbor of root is not assign in dfs_output_tree()

	// 3. free memory resources
	for (int i = 0; i < (int)vertex_fl.size(); ++i)
		delete (vertex_fl[i]);
	vertex_fl.clear();
}
//=====================end edge shifting=============================

/* sort by bounding box size */
void output_2_pin_list()
{
	sort(all_two_pin_list.begin(), all_two_pin_list.end(), comp_2pin_net);
	sort(two_pin_list.begin(), two_pin_list.end(), comp_2pin_net_from_path);
}

/*used for stage2
	return the maximum overflow=cur_cap/max_cap
	update 0307:only assign max_cap to cur_3d_map
*/
void output_3d_map()
{
	int i, j, k;

	// assign max_cap to cur_3d_map
	for (i = 0; i < rr_map->get_gridx() - 1; ++i)
		for (j = 0; j < rr_map->get_gridy(); ++j)
			for (k = 0; k < rr_map->get_layerNumber(); ++k)
			{
				cur_map_3d[i][j][k].edge_list[RIGHT]->max_cap = rr_map->capacity(k, i, j, i + 1, j);
			}
	for (i = 0; i < rr_map->get_gridx(); ++i)
		for (j = 0; j < rr_map->get_gridy() - 1; ++j)
			for (k = 0; k < rr_map->get_layerNumber(); ++k)
			{
				cur_map_3d[i][j][k].edge_list[FRONT]->max_cap = rr_map->capacity(k, i, j, i, j + 1);
			}
#ifdef GARY
	int x_size = rr_map->get_gridx();
    int y_size = rr_map->get_gridy();
    int layer_size = rr_map->get_layerNumber();

	std::vector<std::vector<int>> tile_cap_map_x;
    std::vector<std::vector<int>> tile_cap_map_y;
    tile_cap_map_x.resize(x_size, std::vector<int>(y_size, 0));
    tile_cap_map_y.resize(x_size, std::vector<int>(y_size, 0));
    // calculate 2d tile capacity
    for (int i = x_size - 2; i >= 0; --i)
    {
        for (int j = y_size - 1; j >= 0; --j)
        {

            int i_j_2d_cap_x = std::round(congestionMap2d->edge(i, j, Jm::DIR_EAST).cur_cap);
            tile_cap_map_x[i][j] += i_j_2d_cap_x;
            tile_cap_map_x[i + 1][j] += i_j_2d_cap_x;
        }
    }

    for (int i = x_size - 1; i >= 0; --i)
    {
        for (int j = y_size - 2; j >= 0; --j)
        {

            int i_j_2d_cap_y = std::round(congestionMap2d->edge(i, j, Jm::DIR_NORTH).cur_cap);
            tile_cap_map_y[i][j] += i_j_2d_cap_y;
            tile_cap_map_y[i][j + 1] += i_j_2d_cap_y;
        }
    }

    // caculate 3d tile capacity and demand prediction maps
    for (int i = 0; i < x_size; ++i)
    {
        for (int j = 0; j < y_size; ++j)
        {
            int demand_x = tile_cap_map_x[i][j];
            int demand_y = tile_cap_map_y[i][j];
            for (int k = layer_size - 1; k > 0; --k)
            {
                if (database.getLayerDir(k) == Y)
                {
                    int targetLayer = (database.getLayerDir(1) == Y) ? 1 : 2;
                    int targetLayerM = (database.getLayerDir(3) == Y) ? 3 : 4;
                    if (i > 0 && i < x_size - 1)
                    {
                        cur_map_3d[i][j][k].cap = (cur_map_3d[i][j][k].edge_list[RIGHT]->max_cap + cur_map_3d[i][j][k].edge_list[LEFT]->max_cap);
                    }
                    else if (i == 0)
                    {
                        cur_map_3d[i][j][k].cap = cur_map_3d[i][j][k].edge_list[RIGHT]->max_cap;
                    }
                    else
                    {
                        cur_map_3d[i][j][k].cap = cur_map_3d[i][j][k].edge_list[LEFT]->max_cap;
                    }

                    if (k < 3) // 3 for metal5 testcase 5 for normal
                    {
                        if (cur_map_3d[i][j][targetLayer].discountRatio <= 0.6)
                        {
                            cur_map_3d[i][j][k].cap *= 0.7; // 0.85
                        }
                    }
                    else if (k >= 3)
                    {
                        if (cur_map_3d[i][j][targetLayerM].discountRatio <= 0.65)
                        {
                            cur_map_3d[i][j][k].cap *= 0.8;
                        }
                    }
					assert(sign(cur_map_3d[i][j][k].cap) >= 0);
                    cur_map_3d[i][j][k].max_cap = cur_map_3d[i][j][k].cap;
                    if (cur_map_3d[i][j][k].cap <= demand_x)
                    {
                        cur_map_3d[i][j][k].dem_pred = cur_map_3d[i][j][k].cap;
                        demand_x -= cur_map_3d[i][j][k].cap;
                    }
                    else if (k != 1)
                    {
                        cur_map_3d[i][j][k].dem_pred = demand_x;
                        demand_x = 0;
                    }
                }
                else
                {
                    int targetLayer = (database.getLayerDir(1) == X) ? 1 : 2;
                    int targetLayerM = (database.getLayerDir(3) == X) ? 3 : 4;
                    if (j > 0 && j < y_size - 1)
                    {
                        cur_map_3d[i][j][k].cap = (cur_map_3d[i][j][k].edge_list[FRONT]->max_cap + cur_map_3d[i][j][k].edge_list[BACK]->max_cap);
                    }
                    else if (j == 0)
                    {
                        cur_map_3d[i][j][k].cap = cur_map_3d[i][j][k].edge_list[FRONT]->max_cap;
                    }
                    else
                    {
                        cur_map_3d[i][j][k].cap = cur_map_3d[i][j][k].edge_list[BACK]->max_cap;
                    }

                    if (k < 3)  // 3 for metal5 testcase 5 for normal
                    {
                        if (cur_map_3d[i][j][targetLayer].discountRatio <= 0.6)
                        {
                            cur_map_3d[i][j][k].cap *= 0.7; //0.85
                        }
                    }
                    else if (k >= 3)
                    {
                        if (cur_map_3d[i][j][targetLayerM].discountRatio <= 0.65)
                        {
                            cur_map_3d[i][j][k].cap *= 0.8;
                        }
                    }
					assert(sign(cur_map_3d[i][j][k].cap) >= 0);
                    cur_map_3d[i][j][k].max_cap = cur_map_3d[i][j][k].cap;
                    if (cur_map_3d[i][j][k].cap <= demand_y)
                    {
                        cur_map_3d[i][j][k].dem_pred = cur_map_3d[i][j][k].cap;
                        demand_y -= cur_map_3d[i][j][k].cap;
                    }
                    else
                    {
                        cur_map_3d[i][j][k].dem_pred = demand_y;
                        demand_y = 0;
                    }
                }
            }
        }
    }

    auto &deletedNet = rr_map->get_deleted_netList();
    for (auto &net : deletedNet)
    {
        auto &pinList = net.get_pinList();
        auto maxLayer = rr_map->get_layerNumber();
        for (auto &pin : pinList)
        {
            int x = pin->get_tileX();
            int y = pin->get_tileY();
            int layer = pin->get_layerId();
            if (layer != 0) {
                cur_map_3d[x][y][layer].cap -= 2;
				cur_map_3d[x][y][layer].cap = max(0.0, cur_map_3d[x][y][layer].cap);
			}
			if (layer+1 < layer_size) {
            	cur_map_3d[x][y][layer + 1].cap -= 2;
				cur_map_3d[x][y][layer+1].cap = max(0.0, cur_map_3d[x][y][layer+1].cap);
			}
			if (layer+2 < layer_size) {
            	cur_map_3d[x][y][layer + 2].cap -= 2;
				cur_map_3d[x][y][layer+2].cap = max(0.0, cur_map_3d[x][y][layer+2].cap);
			}
        }
    }

	for (int j=0; j<y_size; ++j) {
		for (int k=0; k<layer_size; ++k) {
			if (database.getLayerDir(k) == Y) {
				cur_map_3d[0][j][k].edge_list[RIGHT]->max_cap = std::round(cur_map_3d[0][j][k].cap);
				cur_map_3d[x_size-2][j][k].edge_list[RIGHT]->max_cap = std::round(cur_map_3d[x_size-1][j][k].cap);
				assert(cur_map_3d[0][j][k].edge_list[RIGHT]->max_cap >= 0);
				assert(cur_map_3d[x_size-2][j][k].edge_list[RIGHT]->max_cap >= 0);
			} else {
				cur_map_3d[0][j][k].edge_list[RIGHT]->max_cap = cur_map_3d[x_size-2][j][k].edge_list[RIGHT]->max_cap = 0;
			}
		}
	}
	for (int i=0; i<x_size; ++i) {
		for (int k=0; k<layer_size; ++k) {
			if (database.getLayerDir(k) == X) {
				cur_map_3d[i][0][k].edge_list[FRONT]->max_cap = std::round(cur_map_3d[i][0][k].cap);
				cur_map_3d[i][y_size-2][k].edge_list[FRONT]->max_cap = std::round(cur_map_3d[i][y_size-1][k].cap);
				assert(cur_map_3d[i][0][k].edge_list[FRONT]->max_cap >= 0);
				assert(cur_map_3d[i][y_size-2][k].edge_list[FRONT]->max_cap >= 0);
			} else {
				cur_map_3d[i][0][k].edge_list[FRONT]->max_cap = cur_map_3d[i][y_size-2][k].edge_list[FRONT]->max_cap = 0;
			}
		}
	}

	for (int i=1; i<x_size-2; ++i) {
		for (int j=0; j<y_size; ++j) {
			for (int k=0; k<layer_size; ++k) {
				if (database.getLayerDir(k) == Y) {
					cur_map_3d[i][j][k].edge_list[RIGHT]->max_cap = max(0, (int)std::round(cur_map_3d[i][j][k].cap) - cur_map_3d[i-1][j][k].edge_list[RIGHT]->max_cap);
				} else {
					cur_map_3d[i][j][k].edge_list[RIGHT]->max_cap = 0;
				}
			}
		}
	}
	for (int i=0; i<x_size; ++i) {
		for (int j=1; j<y_size-2; ++j) {
			for (int k=0; k<layer_size; ++k) {
				if (database.getLayerDir(k) == X) {
					cur_map_3d[i][j][k].edge_list[FRONT]->max_cap = max(0, (int)std::round(cur_map_3d[i][j][k].cap) - cur_map_3d[i][j-1][k].edge_list[FRONT]->max_cap);
				} else {
					cur_map_3d[i][j][k].edge_list[FRONT]->max_cap = 0;
				}
			}
		}
	}

	for (int i=0; i<x_size-1; ++i) {
		for (int j=0; j<y_size; ++j) {
			double cap = 0;
			for (int k=0; k<layer_size; ++k) {
				cap += cur_map_3d[i][j][k].edge_list[RIGHT]->max_cap;
				if (database.getLayerDir(k) == X) {
					assert(cur_map_3d[i][j][k].edge_list[RIGHT]->max_cap == 0);
				}
			}
			if (sign(cap - std::round(congestionMap2d->edge(i, j, DIR_EAST).max_cap)) < 0) {
				int diff = (int)std::round(std::round(congestionMap2d->edge(i, j, DIR_EAST).max_cap) - cap);
				pair<int, int> quota[layer_size];
				for (int k=0; k<layer_size; ++k) {
					quota[k] = {max(0,
					rr_map->capacity(k, i, j, i + 1, j) - cur_map_3d[i][j][k].edge_list[RIGHT]->max_cap), k};
				}
				std::sort(quota, quota+layer_size, std::greater<pair<int,int>>());

				int id = 0;
				while (diff > 0) {
					if (quota[id].first > 0) {
						int k = quota[id].second;
						cur_map_3d[i][j][k].edge_list[RIGHT]->max_cap++;
						cap++;
						diff--;
						quota[id].first--;
					} else {
						// bool run_out = true;
						// for (int k=0; k<layer_size; ++k)
						// 	run_out &= (quota[k].first == 0);
						// assert(run_out == false);
						// if (run_out) break;
					}
					id = (id + 1) % layer_size;
				}
			}
			assert(sign(cap - std::round(congestionMap2d->edge(i, j, DIR_EAST).max_cap)) >= 0);
		}
	}
	for (int i=0; i<x_size; ++i) {
		for (int j=0; j<y_size-1; ++j) {
			double cap = 0;
			for (int k=0; k<layer_size; ++k) {
				cap += cur_map_3d[i][j][k].edge_list[FRONT]->max_cap;
				if (database.getLayerDir(k) == Y)
					assert(cur_map_3d[i][j][k].edge_list[FRONT]->max_cap == 0);
			}
			if (sign(cap - std::round(congestionMap2d->edge(i, j, DIR_NORTH).max_cap)) < 0) {
				int diff = (int)std::round(std::round(congestionMap2d->edge(i, j, DIR_NORTH).max_cap) - cap);
				pair<int, int> quota[layer_size];
				for (int k=0; k<layer_size; ++k) {
					quota[k] = {max(0, 
					rr_map->capacity(k, i, j, i, j+1) - cur_map_3d[i][j][k].edge_list[FRONT]->max_cap), k};
				}
				std::sort(quota, quota+layer_size, std::greater<pair<int,int>>());

				int id = 0;
				while (diff > 0) {
					if (quota[id].first > 0) {
						int k = quota[id].second;
						cur_map_3d[i][j][k].edge_list[FRONT]->max_cap++;
						cap++;
						diff--;
						quota[id].first--;
					} else {
						// bool run_out = true;
						// for (int k=0; k<layer_size; ++k)
						// 	run_out &= (quota[k].first == 0);
						// assert(run_out == false);
						// if (run_out) break;
					}
					id = (id + 1) % layer_size;
				}
			}
			assert(sign(cap - std::round(congestionMap2d->edge(i, j, DIR_NORTH).max_cap)) >= 0);
		}
	}
#endif

#ifdef DEBUG1
	print_cap_3d("max");
	print_cap_3d("cur");
#endif
}

/*stage 1: contruct 2d steiner tree
  output 2-pin list to stage2
  return max_overflow;
*/

vector<bool> NetDirtyBit;
double construct_2d_tree(RoutingRegion *rr)
{
	rr_map = rr; // rr_map: get the pointer point to routing date container

	cur_iter = -1; // current iteration ID.
				   // edgeIterCounter = new EdgeColorMap<int>(rr_map->get_gridx(), rr_map->get_gridy(), -1);

	used_cost_flag = FASTROUTE_COST; // cost function type, i.e., HISTORY_COST, HISTORY_MADEOF_COST, MADEOF_COST, FASTROUTE_COST
	total_overflow = 0;				 // used by post-processing

	readLUT(); // Function in flute, functino: unknown

	/* TroyLee: NetDirtyBit Counter */
	NetDirtyBit = vector<bool>(rr_map->get_netNumber(), true);
	/* TroyLee: End */

	allocate_coor_array(); // Make an 2D coordinate array which contains the (x, y) information

	if (routing_parameter->get_monotonic_en())
	{
		allocate_monotonic(); // Allocate the memory for storing the data while searching monotonic path
							  // 1. A 2D array that stores max congestion
							  // 2. A 2D array that stores parent (x,y) during finding monotonic path
	}

	gen_FR_congestion_map(); // Generate congestion map by flute, then route all nets by L-shap pattern routing with
							 // congestion information from this map. After that, apply edge shifting to the result
							 // to get the initial solution.

	cal_total_wirelength(); // The report value is the sum of demand on every edge

    main_start = std::chrono::high_resolution_clock::now();

	allocate_gridcell(); // question: I don't know what this for. (jalamorm, 07/10/31)

	// Make a 2-pin net list without group by net
	for (int i = 0; i < rr_map->get_netNumber(); ++i)
	{
		for (int j = 0; j < (int)net_2pin_list[i]->size(); ++j)
		{
			two_pin_list.push_back((*net_2pin_list[i])[j]);
		}
	}

	reallocate_two_pin_list(true);

	cache = new EdgePlane<CacheEdge>(rr_map->get_gridx(), rr_map->get_gridy(), CacheEdge());
	mazeroute_in_range = new Multisource_multisink_mazeroute();

	// int pre_overflow = -1;
	int cur_overflow = -1;
	used_cost_flag = HISTORY_COST;
	BOXSIZE_INC = routing_parameter->get_init_box_size_p2();

	for (cur_iter = 1, done_iter = cur_iter;
		 cur_iter <= routing_parameter->get_iteration_p2();
		 ++cur_iter, done_iter = cur_iter) // do n-1 times
	{
		cout << "\033[31mIteration:\033[m " << cur_iter << endl;

		factor = (1.0 - exp(-5 * exp(-(0.1 * cur_iter))));

		WL_Cost = factor;
		via_cost = static_cast<int>(4 * factor);
		adjust_value = cur_iter * (1.25 + 3 * factor); // tuned for experimant

#ifdef MESSAGE
		cout << "Parameters - Factor: " << factor
			 << ", Via_Cost: " << via_cost
			 << ", Box Size: " << BOXSIZE_INC + cur_iter - 1 << endl;
#endif

		pre_evaluate_congestion_cost();

		// route_all_2pin_net(false);
		route_all_2pin_net();

		// pre_overflow = cur_overflow;
		cur_overflow = cal_max_overflow();
		// cal_total_wirelength();

		if (cur_overflow == 0)
			break;

		reallocate_two_pin_list(true);

#ifdef MESSAGE
		printMemoryUsage("Memory Usage:");
#endif

		if (cur_overflow <= routing_parameter->get_overflow_threshold())
		{
			break;
		}
		BOXSIZE_INC += routing_parameter->get_box_size_inc_p2();
	}

	output_2_pin_list(); // order:bbox�p
	
    main_end = std::chrono::high_resolution_clock::now();
	// std::cout << "Max turns = " << maxTurns << "\n";
    



#ifdef FREE
	free_memory_con2d();
#endif
#ifdef MESSAGE
	cout << "================================================================" << endl;
	cout << "===                   Enter Post Processing                  ===" << endl;
	cout << "================================================================" << endl;
#endif
/*
	unordered_map<int, bool> through_of_edge_nets_and_through_zero;
	// ofstream fout("grid.txt");
	int fuck_edge_cnt = 0;
	for (int i=0; i<two_pin_list.size(); ++i) {
		for(int j=two_pin_list[i]->path.size()-2; j >= 0; --j)
		{
			int dir = get_direction_2d(two_pin_list[i]->path[j], two_pin_list[i]->path[j+1]);
			DirectionType dirType = static_cast<DirectionType>(Jr2JmDirArray[dir]);
			int x = two_pin_list[i]->path[j]->x;
			int y = two_pin_list[i]->path[j]->y;
			// if (two_pin_list[i]->net_id == 892) {
			// 	// std::cout << "(" << x << ", " << y << "), dir: " << dir << " dem: " << congestionMap2d->edge(x, y, dirType).cur_cap << ", cap: " << congestionMap2d->edge(x, y, dirType).max_cap << '\n'; 
			// 	fout << x << " " << y << "\n"; 
			// }
			if (congestionMap2d->edge(x, y, dirType).isOverflow()) {
				if (sign(congestionMap2d->edge(x, y, dirType).max_cap) == 0) {
					// std::cout << "Fucking edge: " << x << ' ' << y << ' ' << dir << ' ' << congestionMap2d->edge(x, y, dirType).cur_cap << ' ' << congestionMap2d->edge(x, y, dirType).max_cap << '\n';
					fuck_edge_cnt++;
					through_of_edge_nets_and_through_zero[two_pin_list[i]->net_id] = true;
					for (int k=0; k<rr_map->get_layerNumber(); ++k) {
						if (dir == LEFT)
							assert(rr_map->capacity(k, x, y, x-1, y) == 0);
						if (dir == RIGHT)
							assert(rr_map->capacity(k, x, y, x+1, y) == 0);
						if (dir == FRONT)
							assert(rr_map->capacity(k, x, y, x, y+1) == 0);
						if (dir == BACK)
							assert(rr_map->capacity(k, x, y, x, y-1) == 0);
					}
					// break;
				} else {
					if (!through_of_edge_nets_and_through_zero[two_pin_list[i]->net_id])
						through_of_edge_nets_and_through_zero[two_pin_list[i]->net_id] = false;
				}
			}
		}
		
	}
	// fout.close();
	std::cout << "Fuck edge cnt = " << fuck_edge_cnt << '\n';
	std::cout << "#OF nets:" << through_of_edge_nets_and_through_zero.size() << '\n';
	// for (auto &[net_id, through_zero] : through_of_edge_nets_and_through_zero) {
	// 	std::cout << "net id: " << net_id << ' ' << "through_zero: " << through_zero << '\n';
	// }
*/
	return 0;
}

#include "Post_processing.cpp"