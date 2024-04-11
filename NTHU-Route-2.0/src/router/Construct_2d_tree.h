#ifndef _CONSTRUCT_2D_TREE_H_

#define _CONSTRUCT_2D_TREE_H_

#include "parameter.h"

#include "grdb/RoutingRegion.h"

#include "grdb/plane.h"

#include "misc/geometry.h"

#include "flute/flute-ds.h"

#include "util/traversemap.h"

#include "misc/geometry.h"

#include "gr_db/GrDatabase.h"

#include "db/Database.h"

#include <iostream>

#include <fstream>

#include <vector>

#include <set>

#include <map>

#include <string>

#include <unordered_map>

using std::vector;

using std::set;

using std::map;

// If wanna run IBM testcase, please enable this define

//#define IBM_CASE

#define FREE

//#define MESSAGE

#define parameter_h 0.8 // used in the edge cost function 1/0.5 0.8/2

#define parameter_k 2 // used in the edge cost function

enum
{
    FRONT,
    BACK,
    LEFT,
    RIGHT,
    UP,
    DOWN
};

enum
{
    DIR_X,
    DIR_Y
};

enum
{
    PIN,
    STEINER,
    DELETED
};

enum
{
    HOR,
    VER
};

/* Cost function Prototype */

extern void (*pre_evaluate_congestion_cost_fp)(int i, int j, int dir);

void pre_evaluate_congestion_cost_all(int i, int j, int dir);


// store the information of a 2-pin net

class Two_pin_element_2d
{
public:
    Two_pin_element_2d()
    : net_id(0), done(-1), path_through_zero_edge(false)
    {
    }

public:
    Jm::Coordinate_2d pin1;

    Jm::Coordinate_2d pin2;

    vector<Jm::Coordinate_2d *> path;

    bool path_through_zero_edge;

    int net_id;

    int8_t done;

    int boxSize() const
    {
        return abs(pin1.x - pin2.x) + abs(pin1.y - pin2.y);
    }
};

class Monotonic_element
{

public:
    double max_cost;

    double total_cost; // record the acculmated congestion of the monotonic path

    int net_cost;

    int distance;

    int via_num;
};

class Two_pin_element
{

public:
    Jm::Coordinate_3d pin1;

    Jm::Coordinate_3d pin2;

    vector<Jm::Coordinate_3d *> path;

    int net_id;

    double max_cong;

    Two_pin_element(){};
};

class Vertex_flute
{
public:
    int x, y;
    int type; // PIN, SETINER, DELETED
    int index;
    int visit;
    vector<Vertex_flute *> neighbor;

public:
    Vertex_flute(int x = 0, int y = 0): x(x), y(y), visit(0) /*, copy_ind(-1)*/
    {
    }
};

typedef std::unordered_map<int, int> RoutedNetTable;

class Edge_2d
{
public:
    Edge_2d();
public:
    double cur_cap;
    //2024/04/10 ying revise
    //double max_cap;
    int max_cap;

    int history;

    RoutedNetTable used_net;

    int sign(double x) {
        const double EPS = 1e-5;
        if (fabs(x) < EPS)
            return 0;
        if (x + EPS < 0)
            return -1;
        return 1;
    }

    bool isOverflow() { 
        return sign(cur_cap-max_cap) > 0;
        // return (cur_cap > max_cap); 
    }

    bool isFull() { 
        return sign(cur_cap-max_cap) >= 0;
        // return (cur_cap >= max_cap); 
    }

    int overUsage() { 
        return static_cast<int>(std::round(cur_cap - max_cap));
        // return static_cast<int>(cur_cap - max_cap); 
    }

    bool lookupNet(int netId) { return (used_net.find(netId) != used_net.end()); }

    double congestion() { 
        if (sign(max_cap) == 0) {
            return sign(cur_cap) == 0 ? 0 : 2.0;
        }
        return std::round(cur_cap)/std::round(max_cap);
        // if (sign(cur_cap) == 0 && sign(max_cap) == 0) return 0;
        // return sign(max_cap) == 0 ? 2.0 : std::round(cur_cap)/std::round(max_cap);
        // return (cur_cap / max_cap); 
    }
};

typedef std::unordered_map<int, int> LRoutedNetTable;

class Edge_3d
{
public:
    Edge_3d();
public:
    int max_cap;
    int cur_cap;
    int cur_dem = 0;
    LRoutedNetTable used_net;
};

typedef Edge_2d *Edge_2d_ptr;

typedef Edge_3d *Edge_3d_ptr;

typedef vector<Two_pin_element_2d *> Two_pin_list_2d;

typedef Vertex_flute *Vertex_flute_ptr;

class Vertex_3d
{

public:
    Edge_3d_ptr edge_list[6]; // FRONT,BACK,LEFT,RIGHT,UP,DOWN

    int dem_pred = 0;

    double cap = 0.0;

    double max_cap = 0.0;

    double discountRatio = 0.0;
};

struct CacheEdge
{

    double cost; // Used as cache of cost in whole program

    int MMVisitFlag; // Used as cache of hash table lookup result in MM_mazeroute

    CacheEdge()

        : cost(0.0),

          MMVisitFlag(-1)

    {
    }
};

extern int par_ind;

extern ParameterSet *parameter_set;

extern RoutingParameters *routing_parameter;

extern const int dir_array[4][2]; // FRONT,BACK,LEFT,RIGHT

extern const int Jr2JmDirArray[4];

extern EdgePlane<Edge_2d> *congestionMap2d;

extern vector<Two_pin_element_2d *> two_pin_list;

extern int two_pin_list_size;

extern int flute_mode;

extern Monotonic_element **cong_monotonic; // store max congestion during monotonic path

extern int **parent_monotonic; // record parent (x,y) during finding monotonic path

extern Jm::Coordinate_2d **coor_array;

extern int via_cost;

extern double max_congestion_factor;

extern int fail_find_path_count;

extern Vertex_3d ***cur_map_3d;

extern vector<Two_pin_element *> all_two_pin_list;

extern RoutingRegion *rr_map;

extern int cur_iter;

extern int done_iter;

extern double alpha;

extern int total_overflow;

extern int used_cost_flag;

extern int BOXSIZE_INC;

extern Tree *net_flutetree;

extern EdgePlane<CacheEdge> *cache;

#include "MM_mazeroute.h"

extern Multisource_multisink_mazeroute *mazeroute_in_range;

extern void update_congestion_map_insert_two_pin_net(Two_pin_element_2d *element);

extern void update_congestion_map_remove_two_pin_net(Two_pin_element_2d *element);

extern bool monotonic_pattern_route(int x1, int y1,

                                    int x2, int y2,

                                    Two_pin_element_2d *two_pin_monotonic_path,

                                    int net_id,

                                    double bound_cost,

                                    int bound_distance,

                                    int bound_via_num,

                                    bool bound_flag);

extern void insert_all_two_pin_list(Two_pin_element_2d *mn_path_2d);

extern double get_cost_2d(int i, int j, int dir, int net_id, int *distance);

extern bool smaller_than_lower_bound(double total_cost,

                                     int distance,

                                     int via_num,

                                     double bound_cost,

                                     int bound_distance,

                                     int bound_via_num);

inline bool comp_stn_2pin(const Two_pin_element_2d *a, const Two_pin_element_2d *b)

{
    // if (a->path_through_zero_edge || b->path_through_zero_edge) cout << "ok\n";
    if (a->path_through_zero_edge && !b->path_through_zero_edge) return true;
    if (!a->path_through_zero_edge && b->path_through_zero_edge) return false;
    return (a->boxSize() > b->boxSize());
    // return (a->boxSize() < b->boxSize());
}

inline int get_direction_2d_simple(const Jm::Coordinate_2d *a,

                                   const Jm::Coordinate_2d *b)

{

    assert(!(a->x != b->x && a->y != b->y));

    if (a->x != b->x)
        return LEFT;

    else
        return FRONT;
}

inline int get_direction_2d(const Jm::Coordinate_2d *a, const Jm::Coordinate_2d *b)

{

    assert(!(a->x != b->x && a->y != b->y));

    if (a->x != b->x)
    {

        if (a->x > b->x)
            return LEFT;

        else
            return RIGHT;
    }
    else
    {

        if (a->y > b->y)
            return BACK;

        else
            return FRONT;
    }
}

inline

void printMemoryUsage()
{
    // for print out memory usage

    std::ifstream mem("/proc/self/status");

    std::string memory;
    int i = 0;
    while(getline(mem, memory)){
        if(i == 17) std::cout << i << ": " << memory << std::endl;
        if(i == 27) std::cout << i << ": " << memory << std::endl;
        ++i;
    }
}

extern vector<bool> NetDirtyBit;

#endif /* _CONSTRUCT_2D_TREE_H_ */
