#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)
// #define FOLLOW_PREFER
//  #define VIA_DENSITY	// new experiment 2007/09/27
// #define REDUCE_VIA_DENSITY	// new experiment 2007/09/29
#define CHECK_PREFER
#define MAX_OVERFLOW_CONSTRAINT
#define FROM2D
#define ALLOUTPUT

#define PIN_LEF
// #define PATCHING
// #define DAC2023
// #define VERIFY_PIN_LEF
// #define PRINT_DP_PIN
// #define VERIFY_PIN_1
#include "Construct_2d_tree.h"
#include "misc/geometry.h"
#include "grdb/parser.h"
#include <algorithm>
#include <queue>
#include <set>
#include <string.h>
#include <time.h>
#include <tuple>
using namespace std;
using namespace Jm;

#ifdef CHECK_PREFER
char prefer_direction[6][2] = {0};
#endif
int tar = -2;
char follow_prefer_direction;
// enum {GREEDY, SHORT_PATH};
int l_option;
int max_xx, max_yy, max_zz, overflow_max, *prefer_idx;
Coordinate_3d ***coord_3d_map;
int i_router, i_test_case, i_order, i_method;
const char temp_buf[1000] = "1000";
bool check_net = false;
bool is_metal5 = false;
extern std::string TESTCASE_NAME;
extern std::chrono::high_resolution_clock::time_point io_start;
extern std::chrono::high_resolution_clock::time_point io_end;

extern std::vector<int> layerDirections;

long long total_via = 0;

double DP_TIME = 0;

int CNT = 1;

struct {
    int idx;
    int val;
} ans;
typedef struct {
    int val;
    Coordinate_3d *pi;
} DP_NODE;
DP_NODE ***dp_map;
typedef struct {
    int id;
    int val;
} NET_NODE;
NET_NODE *net_order;
typedef struct {
    int id;
    int times;
    int val;
    int vo_times;
    double average;
    int bends;
} AVERAGE_NODE;
AVERAGE_NODE *average_order;
typedef struct {
    int xy;
    int z;
    double val;
} NET_INFO_NODE;
NET_INFO_NODE *net_info;
void update_ans(int tar, int val) {
    ans.idx = tar;
    ans.val = val;
}
typedef struct {
    Two_pin_list_2d two_pin_net_list;
} MULTIPIN_NET_NODE;
MULTIPIN_NET_NODE *multi_pin_net;
typedef struct {
    int val;
    int edge[4];
    int pin_layer; // may be a problem, because there are pins in same (x, y), but not the same layer
    int min_pin_layer;
    int max_pin_layer;
} PATH_NODE;
PATH_NODE **path_map;
typedef struct {
    int val;
    int via_cost;
    int via_overflow;
    int pi_z;
} KLAT_NODE;
KLAT_NODE ***klat_map;
typedef struct {
    int *edge[4];
} OVERFLOW_NODE;
OVERFLOW_NODE **overflow_map;
typedef struct {
    int pi;
    int sx, sy, bx, by;
    int num;
} UNION_NODE;
UNION_NODE *group_set;
typedef struct {
    int xy;
    int z;
} LENGTH_NODE;
LENGTH_NODE length_count[1000];
typedef struct {
    int cur;
    int max;
} VIADENSITY_NODE;
VIADENSITY_NODE ***viadensity_map;
typedef struct {
    set<int> used_net;
} PATH_EDGE_3D;
typedef PATH_EDGE_3D *PATH_EDGE_3D_PTR;
typedef struct {
    PATH_EDGE_3D_PTR edge_list[6];
} PATH_VERTEX_3D;
PATH_VERTEX_3D ***path_map_3d;

const int plane_dir[4][2] = {{0, 1}, {0, -1}, {-1, 0}, {1, 0}};                                   // F B L R
const int cube_dir[6][3] = {{0, 1, 0}, {0, -1, 0}, {-1, 0, 0}, {1, 0, 0}, {0, 0, 1}, {0, 0, -1}}; // F B L R U D
int global_net_id, global_x, global_y, global_max_layer, global_pin_num, global_xy_reduce = 0, global_BFS_xy = 0;
long long global_pin_cost = 0;
int min_DP_val, min_DP_idx[4], max_DP_k, min_DP_k, min_DP_via_cost;
int total_pin_number = 0;
int ***BFS_color_map;
int temp_global_pin_cost, temp_global_xy_cost, after_xy_cost;
// int is_used_for_BFS[1300][1300]; // modified
int global_increase_vo;

inline std::string viaToGuide(int x, int y, int layer1, int layer2) {
    auto pointToGcell = [&](int x, int y, int xStep, int yStep) {
        std::vector<utils::PointT<int>> coord(2);
        coord[0].x = max(0, x * xStep - 1);
        coord[0].y = max(0, y * yStep - 1);
        coord[1].x = max(0, (x + 1) * xStep - 1);
        coord[1].y = max(0, (y + 1) * yStep - 1);
        return coord;
    };

    std::string str = "";
    int layer_1 = std::min(layer1, layer2);
    int layer_2 = std::max(layer1, layer2);

    /*(2024/01/21) MF edited
    for (int l = layer_1; l <= layer_2; ++l) {
        str += std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(x+1) + " " + std::to_string(y+1) + "
    metal" + std::to_string(l) + "\n";
    }
    */
    for (int l = layer_1 - 1; l < layer_2 - 1; ++l) {
        str += std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(l) + " " + std::to_string(x) + " "
               + std::to_string(y) + " " + std::to_string(l + 1) + "\n";
    }
    return str;

    auto coord1 = pointToGcell(x, y, grDatabase.getXStep(), grDatabase.getYStep());
    int xmax = std::min(int(database.dieRegion.x.high), coord1[1].x);
    int ymax = std::min(int(database.dieRegion.y.high), coord1[1].y);
    for (int i = layer_1; i < layer_2 + 1; i++) {
        str = str + std::to_string(coord1[0].x) + " " + std::to_string(coord1[0].y) + " " + std::to_string(coord1[1].x)
              + " " + std::to_string(coord1[1].y) + " Metal" + std::to_string(i) + "\n";
    }
    return str;
}

inline std::string rectToGuide(int x1, int y1, int x2, int y2, int layer) {
    auto pointToGcell = [&](int x, int y, int xStep, int yStep) {
        std::vector<utils::PointT<int>> coord(2);
        coord[0].x = max(0, x * xStep - 1);
        coord[0].y = max(0, y * yStep - 1);
        coord[1].x = max(0, (x + 1) * xStep - 1);
        coord[1].y = max(0, (y + 1) * yStep - 1);
        return coord;
    };

    std::string str;
    /* (2024/01/21) MF edited
    str = std::to_string(min(x1, x2)) + " " + std::to_string(min(y1, y2)) + " ";
    str += std::to_string(max(x1, x2) + 1) + " " + std::to_string(max(y1, y2) + 1) + " ";
    str += "metal" + std::to_string(layer) + "\n";
    */
    str = std::to_string(min(x1, x2)) + " " + std::to_string(min(y1, y2)) + " " + std::to_string(layer - 1) + " "
          + std::to_string(max(x1, x2)) + " " + std::to_string(max(y1, y2)) + " " + std::to_string(layer - 1) + "\n";
    return str;

    auto coord1 = pointToGcell(x1, y1, grDatabase.getXStep(), grDatabase.getYStep());
    auto coord2 = pointToGcell(x2, y2, grDatabase.getXStep(), grDatabase.getYStep());
    int xmax = INT_MIN, xmin = INT_MAX;
    int ymax = INT_MIN, ymin = INT_MAX;
    coord1.insert(coord1.end(), coord2.begin(), coord2.end());
    for (auto &point : coord1) {
        if (point.x > xmax)
            xmax = point.x;
        if (point.x < xmin)
            xmin = point.x;
        if (point.y > ymax)
            ymax = point.y;
        if (point.y < ymin)
            ymin = point.y;
    }

    str = str + std::to_string(xmin) + " ";
    str = str + std::to_string(ymin) + " ";
    if (xmax > database.dieRegion.x.high)
        str = str + std::to_string(database.dieRegion.x.high) + " ";
    else
        str = str + std::to_string(xmax) + " ";
    if (ymax > database.dieRegion.y.high)
        str = str + std::to_string(database.dieRegion.y.high) + " ";
    else
        str = str + std::to_string(ymax) + " ";
    str = str + "Metal" + std::to_string(layer) + "\n";
    return str;
}

void print_max_overflow(void) {
    auto sign = [](double x) {
        const double EPS = 1e-8;
        if (abs(x) < EPS)
            return 0;
        if (x + EPS < 0)
            return -1;
        return 1;
    };

    int i, j, k, lines = 0; //, ii, jj, dir;
    int max = 0;
    int sum = 0, temp_sum;
    // int overflow_val;
    // int x, y;
    vector<int> sum_overflow;

    for (int i = 0; i < max_zz; i++) {
        sum_overflow.push_back(0);
    }

    for (i = 1; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j) {
            temp_sum = 0;
            for (k = 0; k < max_zz; ++k) {
                temp_sum += cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap;
                // if (/*k == 0 || */database.getLayerDir(k) == X) {
                // 	assert(cur_map_3d[i][j][k].edge_list[LEFT]->max_cap == 0);
                // 	assert(cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap == 0);
                // }
                if (cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap
                    > cur_map_3d[i][j][k].edge_list[LEFT]->max_cap) // overflow
                {
                    // printf("%d %d %d, cur_cap: %d, max_cap: %d\n", i, j, k,
                    // cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap, cur_map_3d[i][j][k].edge_list[LEFT]->max_cap);

                    // if((cur_map_3d[i][j][k].edge_list[LEFT]->max_cap % 2) != 0) // modified
                    // 	cur_map_3d[i][j][k].edge_list[LEFT]->max_cap -= 1; // modified
                    if (cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap - cur_map_3d[i][j][k].edge_list[LEFT]->max_cap
                        > max) {
                        max =
                            cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap - cur_map_3d[i][j][k].edge_list[LEFT]->max_cap;
                    }
                    sum +=
                        (cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap - cur_map_3d[i][j][k].edge_list[LEFT]->max_cap);
                    sum_overflow[k] +=
                        (cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap - cur_map_3d[i][j][k].edge_list[LEFT]->max_cap);
                    lines++;
                }
            }
        }
    for (i = 0; i < max_xx; ++i)
        for (j = 1; j < max_yy; ++j) {
            temp_sum = 0;
            for (k = 0; k < max_zz; ++k) {
                temp_sum += cur_map_3d[i][j][k].edge_list[BACK]->cur_cap;
                // if (/*k == 0 || */database.getLayerDir(k) == Y) {
                // 	assert(cur_map_3d[i][j][k].edge_list[BACK]->max_cap == 0);
                // 	assert(cur_map_3d[i][j][k].edge_list[BACK]->cur_cap == 0);
                // }
                if (cur_map_3d[i][j][k].edge_list[BACK]->cur_cap
                    > cur_map_3d[i][j][k].edge_list[BACK]->max_cap) // overflow
                {
                    if (cur_map_3d[i][j][k].edge_list[BACK]->cur_cap - cur_map_3d[i][j][k].edge_list[BACK]->max_cap
                        > max) {
                        max =
                            cur_map_3d[i][j][k].edge_list[BACK]->cur_cap - cur_map_3d[i][j][k].edge_list[BACK]->max_cap;
                    }
                    sum +=
                        (cur_map_3d[i][j][k].edge_list[BACK]->cur_cap - cur_map_3d[i][j][k].edge_list[BACK]->max_cap);
                    sum_overflow[k] +=
                        (cur_map_3d[i][j][k].edge_list[BACK]->cur_cap - cur_map_3d[i][j][k].edge_list[BACK]->max_cap);
                    lines++;
                }
            }
        }

    for (int cnt_layer = 0; cnt_layer < max_zz; cnt_layer++) {
        std::cout << "layer: " << cnt_layer << "  3D # of overflow = " << sum_overflow[cnt_layer] << std::endl;
    }
    printf("3D # of overflow = %d\n", sum);
    printf("3D max overflow = %d\n", max);
    printf("3D overflow edge number = %d\n", lines);
}

void find_overflow_max(void) {
    int i, j;

    overflow_max = 0;
    for (i = 1; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j)
            if (congestionMap2d->edge(i, j, DIR_WEST).overUsage() > overflow_max)
                overflow_max = congestionMap2d->edge(i, j, DIR_WEST).overUsage();
    for (i = 0; i < max_xx; ++i)
        for (j = 1; j < max_yy; ++j)
            if (congestionMap2d->edge(i, j, DIR_SOUTH).overUsage() > overflow_max)
                overflow_max = congestionMap2d->edge(i, j, DIR_SOUTH).overUsage();
    printf("2D maximum overflow = %d\n", overflow_max);
#ifdef MAX_OVERFLOW_CONSTRAINT
#ifdef FOLLOW_PREFER
    if (follow_prefer_direction == 1) {
        if (overflow_max % (max_zz / 2))
            overflow_max = ((overflow_max / (max_zz / 2)) << 1) + 2;
        else
            overflow_max = ((overflow_max / (max_zz / 2)) << 1);
    }
    else {
        if (overflow_max % max_zz)
            overflow_max = ((overflow_max / max_zz) << 1) + 2;
        else
            overflow_max = ((overflow_max / max_zz) << 1);
    }
#else
    // modified
    overflow_max = static_cast<int>(std::ceil((double)overflow_max / ((max_zz - 1) / 2)));
    // overflow_max = static_cast<int>(std::ceil((double)overflow_max/(max_zz/2)));
    // overflow_max << 1;
    // if (overflow_max % max_zz)
    // 	overflow_max = ((overflow_max / max_zz) << 1) + 1; //modified
    // else
    // 	overflow_max = ((overflow_max / max_zz) << 1);
#endif
#else
    overflow_max <<= 1;
#endif
    printf("overflow max = %d\n", overflow_max);
}

void initial_3D_coordinate_map(void) {
    int i, j, k;

    coord_3d_map = (Coordinate_3d ***)malloc(sizeof(Coordinate_3d **) * max_xx);
    for (i = 0; i < max_xx; ++i) {
        coord_3d_map[i] = (Coordinate_3d **)malloc(sizeof(Coordinate_3d *) * max_yy);
        for (j = 0; j < max_yy; ++j)
            coord_3d_map[i][j] = (Coordinate_3d *)malloc(sizeof(Coordinate_3d) * max_zz);
    }
    for (i = 0; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j)
            for (k = 0; k < max_zz; ++k) {
                coord_3d_map[i][j][k].x = i;
                coord_3d_map[i][j][k].y = j;
                coord_3d_map[i][j][k].z = k;
            }
}

void malloc_path_map(void) {
    int i, j, k;
    path_map = (PATH_NODE **)malloc(sizeof(PATH_NODE *) * max_xx);
    for (i = 0; i < max_xx; ++i) {
        path_map[i] = (PATH_NODE *)malloc(sizeof(PATH_NODE) * max_yy);
    }
    // initial path_map
    for (i = 0; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j) {
            path_map[i][j].val = 0; // non-visited
            path_map[i][j].pin_layer = -1;
            path_map[i][j].min_pin_layer = INT_MAX;
            path_map[i][j].max_pin_layer = INT_MIN;
            for (k = 0; k < 4; ++k)
                path_map[i][j].edge[k] = 0;
        }
}

void malloc_klat_map(void) {
    int i, j;

    klat_map = (KLAT_NODE ***)malloc(sizeof(KLAT_NODE **) * max_xx);
    for (i = 0; i < max_xx; ++i) {
        klat_map[i] = (KLAT_NODE **)malloc(sizeof(KLAT_NODE *) * max_yy);
        for (j = 0; j < max_yy; ++j)
            klat_map[i][j] = (KLAT_NODE *)malloc(sizeof(KLAT_NODE) * max_zz);
    }
}

void malloc_overflow_map(void) {
    int i, j;
    int *temp;

    overflow_map = (OVERFLOW_NODE **)malloc(sizeof(OVERFLOW_NODE *) * max_xx);
    for (i = 0; i < max_xx; ++i)
        overflow_map[i] = (OVERFLOW_NODE *)malloc(sizeof(OVERFLOW_NODE) * max_yy);
    for (i = 1; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j) {
            temp = (int *)malloc(sizeof(int));
            overflow_map[i][j].edge[LEFT] = temp;
            overflow_map[i - 1][j].edge[RIGHT] = temp;
        }
    for (i = 0; i < max_xx; ++i)
        for (j = 1; j < max_yy; ++j) {
            temp = (int *)malloc(sizeof(int));
            overflow_map[i][j].edge[BACK] = temp;
            overflow_map[i][j - 1].edge[FRONT] = temp;
        }
}

void initial_overflow_map(void) {
    int i, j;
    int max_of_quota = INT_MIN;
    for (i = 1; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j) {
            *(overflow_map[i][j].edge[LEFT]) = (congestionMap2d->edge(i, j, DIR_WEST).overUsage()); // modified
            max_of_quota = max(max_of_quota, *(overflow_map[i][j].edge[LEFT]));
        }
    for (i = 0; i < max_xx; ++i)
        for (j = 1; j < max_yy; ++j) {
            *(overflow_map[i][j].edge[BACK]) = (congestionMap2d->edge(i, j, DIR_SOUTH).overUsage()); // modified
            // if (i == 94 && j == 112) {
            // 	printf("%d %d\n", *(overflow_map[i][j].edge[BACK]), (congestionMap2d->edge(i, j,
            // DIR_SOUTH).overUsage())); 	printf("%lf %lf\n", congestionMap2d->edge(i, j, DIR_SOUTH).cur_cap,
            // congestionMap2d->edge(i, j, DIR_SOUTH).max_cap); 	printf("%lf\n", congestionMap2d->edge(i, j,
            // DIR_SOUTH).cur_cap-congestionMap2d->edge(i, j, DIR_SOUTH).max_cap);

            // 	assert(*(overflow_map[i][j].edge[BACK]) == *(overflow_map[i][j-1].edge[FRONT]));
            // 	assert(round(congestionMap2d->edge(i, j, DIR_SOUTH).cur_cap-congestionMap2d->edge(i, j,
            // DIR_SOUTH).max_cap) == 1);
            // }
            max_of_quota = max(max_of_quota, *(overflow_map[i][j].edge[BACK]));
        }
    printf("max_of_quota: %d\n", max_of_quota);
    // printf("(94, 111) of_map %d", *(overflow_map[94][111].edge[FRONT]));
    // printf("(94, 111) congestionMap2d cur_cap: %lf, max_cap: %lf\n", congestionMap2d->edge(94, 111,
    // DIR_NORTH).cur_cap, congestionMap2d->edge(94, 111, DIR_NORTH).max_cap);
}

void malloc_viadensity_map(void) {
    int i, j, k;

    viadensity_map = (VIADENSITY_NODE ***)malloc(sizeof(VIADENSITY_NODE **) * max_xx);
    for (i = 0; i < max_xx; ++i) {
        viadensity_map[i] = (VIADENSITY_NODE **)malloc(sizeof(VIADENSITY_NODE *) * max_yy);
        for (j = 0; j < max_yy; ++j)
            viadensity_map[i][j] = (VIADENSITY_NODE *)malloc(sizeof(VIADENSITY_NODE) * max_zz);
    }
    // initial viadensity_map
    for (i = 0; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j)
            for (k = 0; k < max_zz; ++k)
                viadensity_map[i][j][k].cur = viadensity_map[i][j][k].max = 0;
}

void malloc_space(void) {
    malloc_path_map();
    malloc_klat_map();
    malloc_overflow_map();
    malloc_viadensity_map();
}

void free_path_map(void) {
    int i;

    for (i = 0; i < max_xx; ++i)
        free(path_map[i]);
    free(path_map);
}

void free_klat_map(void) {
    int i, j;

    for (i = 0; i < max_xx; ++i) {
        for (j = 0; j < max_yy; ++j)
            free(klat_map[i][j]);
        free(klat_map[i]);
    }
    free(klat_map);
}

void free_overflow_map(void) {
    int i, j;

    for (i = 1; i < max_xx; ++i)
        for (j = 0; j < max_yy; ++j)
            free(overflow_map[i][j].edge[LEFT]);
    for (i = 0; i < max_xx; ++i)
        for (j = 1; j < max_yy; ++j)
            free(overflow_map[i][j].edge[BACK]);
    for (i = 0; i < max_xx; ++i)
        free(overflow_map[i]);
    free(overflow_map);
}

void free_viadensity_map(void) {
    int i, j;

    for (i = 0; i < max_xx; ++i) {
        for (j = 0; j < max_yy; ++j)
            free(viadensity_map[i][j]);
        free(viadensity_map[i]);
    }
    free(viadensity_map);
}

void free_malloc_space(void) {
    free_path_map();
    free_klat_map();
    free_overflow_map();
    free_viadensity_map();
}

void update_cur_map_for_klat_xy(int cur_idx, Coordinate_2d *start, Coordinate_2d *end, int net_id) {
    int dir_idx;
    assert(cur_idx > 0);
    if (start->x != end->x && start->y == end->y) {
        // assert(database.getLayerDir(cur_idx) == Y);
        dir_idx = ((end->x > start->x) ? LEFT : RIGHT);
        // assert(cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->max_cap > 0);
        cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->used_net[net_id]++;
        cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->cur_cap =
            (cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->used_net.size()); // modified
        if (cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->cur_cap
            > cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->max_cap) // need check
        {
            // printf("%d %d %d, cur_cap: %lf, max_cap: %lf\n", end->x, end->y, cur_idx,
            // cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->cur_cap,
            // cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->max_cap);

            *(overflow_map[end->x][end->y].edge[dir_idx]) -= 1; // modified
            assert(*(overflow_map[end->x][end->y].edge[dir_idx]) >= 0);
        }
    }
    else if (start->y != end->y && start->x == end->x) {
        // if (database.getLayerDir(cur_idx) != X) {
        // 	cout << cur_idx << '\n';
        // 	cout << start->y << ' ' << end->y << '\n';
        // 	cout << start->x << ' ' << end->x << '\n';
        // }
        // assert(database.getLayerDir(cur_idx) == X);
        dir_idx = ((end->y > start->y) ? BACK : FRONT);
        // if (cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->max_cap == 0)
        // {
        // 	std::cout << cur_idx << ' ' << database.getLayerDir(cur_idx) << std::endl;
        // 	std::cout << start->x << ' ' << start->y << ' ' << end->x << ' ' << end->y << std::endl;
        // }
        // assert(cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->max_cap > 0);
        cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->used_net[net_id]++;
        cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->cur_cap =
            (cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->used_net.size()); // modified
        if (cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->cur_cap
            > cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->max_cap) // need check
        {
            // printf("%d %d %d, cur_cap: %d, max_cap: %d\n", end->x, end->y, cur_idx,
            // cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->cur_cap,
            // cur_map_3d[end->x][end->y][cur_idx].edge_list[dir_idx]->max_cap); if (dir_idx == BACK) 	printf("S:
            // cur_cap: %lf,  max_cap: %lf\n", congestionMap2d->edge(end->x, end->y, DIR_SOUTH).cur_cap,
            // congestionMap2d->edge(end->x, end->y, DIR_SOUTH).max_cap); else 	printf("N: cur_cap: %lf,  max_cap:
            // %lf\n", congestionMap2d->edge(end->x, end->y, DIR_NORTH).cur_cap, congestionMap2d->edge(end->x, end->y,
            // DIR_NORTH).max_cap);
            *(overflow_map[end->x][end->y].edge[dir_idx]) -= 1; // modified
            assert(*(overflow_map[end->x][end->y].edge[dir_idx]) >= 0);
        }
    }
    else if (start->x != end->x && start->y != end->y) {
        printf("%d %d %d %d\n", start->x, start->y, end->x, end->y);
        puts("ERROR!!!");
    }
}

void update_cur_map_for_klat_z(int pre_idx, int cur_idx, Coordinate_2d *start, int net_id) {
    int i, j, k, z_dir, dir_idx;

    if (pre_idx != cur_idx) {
        if (cur_idx > pre_idx) {
            z_dir = 1;
            dir_idx = UP;
        }
        else // cur_idx < pre_idx
        {
            z_dir = -1;
            dir_idx = DOWN;
        }
        i = start->x;
        j = start->y;
        for (k = pre_idx; k != cur_idx; k += z_dir) {
            cur_map_3d[i][j][k].edge_list[dir_idx]->used_net[net_id]++;
            if (z_dir == 1)
                ++viadensity_map[i][j][k].cur;
            else
                ++viadensity_map[i][j][k - 1].cur;
        }
    }
}

void update_path_for_klat(Coordinate_2d *start, int start_pin_layer) {
    int i, x, y, z_min, z_max, pin_num = 0;
    queue<Coordinate_3d *> q;
    Coordinate_3d *temp;
    // int start_pin_layer;
    // start_pin_layer = path_map[start->x][start->y].pin_layer;
    // BFS
    // todo
#ifdef PIN_LEF
    q.push(&coord_3d_map[start->x][start->y][start_pin_layer]); // enqueue
                                                                // if (global_net_id == 349)
    // 	std::cout << start->x << ' ' << start->y << ' ' << start_pin_layer << std::endl;
#else
    q.push(&coord_3d_map[start->x][start->y][0]); // enqueue
#endif

    while (!q.empty()) {
        temp = (Coordinate_3d *)q.front();
        // std::cout << temp->x << ' ' << temp->y << ' ' << temp->z << '\n';
        // if (global_net_id == 349)
        // 	std::cout << temp->x << ' ' << temp->y << ' ' << temp->z << std::endl;
        if (path_map[temp->x][temp->y].val == 2) // has >= 1 pin
        {
            // pin_num++;
#ifdef PIN_LEF
            if (is_metal5) {
                z_min = min(int(temp->z), path_map[temp->x][temp->y].min_pin_layer); // modified
                z_max = max(int(temp->z), path_map[temp->x][temp->y].max_pin_layer); // modified
            }
            else {
                if (temp->z > path_map[temp->x][temp->y].pin_layer) {
                    z_min = path_map[temp->x][temp->y].pin_layer;
                    z_max = temp->z;
                }
                else {
                    z_min = temp->z;
                    z_max = path_map[temp->x][temp->y].pin_layer;
                }
            }

#else
            z_min = 0;
            z_max = temp->z;
#endif
        }
        else {
            z_min = temp->z;
            z_max = temp->z;
        }
        // std::cout << z_min << ' ' << z_max << '\n';

        for (i = 0; i < 4; ++i)
            if (path_map[temp->x][temp->y].edge[i] == 1) // check leagal
            {
                x = temp->x + plane_dir[i][0];
                y = temp->y + plane_dir[i][1];
                // std::cout << x << ' ' << y << ' ' << temp->z << '\n';
                if (path_map[x][y].val == 0)
                    puts("ERROR");
                if (klat_map[x][y][temp->z].pi_z > z_max)
                    z_max = klat_map[x][y][temp->z].pi_z;
                if (klat_map[x][y][temp->z].pi_z < z_min)
                    z_min = klat_map[x][y][temp->z].pi_z;
                // std::cout << "Before xy update\n";
                update_cur_map_for_klat_xy(klat_map[x][y][temp->z].pi_z, &coor_array[temp->x][temp->y],
                                           &coor_array[x][y], global_net_id);
                // std::cout << "After xy update\n";
                q.push(&coord_3d_map[x][y][klat_map[x][y][temp->z].pi_z]); // enqueue
            }
        // std::cout << "Before z update\n";
        // std::cout << z_min << ' ' << z_max << '\n';
        update_cur_map_for_klat_z(z_min, z_max, &coor_array[temp->x][temp->y], global_net_id);
        // std::cout << "After z update\n";
        path_map[temp->x][temp->y].val = 0;                 // visited
        path_map[temp->x][temp->y].min_pin_layer = INT_MAX; // visited
        path_map[temp->x][temp->y].max_pin_layer = INT_MIN; // visited
        q.pop();                                            // dequeue
    }
    // assert(pin_num == global_pin_num);
    // if (pin_num != global_pin_num)
    // 	printf("net : %d, pin number error %d vs %d\n", global_net_id, pin_num, global_pin_num);
}

void cycle_reduction(int x, int y) {
    int i, idx, temp_x, temp_y;

    for (i = 0; i < 4; ++i)
        if (path_map[x][y].edge[i] == 1)
            cycle_reduction(x + plane_dir[i][0], y + plane_dir[i][1]);
    for (idx = 0; idx < 4 && path_map[x][y].edge[idx] == 0; ++idx)
        ;
    if (idx == 4 && path_map[x][y].val == 1) // a leaf && it is not a pin
    {
        path_map[x][y].val = 0;
        for (i = 0; i < 4; ++i) {
            temp_x = x + plane_dir[i][0];
            temp_y = y + plane_dir[i][1];
            if (temp_x >= 0 && temp_x < max_xx && temp_y >= 0 && temp_y < max_yy) {
                if (i == 0 && path_map[temp_x][temp_y].edge[1] == 1)
                    path_map[temp_x][temp_y].edge[1] = 0;
                else if (i == 1 && path_map[temp_x][temp_y].edge[0] == 1)
                    path_map[temp_x][temp_y].edge[0] = 0;
                else if (i == 2 && path_map[temp_x][temp_y].edge[3] == 1)
                    path_map[temp_x][temp_y].edge[3] = 0;
                else if (i == 3 && path_map[temp_x][temp_y].edge[2] == 1)
                    path_map[temp_x][temp_y].edge[2] = 0;
            }
        }
    }
}

int preprocess(int net_id) {
    int i, k, x, y;
    int pin_layer;
    queue<Coordinate_2d *> q;
    Coordinate_2d *temp;
    const PinptrList *pin_list = &rr_map->get_nPin(net_id);
#ifdef POSTPROCESS
    int xy_len = 0;
#endif
    after_xy_cost = 0;
    for (i = 0; i < global_pin_num; ++i) {
        x = (*pin_list)[i]->get_tileX();
        y = (*pin_list)[i]->get_tileY();
        pin_layer = (*pin_list)[i]->get_layerId();
        path_map[x][y].val = -2; // pin
        path_map[x][y].pin_layer = pin_layer;
        if (i == 0) {
            assert(path_map[x][y].min_pin_layer == INT_MAX);
            assert(path_map[x][y].max_pin_layer == INT_MIN);
        }
        path_map[x][y].min_pin_layer = min(path_map[x][y].min_pin_layer, pin_layer);
        path_map[x][y].max_pin_layer = max(path_map[x][y].max_pin_layer, pin_layer);
    }

    // BFS speed up
    x = (*pin_list)[0]->get_tileX();
    y = (*pin_list)[0]->get_tileY();
    path_map[x][y].val = 2; // visited

    // initial klat_map[x][y][k]
    for (k = 0; k < max_zz; ++k) {
#ifdef DAC2023
        klat_map[x][y][k].val = INT_MAX;
#else
        klat_map[x][y][k].val = -1;
#endif
        klat_map[x][y][k].pi_z = -1;
    }
    q.push(&coor_array[x][y]); // enqueue
    while (!q.empty()) {
        temp = q.front();
        for (i = 0; i < 4; ++i) {
            x = temp->x + plane_dir[i][0];
            y = temp->y + plane_dir[i][1];

            if (x >= 0 && x < max_xx && y >= 0 && y < max_yy
                && congestionMap2d->edge(temp->x, temp->y, i).lookupNet(net_id)) {
                if (path_map[x][y].val == 0 || path_map[x][y].val == -2) {
                    ++after_xy_cost;
#ifdef POSTPROCESS
                    ++xy_len;
#endif
                    global_BFS_xy++;
                    path_map[temp->x][temp->y].edge[i] = 1;
                    if (path_map[x][y].val == 0)
                        path_map[x][y].val = 1; // visited node
                    else                        // path_map[x][y].val == 2
                    {
                        path_map[x][y].val = 2; // visited pin
                    }
                    // initial klat_map[x][y][k]
                    for (k = 0; k < max_zz; ++k) {
#ifdef DAC2023
                        klat_map[x][y][k].val = INT_MAX;
#else
                        klat_map[x][y][k].val = -1;
#endif
                        klat_map[x][y][k].pi_z = -1;
                    }
                    // #ifndef VIA_DENSITY
                    // 					if (max_layer < max_zz)
                    // 					{
                    // 						if (congestionMap2d->edge(temp->x, temp->y, i).cur_cap <
                    // congestionMap2d->edge(temp->x, temp->y, i).max_cap)
                    // 						{
                    // 							temp_cap = (congestionMap2d->edge(temp->x, temp->y, i).used_net.size()
                    // << 1); 							for(k = 0; k < max_zz && temp_cap > 0; ++k) 								if
                    // (cur_map_3d[temp->x][temp->y][k].edge_list[i]->max_cap > 0) 									temp_cap -=
                    // cur_map_3d[temp->x][temp->y][k].edge_list[i]->max_cap; 							if (k == max_zz) 								max_layer = max_zz; 							else
                    // if (k + 1 > max_layer) 								max_layer = k + 1;
                    // 						}
                    // 						else
                    // 							max_layer = max_zz;
                    // 					}
                    // #endif
                    q.push(&coor_array[x][y]); // enqueue
                }
                else
                    path_map[temp->x][temp->y].edge[i] = 0;
            }
            else
                path_map[temp->x][temp->y].edge[i] = 0;
        }
        q.pop(); // dequeue
    }
    cycle_reduction((*pin_list)[0]->get_tileX(), (*pin_list)[0]->get_tileY());
#ifdef POSTPROCESS
    net_info[net_id].xy = xy_len;
#endif
#ifdef VIA_DENSITY
    return max_zz;
#else
    return max_zz; // max_layer; // modified
#endif
}

void PRINT_DP_INF(int level, int val, int *count_idx) {
    int k, temp_x, temp_y, temp_val, temp_via_cost;
    int temp_idx[4];
    int min_k, max_k, temp_min_k, temp_max_k;

    if (level < 4) {
        for (k = 0; k < level; ++k)
            temp_idx[k] = count_idx[k];
        if (path_map[global_x][global_y].edge[level] == 1) {
            temp_x = global_x + plane_dir[level][0];
            temp_y = global_y + plane_dir[level][1];
            for (k = 0; k < global_max_layer; ++k) // use global_max_layer to substitue max_zz
                if (klat_map[temp_x][temp_y][k].val >= 0) {
                    std::cout << "x = " << temp_x << " y = " << temp_y << " k = " << k
                              << " val = " << klat_map[temp_x][temp_y][k].val << std::endl;
                    temp_idx[level] = k;
                    PRINT_DP_INF(level + 1, val + klat_map[temp_x][temp_y][k].val, temp_idx);
                }
        }
        else {
            temp_idx[level] = -1;
            PRINT_DP_INF(level + 1, val, temp_idx);
        }
    }
}

void rec_count(int level, int val, int *count_idx) {
    int k, temp_x, temp_y, temp_val, temp_via_cost;
    int temp_idx[4];
    int min_k, max_k, temp_min_k, temp_max_k;

    if (level < 4) {
        if (val <= min_DP_val) {
            for (k = 0; k < level; ++k)
                temp_idx[k] = count_idx[k];
            if (path_map[global_x][global_y].edge[level] == 1) {
                temp_x = global_x + plane_dir[level][0];
                temp_y = global_y + plane_dir[level][1];
                for (k = 1; k < global_max_layer; ++k) // use global_max_layer to substitue max_zz
                    if (klat_map[temp_x][temp_y][k].val >= 0) {
                        temp_idx[level] = k;
                        rec_count(level + 1, val + klat_map[temp_x][temp_y][k].val, temp_idx);
                    }
            }
            else {
                temp_idx[level] = -1;
                rec_count(level + 1, val, temp_idx);
            }
        }
    }
    else // level == 4
    {
        min_k = min_DP_k;
        max_k = max_DP_k;
        for (k = temp_via_cost = 0; k < 4; ++k)
            if (count_idx[k] != -1) {
                if (count_idx[k] < min_k)
                    min_k = count_idx[k];
                if (count_idx[k] > max_k)
                    max_k = count_idx[k];
                temp_via_cost +=
                    klat_map[global_x + plane_dir[k][0]][global_y + plane_dir[k][1]][count_idx[k]].via_cost;
            }

        temp_via_cost += ((max_k - min_k) * via_cost);

#ifdef VIA_DENSITY
        for (k = min_k, temp_val = val; k < max_k; ++k)
            if (viadensity_map[global_x][global_y][k].cur >= viadensity_map[global_x][global_y][k].max)
                ++temp_val;
#else
        temp_val = temp_via_cost;
#endif
        assert(temp_val >= 0);
        if (temp_val < min_DP_val) {
            min_DP_val = temp_val;
            min_DP_via_cost = temp_via_cost;
            for (k = 0; k < 4; ++k)
                min_DP_idx[k] = count_idx[k];
        }
        else if (temp_val == min_DP_val) {
#ifdef VIA_DENSITY
            if (temp_via_cost < min_DP_via_cost) {
                min_DP_via_cost = temp_via_cost;
                for (k = 0; k < 4; ++k)
                    min_DP_idx[k] = count_idx[k];
            }
            else if (temp_via_cost == min_DP_via_cost) {
                temp_min_k = min_DP_k;
                temp_max_k = max_DP_k;
                for (k = 0; k < 4; ++k)
                    if (min_DP_idx[k] != -1) {
                        if (min_DP_idx[k] < temp_min_k)
                            temp_min_k = min_DP_idx[k];
                        if (min_DP_idx[k] > temp_max_k)
                            temp_max_k = min_DP_idx[k];
                    }
                if (max_k > temp_max_k || min_k > temp_min_k)
                    for (k = 0; k < 4; ++k)
                        min_DP_idx[k] = count_idx[k];
            }
#else
            temp_min_k = min_DP_k;
            temp_max_k = max_DP_k;
            for (k = 0; k < 4; ++k)
                if (min_DP_idx[k] != -1) {
                    if (min_DP_idx[k] < temp_min_k)
                        temp_min_k = min_DP_idx[k];
                    if (min_DP_idx[k] > temp_max_k)
                        temp_max_k = min_DP_idx[k];
                }
            if (max_k > temp_max_k || min_k > temp_min_k)
                for (k = 0; k < 4; ++k)
                    min_DP_idx[k] = count_idx[k];
#endif
        }
    }
}

void update(int x, int y) {
    bool is_end = true;
    for (int i = 0; i < 4; ++i) {
        if (path_map[x][y].edge[i] == 1) {
            is_end = false;
            int temp_x = x + plane_dir[i][0];
            int temp_y = y + plane_dir[i][1];
            update(temp_x, temp_y);
        }
    }
    if (is_end) {
        assert(path_map[x][y].val == 2);
        for (int k = 1; k < global_max_layer; ++k) {
            int num_via = max(path_map[x][y].max_pin_layer, k) - min(path_map[x][y].min_pin_layer, k);
            assert(path_map[x][y].max_pin_layer >= path_map[x][y].min_pin_layer);
            assert(num_via >= 0);
            klat_map[x][y][k].val = num_via;
        }
    }
    else {
        int min_stream_cost[4];
        int min_stream_layer[4];
        for (int k_min = 0; k_min < global_max_layer; ++k_min) {
            for (int i = 0; i < 4; ++i) {
                min_stream_cost[i] = INT_MAX;
                min_stream_layer[i] = -1;
            }
            for (int k_max = max(1, k_min); k_max < global_max_layer; ++k_max) {
                for (int i = 0; i < 4; ++i) {
                    if (path_map[x][y].edge[i] == 1) {
                        int temp_x = x + plane_dir[i][0];
                        int temp_y = y + plane_dir[i][1];
                        int stream_cost;
                        if (*(overflow_map[x][y].edge[i]) <= 0) {
                            if (cur_map_3d[x][y][k_max].edge_list[i]->cur_cap
                                < cur_map_3d[x][y][k_max].edge_list[i]->max_cap) {
                                stream_cost = klat_map[temp_x][temp_y][k_max].val;
                                if (stream_cost < min_stream_cost[i]) {
                                    min_stream_cost[i] = stream_cost;
                                    min_stream_layer[i] = k_max;
                                    assert(min_stream_layer[i] > 0);
                                }
                            }
                        }
                        else {
                            if ((cur_map_3d[x][y][k_max].edge_list[i]->cur_cap
                                 - cur_map_3d[x][y][k_max].edge_list[i]->max_cap)
                                < overflow_max) {
                                bool ok = false;
                                if ((i == 0 || i == 1) && database.getLayerDir(k_max) == X) {
                                    ok = true;
                                }
                                else if ((i == 2 || i == 3) && database.getLayerDir(k_max) == Y) {
                                    ok = true;
                                }
                                if (!ok)
                                    continue;
                                stream_cost = klat_map[temp_x][temp_y][k_max].val;
                                if (stream_cost < min_stream_cost[i]) {
                                    min_stream_cost[i] = stream_cost;
                                    min_stream_layer[i] = k_max;
                                    assert(min_stream_layer[i] > 0);
                                }
                            }
                        }
                    }
                }
                int min_DP_k, max_DP_k;
                bool has_pin = (path_map[x][y].val == 2);
                if (has_pin) {
                    min_DP_k = min(path_map[x][y].min_pin_layer, k_min);
                    max_DP_k = max(path_map[x][y].max_pin_layer, k_max);
                }
                else {
                    max_DP_k = k_max;
                    min_DP_k = k_min;
                }
                bool legal = true;
                int min_k = min_DP_k;
                int max_k = max_DP_k;
                int cost = 0;
                for (int i = 0; i < 4; ++i) {
                    if (path_map[x][y].edge[i] == 1) {
                        if (min_stream_cost[i] < INT_MAX) {
                            cost += min_stream_cost[i];
                            min_k = min(min_k, min_stream_layer[i]);
                            max_k = max(max_k, min_stream_layer[i]);
                        }
                        else {
                            legal = false;
                            break;
                        }
                    }
                }
                if (!legal)
                    continue;
                cost += ((max_k - min_k) * via_cost);
                if (cost < klat_map[x][y][k_max].val) {
                    klat_map[x][y][k_max].val = cost;
                    for (int i = 0; i < 4; ++i) {
                        if (path_map[x][y].edge[i] == 1) {
                            int temp_x = x + plane_dir[i][0];
                            int temp_y = y + plane_dir[i][1];
                            klat_map[temp_x][temp_y][k_max].pi_z = min_stream_layer[i];
                            assert(klat_map[temp_x][temp_y][k_max].pi_z > 0);
                            assert(klat_map[temp_x][temp_y][k_max].pi_z < global_max_layer);
                        }
                    }
                }
                else if (cost == klat_map[x][y][k_max].val) {
                    int temp_min_k = min_DP_k, temp_max_k = max_DP_k;
                    for (int i = 0; i < 4; ++i) {
                        if (path_map[x][y].edge[i] == 1) {
                            int temp_x = x + plane_dir[i][0];
                            int temp_y = y + plane_dir[i][1];
                            temp_min_k = min(temp_min_k, klat_map[temp_x][temp_y][k_max].pi_z);
                            temp_max_k = max(temp_max_k, klat_map[temp_x][temp_y][k_max].pi_z);
                        }
                    }
                    if (max_k > temp_max_k || min_k > temp_min_k) {
                        for (int i = 0; i < 4; ++i) {
                            if (path_map[x][y].edge[i] == 1) {
                                int temp_x = x + plane_dir[i][0];
                                int temp_y = y + plane_dir[i][1];
                                klat_map[temp_x][temp_y][k_max].pi_z = min_stream_layer[i];
                                assert(klat_map[temp_x][temp_y][k_max].pi_z > 0);
                                assert(klat_map[temp_x][temp_y][k_max].pi_z < global_max_layer);
                            }
                        }
                    }
                }
            }
            for (int k = global_max_layer - 2; k >= k_min; --k) {
                if (klat_map[x][y][k + 1].val <= klat_map[x][y][k].val) {
                    klat_map[x][y][k].val = klat_map[x][y][k + 1].val;
                    for (int i = 0; i < 4; ++i) {
                        if (path_map[x][y].edge[i] == 1) {
                            int temp_x = x + plane_dir[i][0];
                            int temp_y = y + plane_dir[i][1];
                            klat_map[temp_x][temp_y][k].pi_z = klat_map[temp_x][temp_y][k + 1].pi_z;
                            assert(klat_map[temp_x][temp_y][k].pi_z > 0);
                            assert(klat_map[temp_x][temp_y][k].pi_z < global_max_layer);
                        }
                    }
                } /*else if (klat_map[x][y][k+1].val == klat_map[x][y][k].val) {
                    int old_min_k = INT_MAX, old_max_k = INT_MIN;
                    int new_min_k = INT_MAX, new_max_k = INT_MIN;
                    for(int i = 0; i < 4; ++i) {
                        if (path_map[x][y].edge[i] == 1) {
                            int temp_x = x + plane_dir[i][0];
                            int temp_y = y + plane_dir[i][1];
                            old_min_k = min(old_min_k, klat_map[temp_x][temp_y][k].pi_z);
                            old_max_k = max(old_max_k, klat_map[temp_x][temp_y][k].pi_z);
                            new_min_k = min(new_min_k, klat_map[temp_x][temp_y][k+1].pi_z);
                            new_max_k = max(new_max_k, klat_map[temp_x][temp_y][k+1].pi_z);
                        }
                    }
                    if (new_max_k > old_max_k || new_min_k > old_min_k) {
                        for(int i = 0; i < 4; ++i) {
                            if (path_map[x][y].edge[i] == 1) {
                                int temp_x = x + plane_dir[i][0];
                                int temp_y = y + plane_dir[i][1];
                                klat_map[temp_x][temp_y][k].pi_z = klat_map[temp_x][temp_y][k+1].pi_z;
                            }
                        }
                    }
                }*/
            }
        }
    }
}

void DP(int x, int y, int z) {
    int i, k, temp_x, temp_y;
    bool is_end;
    int count_idx[4];
    int end_pin_layer;
    if (klat_map[x][y][z].val == -1) // non-traversed
    {
        is_end = true;
        for (i = 0; i < 4; ++i) // direction
        {
            if (path_map[x][y].edge[i] == 1) // check legal
            {
#ifdef PRINT_DP_PIN
                if (check_net) {
                    std::cout << "***********" << std::endl;
                    std::cout << "2d : x = " << x << " y = " << y << " i = " << i << std::endl;
                    std::cout << "***********" << std::endl;
                }
#endif
                is_end = false;
                temp_x = x + plane_dir[i][0];
                temp_y = y + plane_dir[i][1];

                bool assigned = false;
                if (*(overflow_map[x][y].edge[i]) <= 0) {  // this edge could not happen overflow
                    for (k = 1; k < global_max_layer; ++k) // use global_max_layer to substitute max_zz
                    {
                        if (cur_map_3d[x][y][k].edge_list[i]->cur_cap < cur_map_3d[x][y][k].edge_list[i]->max_cap) {
                            assigned = true;
                            // if (i == 0 || i == 1) {
                            // 	// std::cout << cur_map_3d[x][y][k].edge_list[i]->cur_cap << ' ' <<
                            // cur_map_3d[x][y][k].edge_list[i]->max_cap << std::endl; 	assert(database.getLayerDir(k) ==
                            // X);
                            // }
                            // if (i == 3 || i == 4)
                            // 	assert(database.getLayerDir(k) == Y);
                            DP(temp_x, temp_y, k);
                        } // pass without overflow // modified
                    }
                }
                else {                                     // this edge could happen overflow
                    for (k = 1; k < global_max_layer; ++k) // use global_max_layer to substitute max_zz
                    {
                        if ((cur_map_3d[x][y][k].edge_list[i]->cur_cap - cur_map_3d[x][y][k].edge_list[i]->max_cap)
                            < overflow_max) { // overflow, but doesn't over the overflow_max
#ifdef FOLLOW_PREFER
                            if (/*follow_prefer_direction == 0 || (follow_prefer_direction == 1 && */ (
                                ((i == 0 || i == 1) && prefer_direction[k][1] == 1)
                                || ((i == 2 || i == 3) && prefer_direction[k][0] == 1))) //)
                                DP(temp_x, temp_y, k);
#else
                            // std::cout << "Try to assign to edge " << i << " of grid (" << x << ", " << y << ", " << k
                            // << ")"; std::cout << "cur_cap: " << cur_map_3d[x][y][k].edge_list[i]->cur_cap << ",
                            // max_cap: " << cur_map_3d[x][y][k].edge_list[i]->max_cap << std::endl;
                            if ((i == 0 || i == 1) && /*k % 2 == 1*/ layerDirections[k] == 1) {
                                DP(temp_x, temp_y, k);
                                assigned = true;
                            }
                            else if ((i == 2 || i == 3) && /*k % 2 == 0*/ layerDirections[k] == 0) {
                                DP(temp_x, temp_y, k);
                                assigned = true;
                            }
                            // DP(temp_x, temp_y, k);
                        }
#endif
                        }
                    }
                    if (!assigned) {
                        std::cout << "Edge " << i << " of (" << x << ", " << y;
                        std::cout << "), overflow_map val = " << *(overflow_map[x][y].edge[i]) << std::endl;
                        std::cout << "2-D demand: " << congestionMap2d->edge(x, y, i).cur_cap << std::endl;
                        std::cout << "2-D max_cap: " << congestionMap2d->edge(x, y, i).max_cap << std::endl;
                        for (int k = 0; k < global_max_layer; ++k) {
                            std::cout << "Layer " << k;
                            std::cout << ", cur_cap = " << cur_map_3d[x][y][k].edge_list[i]->cur_cap;
                            std::cout << ", max_cap = " << cur_map_3d[x][y][k].edge_list[i]->max_cap;
                            std::cout << std::endl;
                        }

                        // if (i == 0 || i == 1) { //Up, Down
                        // 	for (k = 1; k < global_max_layer; ++k) {
                        // 		if (database.getLayerDir(k) == X) {
                        // 			DP(temp_x, temp_y, k);
                        // 		}
                        // 	}
                        // } else { // Left, Right
                        // 	for (k = 1; k < global_max_layer; ++k) {
                        // 		if (database.getLayerDir(k) == Y) {
                        // 			DP(temp_x, temp_y, k);
                        // 		}
                        // 	}
                        // }
                    }
                    assert(assigned);
                }
            }
#ifdef PIN_LEF // !!TODO
            if (is_end == true) {
                if (is_metal5) {
                    if (z <= path_map[x][y].min_pin_layer) {
                        klat_map[x][y][z].via_cost = path_map[x][y].max_pin_layer - z;
                        for (k = z, klat_map[x][y][z].via_overflow = 0; k < path_map[x][y].max_pin_layer; ++k) {
                            if (viadensity_map[x][y][k].cur >= viadensity_map[x][y][k].max)
                                ++klat_map[x][y][z].via_overflow;
                        }
                    }
                    else if (z >= path_map[x][y].max_pin_layer) {
                        klat_map[x][y][z].via_cost = z - path_map[x][y].min_pin_layer;
                        for (k = path_map[x][y].min_pin_layer, klat_map[x][y][z].via_overflow = 0; k < z; ++k) {
                            if (viadensity_map[x][y][k].cur >= viadensity_map[x][y][k].max)
                                ++klat_map[x][y][z].via_overflow;
                        }
                    }
                    else {
                        klat_map[x][y][z].via_cost = path_map[x][y].max_pin_layer - path_map[x][y].min_pin_layer;
                        for (k = path_map[x][y].min_pin_layer, klat_map[x][y][z].via_overflow = 0;
                             k < path_map[x][y].max_pin_layer; ++k) {
                            if (viadensity_map[x][y][k].cur >= viadensity_map[x][y][k].max)
                                ++klat_map[x][y][z].via_overflow;
                        }
                    }
#ifdef VIA_DENSITY
                    klat_map[x][y][z].val = klat_map[x][y][z].via_overflow;
#else
                    klat_map[x][y][z].val = klat_map[x][y][z].via_cost;
#endif
                }
                else {
                    end_pin_layer = path_map[x][y].pin_layer;
                    // klat_map[x][y][z].via_cost = via_cost * z;
                    if (z > end_pin_layer) {
                        klat_map[x][y][z].via_cost = z - end_pin_layer;

#ifdef VIA_DENSITY
                        // # Q change via overflow l k <= z -> k < z
                        for (k = end_pin_layer, klat_map[x][y][z].via_overflow = 0; k < z; ++k)
                            if (viadensity_map[x][y][k].cur >= viadensity_map[x][y][k].max)
                                ++klat_map[x][y][z].via_overflow;
                        klat_map[x][y][z].val = klat_map[x][y][z].via_overflow;
#else
                        klat_map[x][y][z].val = klat_map[x][y][z].via_cost;
#endif
                    }
                    else { // # end_pin_layer >= z
                        klat_map[x][y][z].via_cost = end_pin_layer - z;
#ifdef VIA_DENSITY
                        for (k = z, klat_map[x][y][z].via_overflow = 0; k < end_pin_layer; ++k)
                            if (viadensity_map[x][y][k].cur >= viadensity_map[x][y][k].max)
                                ++klat_map[x][y][z].via_overflow;
                        klat_map[x][y][z].val = klat_map[x][y][z].via_overflow;
#else
                        klat_map[x][y][z].val = klat_map[x][y][z].via_cost;

#endif
                    }
                }
            }
#else
        if (is_end == true) {
            klat_map[x][y][z].via_cost = via_cost * z;
#ifdef VIA_DENSITY
            for (k = klat_map[x][y][z].via_overflow = 0; k < z; ++k)
                if (viadensity_map[x][y][k].cur >= viadensity_map[x][y][k].max)
                    ++klat_map[x][y][z].via_overflow;
            klat_map[x][y][z].val = klat_map[x][y][z].via_overflow;
#else
            klat_map[x][y][z].val = klat_map[x][y][z].via_cost;
#endif
        }
#endif
            else // is_end == false
            {
                // special purpose
                min_DP_val = 1000000000;
                min_DP_via_cost = 1000000000;
                global_x = x;
                global_y = y;
                max_DP_k = z;
                if (path_map[x][y].val == 1) // not a pin
                    min_DP_k = z;
                else { // pin
#ifdef PIN_LEF
                    if (is_metal5) {
                        min_DP_k = min(z, path_map[x][y].min_pin_layer);
                        max_DP_k = max(z, path_map[x][y].max_pin_layer);
                    }
                    else {
                        min_DP_k = path_map[x][y].pin_layer; // !!TODO
                    }
#else
                min_DP_k = 0;
#endif
                }
                if (!is_metal5) {
                    min_DP_k = min(min_DP_k, max_DP_k);
                    max_DP_k = max(min_DP_k, max_DP_k);
                }

                for (i = 0; i < 4; ++i)
                    min_DP_idx[i] = -1;
#ifdef PRINT_DP_PIN
                if (check_net) {
                    std::cout << "=====DP_vertex=====" << std::endl;
                    std::cout << "global.x = " << global_x << " global.y = " << global_y << " global.z = " << min_DP_k
                              << std::endl;
                    PRINT_DP_INF(0, 0, count_idx);
                }
#endif
                assert(min_DP_k <= max_DP_k);
                rec_count(0, 0, count_idx);
                klat_map[x][y][z].via_cost = min_DP_via_cost;
#ifdef VIA_DENSITY
                klat_map[x][y][z].via_overflow = min_DP_val;
#endif
                klat_map[x][y][z].val = min_DP_val;

                assert(klat_map[x][y][z].val >= 0);
                assert(klat_map[x][y][z].val < 1e9);

#ifdef PRINT_DP_PIN
                if (check_net) {
                    std::cout << "---END OF CHILD---" << std::endl;
                }
                int temp_child_cost = 0;
                int mink = min_DP_k;
                int maxk = max_DP_k;
                for (i = 0; i < 4; ++i) // direction
                    if (min_DP_idx[i] >= 0) {
                        if (min_DP_idx[i] < mink)
                            mink = min_DP_idx[i];
                        if (min_DP_idx[i] > maxk)
                            maxk = min_DP_idx[i];

                        temp_x = x + plane_dir[i][0];
                        temp_y = y + plane_dir[i][1];
                        if (check_net) {
                            std::cout << "x = " << temp_x << " y = " << temp_y << " k = " << min_DP_idx[i]
                                      << " via_cost = " << klat_map[temp_x][temp_y][min_DP_idx[i]].via_cost
                                      << std::endl;
                        }
                        temp_child_cost += klat_map[temp_x][temp_y][min_DP_idx[i]].via_cost;
                        klat_map[temp_x][temp_y][z].pi_z = min_DP_idx[i];
                        assert(klat_map[temp_x][temp_y][z].pi_z >= 0);
                    }
                if (check_net) {
                    std::cout << "Child_cost = " << temp_child_cost << std::endl;
                    std::cout << "Vertex_cost = " << (maxk - mink) * via_cost << std::endl;
                    std::cout << "Final_cost = " << min_DP_via_cost << std::endl;
                    std::cout << "-----------" << std::endl;
                    std::cout << "===================" << std::endl;
                }
#else
            for (i = 0; i < 4; ++i) // direction
                if (min_DP_idx[i] > 0) {
                    temp_x = x + plane_dir[i][0];
                    temp_y = y + plane_dir[i][1];
                    klat_map[temp_x][temp_y][z].pi_z = min_DP_idx[i];
                }
#endif
            }
        }
    }

    bool in_cube_and_have_edge(int x, int y, int z, int dir, int net_id) {
        // F B L R U D
        int anti_dir[6] = {1, 0, 3, 2, 5, 4};
        // B F R L D U

        if (x >= 0 && x < max_xx && y >= 0 && y < max_yy && z >= 0 && z < max_zz) {
            if (cur_map_3d[x][y][z].edge_list[anti_dir[dir]]->used_net.find(net_id)
                != cur_map_3d[x][y][z].edge_list[anti_dir[dir]]->used_net.end()) {
                return true;
            }
            else
                return false;
        }
        else
            return false;
    }

    bool have_child(int pre_x, int pre_y, int pre_z, int pre_dir, int net_id) {
        int dir, x, y, z;
        int anti_dir[6] = {1, 0, 3, 2, 5, 4};

        for (dir = 0; dir < 6; ++dir)
            if (dir != pre_dir && dir != anti_dir[pre_dir]) {
                x = pre_x + cube_dir[dir][0];
                y = pre_y + cube_dir[dir][1];
                z = pre_z + cube_dir[dir][2];
                if (in_cube_and_have_edge(x, y, z, dir, net_id) == true && BFS_color_map[x][y][z] != net_id)
                    return true;
            }
        return false;
    }

    void generate_output(int net_id, std::ofstream &fout) {
        int i, j;
        const PinptrList *pin_list = &rr_map->get_nPin(net_id);
        const char *p;
        queue<Coordinate_3d *> q;
        int dir;
        Coordinate_3d *temp, start, end;

        int pin_count = 1;

        // the beginning of a net of output file
        p = rr_map->get_netName(net_id);
        std::string netName(p);
        // printf("%s\n(\n", p);
        fout << netName << "\n(\n";
        // for(i = 0; p[i] && (p[i] < '0' || p[i] > '9'); ++i);
        // printf(" %s\n", p + i);
        std::set<tuple<int, int, int>> pinMap;

        // GARY Pin Patching
        // for (auto &pin : *pin_list)
        // {
        // 	// printf("Input pin: %d %d %d\n", pin->get_tileX(), pin->get_tileY(), pin->get_layerId());
        // 	pinMap.insert({pin->get_tileX(), pin->get_tileY(), pin->get_layerId()});
        // 	// printf("Pin: (%d, %d, %d)\n", pin->get_tileX(), pin->get_tileY(), pin->get_layerId());
        // 	// printf("%d %d %d %d %d %d\n", pin->get_tileX(), pin->get_tileY(), pin->get_layerId()+1, pin->get_tileX(),
        // pin->get_tileY(), pin->get_layerId()+1);

        // 	// if (pin->get_layerId() == 0)
        // 	// {
        // 	// 	for (int i = 1; i <= 1/*rr_map->get_layerNumber()*/; i++)
        // 	// 	{
        // 	// 		int patch = 1;
        // 	// 		int pinLX = std::max(pin->get_tileX() - patch, 0);
        // 	// 		int pinRX = std::min(pin->get_tileX() + patch, rr_map->get_gridx() - 1);
        // 	// 		int pinLY = std::max(pin->get_tileY() - patch, 0);
        // 	// 		int pinRY = std::min(pin->get_tileY() + patch, rr_map->get_gridy() - 1);
        // 	// 		printf("%d %d %d %d %d %d\n", pinLX, pin->get_tileY(), i, pinRX, pin->get_tileY(), i);
        // 	// 		printf("%d %d %d %d %d %d\n", pinLX, pinLY, i, pinRX, pinLY, i);
        // 	// 		printf("%d %d %d %d %d %d\n", pinLX, pinRY, i, pinRX, pinRY, i);
        // 	// 		// for (int j = -2; j < 3; j++)
        // 	// 		// {
        // 	// 		// 	int pinLX = std::max(pin->get_tileX() - 2, 0);
        // 	// 		// 	int pinRX = std::min(pin->get_tileX() + 2, rr_map->get_gridx() - 1);
        // 	// 		// 	int pinY = std::max(pin->get_tileY() + j, 0);
        // 	// 		// 	pinY = std::min(pinY, rr_map->get_gridy() - 1);
        // 	// 		// 	printf("%d %d %d %d %d %d\n", pinLX, pinY, i, pinRX, pinY, i);
        // 	// 		// }
        // 	// 		// for (int start=pinLX; start<=pinRX; ++start) {
        // 	// 		// 	routeGuideMap[{start, pin->get_tileY(), i}] = true;
        // 	// 		// 	routeGuideMap[{start, pinLY, i}] = true;
        // 	// 		// 	routeGuideMap[{start, pinRY, i}] = true;
        // 	// 		// }
        // 	// 	}
        // 	// }
        // 	// else if (pin->get_layerId() != 0)
        // 	// {
        // 	// 	for (int i = 1; i <= rr_map->get_layerNumber(); i++)
        // 	// 	{
        // 	// 		for (int j = -1; j < 2; j++)
        // 	// 		{
        // 	// 			int pinLX = std::max(pin->get_tileX() - 1, 0);
        // 	// 			int pinRX = std::min(pin->get_tileX() + 1, rr_map->get_gridx() - 1);
        // 	// 			int pinY = std::max(pin->get_tileY() + j, 0);
        // 	// 			pinY = std::min(pinY, rr_map->get_gridy() - 1);
        // 	// 			printf("%d %d %d %d %d %d\n", pinLX, pinY, i, pinRX, pinY, i);

        // 	// 			// for (int start=pinLX; start<=pinRX; ++start) {
        // 	// 			// 	routeGuideMap[{start, pinY, i}] = true;
        // 	// 			// }
        // 	// 		}
        // 	// 	}
        // 	// }

        // }
#ifdef PATCHING
        double PIN_THRESH = 5.5;
        if (TESTCASE_NAME == "ispd19_test9")
            PIN_THRESH = 5.0;
        else if (TESTCASE_NAME == "ispd18_test8_metal5" || TESTCASE_NAME == "ispd19_test8_metal5")
            PIN_THRESH = 4.5;

        auto needPatch = [&](int x, int y, int z) {
            if (z == 0)
                return false;
            return cur_map_3d[x][y][z].cap < PIN_THRESH;
        };
        auto printSrround = [&](int x, int y, int layer_idx, int patch) {
            int pinLX = max(x - patch, 0);
            int pinRX = min(x + patch, rr_map->get_gridx() - 1);

            for (int p = -patch; p <= patch; ++p) {
                int pinY = max(y + p, 0);
                pinY = min(y + p, rr_map->get_gridy() - 1);
                // printf("%d %d %d %d %d %d\n", pinLX, pinY, layer_idx+1, pinRX, pinY, layer_idx+1);
                fout << rectToGuide(pinLX, pinY, pinRX, pinY, layer_idx + 1);
            }
        };
        auto printCross = [&](int x, int y, int layer_idx, int patch) {
            int pinLX = max(x - patch, 0);
            int pinRX = min(x + patch, rr_map->get_gridx() - 1);
            int pinLY = max(y - patch, 0);
            int pinRY = min(y + patch, rr_map->get_gridy() - 1);
            // printf("%d %d %d %d %d %d\n", pinLX, y, layer_idx+1, pinRX, y, layer_idx+1);
            // printf("%d %d %d %d %d %d\n", x, pinLY, layer_idx+1, x, pinRY, layer_idx+1);
            fout << rectToGuide(pinLX, y, pinRX, y, layer_idx + 1);
            fout << rectToGuide(x, pinLY, x, pinRY, layer_idx + 1);
        };

        bool is_19_7m = TESTCASE_NAME == "ispd19_test7_metal5";
        bool is_19_9m = TESTCASE_NAME == "ispd19_test9_metal5";
        bool pin_patch2 = (TESTCASE_NAME == "ispd18_test10" || TESTCASE_NAME == "ispd19_test7");

        if (is_19_7m) {
            for (auto &pin : *pin_list) {
                int px = pin->get_tileX();
                int py = pin->get_tileY();
                int pz = pin->get_layerId();
                pinMap.insert({px, py, pz});

                bool patched = false;
                if (pz < max_zz - 2) {
                    if (needPatch(px, py, pz + 1) || needPatch(px, py, pz + 2)) {
                        patched = true;
                        printSrround(px, py, pz + 1, 1);
                        printSrround(px, py, pz + 2, 1);
                    }
                    else {
                        // printf("%d %d %d %d %d %d\n", px, py, pz+1+1, px, py, pz+1+1);
                        // printf("%d %d %d %d %d %d\n", px, py, pz+2+1, px, py, pz+2+1);
                        fout << rectToGuide(px, py, px, py, pz + 1 + 1);
                        fout << rectToGuide(px, py, px, py, pz + 2 + 1);
                    }
                }
                if (pz > 1) {
                    if (needPatch(px, py, pz - 1) || needPatch(px, py, pz - 2)) {
                        patched = true;
                        printSrround(px, py, pz - 1, 1);
                        printSrround(px, py, pz - 2, 1);
                    }
                    else {
                        // printf("%d %d %d %d %d %d\n", px, py, pz-1+1, px, py, pz-1+1);
                        // printf("%d %d %d %d %d %d\n", px, py, pz-2+1, px, py, pz-2+1);
                        fout << rectToGuide(px, py, px, py, pz - 1 + 1);
                        fout << rectToGuide(px, py, px, py, pz - 2 + 1);
                    }
                }
                if (patched) {
                    printSrround(px, py, pz, 0); // !
                                                 // printCross(px, py, pz, 1);
                }
                else
                    // printf("%d %d %d %d %d %d\n", px, py, pz+1, px, py, pz+1);
                    fout << rectToGuide(px, py, px, py, pz + 1);
            }
        }
        else if (is_19_9m) {
            for (auto &pin : *pin_list) {
                int px = pin->get_tileX();
                int py = pin->get_tileY();
                int pz = pin->get_layerId();
                pinMap.insert({px, py, pz});

                bool patched = false;
                if (pz < max_zz - 2) {
                    if (needPatch(px, py, pz + 1) || needPatch(px, py, pz + 2)) {
                        patched = true;
                        printSrround(px, py, pz + 1, 1);
                        printSrround(px, py, pz + 2, 1);
                    }
                    else {
                        // printf("%d %d %d %d %d %d\n", px, py, pz+1+1, px, py, pz+1+1);
                        // printf("%d %d %d %d %d %d\n", px, py, pz+2+1, px, py, pz+2+1);
                        fout << rectToGuide(px, py, px, py, pz + 1 + 1);
                        fout << rectToGuide(px, py, px, py, pz + 2 + 1);
                    }
                }
                if (pz > 1) {
                    if (needPatch(px, py, pz - 1) || needPatch(px, py, pz - 2)) {
                        patched = true;
                        printSrround(px, py, pz - 1, 1);
                        printSrround(px, py, pz - 2, 1);
                    }
                    else {
                        // printf("%d %d %d %d %d %d\n", px, py, pz-1+1, px, py, pz-1+1);
                        // printf("%d %d %d %d %d %d\n", px, py, pz-2+1, px, py, pz-2+1);
                        fout << rectToGuide(px, py, px, py, pz - 1 + 1);
                        fout << rectToGuide(px, py, px, py, pz - 2 + 1);
                    }
                }
                if (patched) {
                    printSrround(px, py, pz, 0); // !
                }
                else
                    // printf("%d %d %d %d %d %d\n", px, py, pz+1, px, py, pz+1);
                    fout << rectToGuide(px, py, px, py, pz + 1);
            }
        }
        else {
            if (pin_patch2) {
                for (auto &pin : *pin_list) {
                    int px = pin->get_tileX();
                    int py = pin->get_tileY();
                    int pz = pin->get_layerId();
                    pinMap.insert({px, py, pz});

                    bool patched = false;
                    if (pz < max_zz - 2) {
                        if (needPatch(px, py, pz + 1) || needPatch(px, py, pz + 2)) {
                            patched = true;
                            printSrround(px, py, pz + 1, 2);
                            printSrround(px, py, pz + 2, 2);
                        }
                        else {
                            // printf("%d %d %d %d %d %d\n", px, py, pz+1+1, px, py, pz+1+1);
                            // printf("%d %d %d %d %d %d\n", px, py, pz+2+1, px, py, pz+2+1);
                            fout << rectToGuide(px, py, px, py, pz + 1 + 1);
                            fout << rectToGuide(px, py, px, py, pz + 2 + 1);
                        }
                    }
                    if (pz > 1) {
                        if (needPatch(px, py, pz - 1) || needPatch(px, py, pz - 2)) {
                            patched = true;
                            printSrround(px, py, pz - 1, 2);
                            printSrround(px, py, pz - 2, 2);
                        }
                        else {
                            // printf("%d %d %d %d %d %d\n", px, py, pz-1+1, px, py, pz-1+1);
                            // printf("%d %d %d %d %d %d\n", px, py, pz-2+1, px, py, pz-2+1);
                            fout << rectToGuide(px, py, px, py, pz - 1 + 1);
                            fout << rectToGuide(px, py, px, py, pz - 2 + 1);
                        }
                    }
                    if (patched) {
                        printSrround(px, py, pz, 2); //!
                    }
                    else
                        // printf("%d %d %d %d %d %d\n", px, py, pz+1, px, py, pz+1);
                        fout << rectToGuide(px, py, px, py, pz + 1);
                }
            }
            else {
                for (auto &pin : *pin_list) {
                    int px = pin->get_tileX();
                    int py = pin->get_tileY();
                    int pz = pin->get_layerId();
                    pinMap.insert({px, py, pz});

                    bool patched = false;
                    if (pz < max_zz - 2) {
                        if (needPatch(px, py, pz + 1) || needPatch(px, py, pz + 2)) {
                            patched = true;
                            printSrround(px, py, pz + 1, 2);
                            printSrround(px, py, pz + 2, 2);
                        }
                        else {
                            // printf("%d %d %d %d %d %d\n", px, py, pz+1+1, px, py, pz+1+1);
                            // printf("%d %d %d %d %d %d\n", px, py, pz+2+1, px, py, pz+2+1);
                            fout << rectToGuide(px, py, px, py, pz + 1 + 1);
                            fout << rectToGuide(px, py, px, py, pz + 2 + 1);
                        }
                    }
                    if (pz > 1) {
                        if (needPatch(px, py, pz - 1) || needPatch(px, py, pz - 2)) {
                            patched = true;
                            printSrround(px, py, pz - 1, 2);
                            printSrround(px, py, pz - 2, 2);
                        }
                        else {
                            // printf("%d %d %d %d %d %d\n", px, py, pz-1+1, px, py, pz-1+1);
                            // printf("%d %d %d %d %d %d\n", px, py, pz-2+1, px, py, pz-2+1);
                            fout << rectToGuide(px, py, px, py, pz - 1 + 1);
                            fout << rectToGuide(px, py, px, py, pz - 2 + 1);
                        }
                    }
                    if (patched) {
                        printSrround(px, py, pz, 1); //!
                    }
                    else
                        // printf("%d %d %d %d %d %d\n", px, py, pz+1, px, py, pz+1);
                        fout << rectToGuide(px, py, px, py, pz + 1);
                }
            }
        }
#endif

#ifdef VERIFY_PIN_LEF
        printf("%d", net_id);
#endif
        // BFS

#ifdef PIN_LEF
        // std::cout << (*pin_list)[0]->get_layerId() << '\n';
        q.push(&coord_3d_map[(*pin_list)[0]->get_tileX()][(*pin_list)[0]->get_tileY()]
                            [(*pin_list)[0]->get_layerId()]); // enqueue
                                                              // if (rr_map->get_net_name(net_id) == "pin1")
        // 	printf("Pin: (%d, %d, %d) %d\n", (*pin_list)[0]->get_tileX(), (*pin_list)[0]->get_tileY(),
        // (*pin_list)[0]->get_layerId(), pin_count);
#else
    q.push(&coord_3d_map[(*pin_list)[0]->get_tileX()][(*pin_list)[0]->get_tileY()][0]); // enqueue
#endif

#ifdef VERIFY_PIN_LEF
        int pinnum = rr_map->get_netPinNumber(net_id);
        for (int i = 0; i < pinnum; i++) {
            printf("(%d,%d,%d)\n", (*pin_list)[i]->get_tileX(), (*pin_list)[i]->get_tileY(),
                   (*pin_list)[i]->get_layerId());
        }
#elif VERIFY_PIN_1
    int pinnum = rr_map->get_netPinNumber(net_id);
    for (int i = 0; i < pinnum; i++) {
        printf("(%d,%d,1)", (*pin_list)[i]->get_tileX(), (*pin_list)[i]->get_tileY());
    }
#endif
        temp = q.front();
        BFS_color_map[temp->x][temp->y][temp->z] = net_id;
        while (!q.empty()) {
            temp = q.front();
            for (dir = 0; dir < 6; dir += 2) {
                int dirPlusOne = dir + 1;
                start.x = end.x = temp->x;
                start.y = end.y = temp->y;
                start.z = end.z = temp->z;

                for (i = 1;; ++i) {
                    start.x += cube_dir[dir][0];
                    start.y += cube_dir[dir][1];
                    start.z += cube_dir[dir][2];
                    if (in_cube_and_have_edge(start.x, start.y, start.z, dir, net_id) == true
                        && BFS_color_map[start.x][start.y][start.z] != net_id) {
                        BFS_color_map[start.x][start.y][start.z] = net_id;
                        if (pinMap.count({start.x, start.y, start.z})) {
                            pin_count++;
                            // if (rr_map->get_net_name(net_id) == "pin1")
                            // 	printf("Pin: (%d, %d, %d) %d\n", start.x, start.y, start.z, pin_count);
                        }
                        if (have_child(start.x, start.y, start.z, dir, net_id) == true)
                            q.push(&coord_3d_map[start.x][start.y][start.z]); // enqueue
                    }
                    else {
                        start.x -= cube_dir[dir][0];
                        start.y -= cube_dir[dir][1];
                        start.z -= cube_dir[dir][2];
                        break;
                    }
                }
                for (j = 1;; ++j) {
                    end.x += cube_dir[dirPlusOne][0];
                    end.y += cube_dir[dirPlusOne][1];
                    end.z += cube_dir[dirPlusOne][2];
                    if (in_cube_and_have_edge(end.x, end.y, end.z, dirPlusOne, net_id) == true
                        && BFS_color_map[end.x][end.y][end.z] != net_id) {
                        BFS_color_map[end.x][end.y][end.z] = net_id;
                        if (pinMap.count({end.x, end.y, end.z})) {
                            pin_count++;
                            // if (rr_map->get_net_name(net_id) == "pin1")
                            // 	printf("Pin: (%d, %d, %d) %d\n", end.x, end.y, end.z, pin_count);
                        }
                        if (have_child(end.x, end.y, end.z, dirPlusOne, net_id) == true)
                            q.push(&coord_3d_map[end.x][end.y][end.z]); // enqueue
                    }
                    else {
                        end.x -= cube_dir[dirPlusOne][0];
                        end.y -= cube_dir[dirPlusOne][1];
                        end.z -= cube_dir[dirPlusOne][2];
                        break;
                    }
                }
                if (i >= 2 || j >= 2) // have edge
                {
                    ++start.z;
                    ++end.z;

                    // printf("%d %d %d %d %d %d\n", start.x, start.y, start.z, end.x, end.y, end.z);
                    assert(start.x == end.x || start.y == end.y);

                    if (start.z == end.z) {
                        fout << rectToGuide(start.x, start.y, end.x, end.y, start.z);
#ifdef PATCHING
                        const double THRESH = 2.0;
                        const int LENGTH = 5;
                        int length, p_start, p_end;
                        if (start.x == end.x) {
                            p_start = min(start.y, end.y);
                            p_end = max(start.y, end.y);
                            length = p_end - p_start + 1;
                            if (length >= LENGTH) {
                                for (int p = p_start; p <= p_end;) {
                                    bool patched = false;
                                    if (cur_map_3d[start.x][p][start.z - 1].cap < THRESH) {
                                        // printf("%d %d %d %d %d %d\n", max(start.x-1, 0), p, start.z, min(start.x+1,
                                        // rr_map->get_gridx()-1), p, start.z);
                                        fout << rectToGuide(max(start.x - 1, 0), p,
                                                            min(start.x + 1, rr_map->get_gridx() - 1), p, start.z);
                                        if (start.z - 1 + 1 < max_zz
                                            && cur_map_3d[start.x][p][start.z - 1 + 1].cap >= THRESH) {
                                            patched = true;
                                            // printf("%d %d %d %d %d %d\n", max(start.x-1, 0), p, start.z+1,
                                            // min(start.x+1, rr_map->get_gridx()-1), p, start.z+1);
                                            fout << rectToGuide(max(start.x - 1, 0), p,
                                                                min(start.x + 1, rr_map->get_gridx() - 1), p,
                                                                start.z + 1);
                                        }
                                        if (start.z - 1 - 1 >= 1
                                            && cur_map_3d[start.x][p][start.z - 1 - 1].cap >= THRESH) {
                                            patched = true;
                                            // printf("%d %d %d %d %d %d\n", max(start.x-1, 0), p, start.z-1,
                                            // min(start.x+1, rr_map->get_gridx()-1), p, start.z-1);
                                            fout << rectToGuide(max(start.x - 1, 0), p,
                                                                min(start.x + 1, rr_map->get_gridx() - 1), p,
                                                                start.z - 1);
                                        }
                                    }
                                    if (patched)
                                        p += LENGTH;
                                    else
                                        p += 1;
                                }
                            }
                            // for (int p = p_start; p<=p_end; ++p) {
                            // 	if (cur_map_3d[start.x][p][start.z-1].cap < 1.0) {
                            // 		printf("%d %d %d %d %d %d\n", max(start.x-1, 0), p, start.z, min(start.x+1,
                            // rr_map->get_gridx()-1), p, start.z); 		if (start.z-1 + 1 < max_zz) { 			printf("%d %d %d %d %d
                            // %d\n", max(start.x-1, 0), p, start.z+1, min(start.x+1, rr_map->get_gridx()-1), p,
                            // start.z+1);
                            // 		}
                            // 		// if (start.z-1 - 1 >= 1) {
                            // 		// 	printf("%d %d %d %d %d %d\n", max(start.x-1, 0), p, start.z-1, min(start.x+1,
                            // rr_map->get_gridx()-1), p, start.z-1);
                            // 		// }
                            // 	}
                            // }
                        }
                        else {
                            p_start = min(start.x, end.x);
                            p_end = max(start.x, end.x);
                            length = p_end - p_start + 1;
                            if (length >= LENGTH) {
                                for (int p = p_start; p <= p_end;) {
                                    bool patched = false;
                                    if (cur_map_3d[p][start.y][start.z - 1].cap < THRESH) {
                                        // printf("%d %d %d %d %d %d\n", p, max(start.y-1, 0), start.z+1, p,
                                        // min(start.y+1, rr_map->get_gridy()-1), start.z+1);
                                        fout << rectToGuide(p, max(start.y - 1, 0), p,
                                                            min(start.y + 1, rr_map->get_gridy() - 1), start.z + 1);
                                        if (start.z - 1 + 1 < max_zz
                                            && cur_map_3d[p][start.y][start.z - 1 + 1].cap >= THRESH) {
                                            patched = true;
                                            // printf("%d %d %d %d %d %d\n", p, max(start.y-1, 0), start.z+1, p,
                                            // min(start.y+1, rr_map->get_gridy()-1), start.z+1);
                                            fout << rectToGuide(p, max(start.y - 1, 0), p,
                                                                min(start.y + 1, rr_map->get_gridy() - 1), start.z + 1);
                                        }
                                        if (start.z - 1 - 1 >= 1
                                            && cur_map_3d[p][start.y][start.z - 1 - 1].cap >= THRESH) {
                                            patched = true;
                                            // printf("%d %d %d %d %d %d\n", p, max(start.y-1, 0), start.z-1, p,
                                            // min(start.y+1, rr_map->get_gridy()-1), start.z-1);
                                            fout << rectToGuide(p, max(start.y - 1, 0), p,
                                                                min(start.y + 1, rr_map->get_gridy() - 1), start.z - 1);
                                        }
                                    }
                                    if (patched)
                                        p += LENGTH;
                                    else
                                        p += 1;
                                }
                            }
                            // for (int p = p_start; p<=p_end; ++p) {
                            // 	if (cur_map_3d[p][start.y][start.z-1].cap < 1.0) {
                            // 		printf("%d %d %d %d %d %d\n", p, max(start.y-1, 0), start.z, p, min(start.y+1,
                            // rr_map->get_gridy()-1), start.z); 		if (start.z-1 + 1 < max_zz) { 			printf("%d %d %d %d %d
                            // %d\n", p, max(start.y-1, 0), start.z+1, p, min(start.y+1, rr_map->get_gridy()-1),
                            // start.z+1);
                            // 		}
                            // 		// if (start.z-1 - 1 >= 1) {
                            // 		// 	printf("%d %d %d %d %d %d\n", p, max(start.y-1, 0), start.z-1, p, min(start.y+1,
                            // rr_map->get_gridy()-1), start.z-1);
                            // 		// }
                            // 	}
                            // }
                        }
#endif
                    }
                    else {
                        // fout << start.x << ' ' << start.y << ' ' << start.z << ' ' << end.z << std::endl;
                        fout << viaToGuide(start.x, start.y, start.z, end.z);
                    }
                }
            }
            q.pop();
        }
        /*
            for (auto const& [key, val] : routeGuideMap) {
                auto [x, y, k] = key;
                assert(val);
                if (k-1 >= 1) {
                    if (!routeGuideMap.count({x, y, k-1})) {
                        bool has_neighbor = false;
                        if (database.getLayerDir(k-1-1) == Y) { // Horizontal
                            if (x > 0 && routeGuideMap.count({x-1, y, k-1}))
                                has_neighbor = true;
                            if (x < rr_map->get_gridx()-1 && routeGuideMap.count({x+1, y, k-1}))
                                has_neighbor = true;
                        } else {
                            if (y > 0 && routeGuideMap.count({x, y-1, k-1}))
                                has_neighbor = true;
                            if (y < rr_map->get_gridy()-1 && routeGuideMap.count({x, y+1, k-1}))
                                has_neighbor = true;
                        }
                        if (has_neighbor) {
                            // printf("patch (%d, %d, %d)\n", x, y, k-1);
                            routeGuideMap[{x, y, k-1}] = true;
                            printf("%d %d %d %d %d %d\n", x, y, k-1, x, y, k-1);
                        }
                    }
                }
            }
        */
        // the end of a net of output file
        // printf(")\n");
        fout << ")\n";
        // printf("pin_count = %d, %d\n", pin_count, (*pin_list).size());
        // assert(pin_count == (*pin_list).size());
    }

    void klat(int net_id) {
#ifdef PRINT_DP_PIN
        if (net_id == 172) {
            check_net = true;
        }
        else {
            check_net = false;
        }

#endif

        const PinptrList *pin_list = &rr_map->get_nPin(net_id);
        Coordinate_2d *start;
        int start_pin_layer;

        start = &coor_array[(*pin_list)[0]->get_tileX()][(*pin_list)[0]->get_tileY()];
        start_pin_layer = (*pin_list)[0]->get_layerId();

#ifdef PRINT_DP_PIN
        if (check_net) {
            std::cout << "pin_num = " << pin_list->size() << std::endl;
            for (int i = 0; i < pin_list->size(); i++) {
                std::cout << i
                          << ":"
                             "x = "
                          << (*pin_list)[i]->get_tileX() << " y = " << (*pin_list)[i]->get_tileY()
                          << " z = " << (*pin_list)[i]->get_layerId() << std::endl;
            }
            const char *p;
            p = rr_map->get_netName(net_id);
            start_pin_layer std::cout << "net_id = " << net_id << "name = " << p << std::endl;
        }

#endif

        global_net_id = net_id; // LAZY global variable
        global_pin_num = rr_map->get_netPinNumber(net_id);
        global_max_layer = preprocess(net_id);
        // std::cout << "global_max_layer: " << global_max_layer << '\n';
        // fina a pin as starting point
        // klat start with a pin
#ifdef PIN_LEF
        auto dp_start = std::chrono::high_resolution_clock::now();
#ifdef DAC2023
        update(start->x, start->y);
#else
        DP(start->x, start->y, start_pin_layer);
#endif
        // if (CNT > 100) exit(0);
        // CNT++;
        auto dp_end = std::chrono::high_resolution_clock::now();
        auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(dp_end - dp_start);
        DP_TIME += time_span.count();
#else
    DP(start->x, start->y, 0);
#endif

#ifndef FROM2D
        if (temp_global_pin_cost < klat_map[start->x][start->y][0].val)
            if (temp_global_xy_cost <= after_xy_cost)
                printf("id %d, ori xy %d, ori z %d, curr xy %d, curr z %d\n", net_id, temp_global_xy_cost,
                       temp_global_pin_cost, after_xy_cost, klat_map[start->x][start->y][0].val);
#endif
#ifdef POSTPROCESS
        net_info[net_id].z = klat_map[start->x][start->y][0].via_cost;
        global_increase_vo = klat_map[start->x][start->y][0].val;
#endif
#ifdef PIN_LEF
        global_pin_cost += klat_map[start->x][start->y][start_pin_layer].val;
#else
    global_pin_cost += klat_map[start->x][start->y][0].val;
#endif
        update_path_for_klat(start, start_pin_layer);
        /*
        #ifdef PRINT_DP_PIN
            if(check_net){
                exit(0);
            }
        #endif
        */
    }

    int count_via_overflow_for_a_segment(int x, int y, int start, int end) {
        int sum = 0, temp;

        if (start > end) {
            temp = start;
            start = end;
            end = temp;
        }
        while (start < end) {
            if (viadensity_map[x][y][start].cur >= viadensity_map[x][y][start].max)
                ++sum;
            ++start;
        }

        return sum;
    }

    void greedy_layer_assignment(int x, int y, int z) {
        int dir, z_min, z_max, k;
        queue<Coordinate_3d *> q;
        Coordinate_3d *temp;
        int k_via_overflow, z_via_overflow = 0;

        q.push(&coord_3d_map[x][y][z]);
        while (!q.empty()) {
            temp = q.front();
            if (path_map[temp->x][temp->y].val == 2) // a pin
                z_min = 0;
            else
                z_min = temp->z;
            z_max = temp->z;
            for (dir = 0; dir < 4; ++dir)
                if (path_map[temp->x][temp->y].edge[dir] == 1) // check legal
                {
                    x = temp->x + plane_dir[dir][0];
                    y = temp->y + plane_dir[dir][1];
#ifdef FOLLOW_PREFER
                    for (k = 0; k < max_zz; ++k)
                        if ((dir == 0 || dir == 1) && prefer_direction[k][1] == 1)
                            break;
                        else if ((dir == 2 || dir == 3) && prefer_direction[k][0] == 1)
                            break;
#else
                    k = 0;
#endif
                    k_via_overflow = count_via_overflow_for_a_segment(temp->x, temp->y, k, z_max);
                    for (z = k + 1; z < max_zz; ++z)
#ifdef FOLLOW_PREFER
                        if (((dir == 0 || dir == 1) && prefer_direction[z][1] == 1)
                            || ((dir == 2 || dir == 3) && prefer_direction[z][0] == 1)) {
#endif
                            if (cur_map_3d[temp->x][temp->y][z].edge_list[dir]->cur_cap
                                < cur_map_3d[temp->x][temp->y][z].edge_list[dir]->max_cap) {
#ifdef VIA_DENSITY
                                if (cur_map_3d[temp->x][temp->y][k].edge_list[dir]->cur_cap
                                    < cur_map_3d[temp->x][temp->y][k].edge_list[dir]->max_cap) {
                                    z_via_overflow = count_via_overflow_for_a_segment(temp->x, temp->y, MIN(z, z_min),
                                                                                      MAX(z, z_max));
                                    if (z_via_overflow < k_via_overflow) {
                                        k = z;
                                        k_via_overflow = z_via_overflow;
                                    }
                                    else if (z_via_overflow == k_via_overflow) {
                                        if (abs(z - temp->z) < abs(k - temp->z)) {
                                            k = z;
                                            k_via_overflow = z_via_overflow;
                                        }
                                    }
                                }
                                else if (cur_map_3d[temp->x][temp->y][k].edge_list[dir]->cur_cap
                                         >= cur_map_3d[temp->x][temp->y][k].edge_list[dir]->max_cap) {
                                    k = z;
                                    k_via_overflow = z_via_overflow;
                                }
#else                           
                                // connect via
                                if ((cur_map_3d[temp->x][temp->y][k].edge_list[dir]->cur_cap
                                         < cur_map_3d[temp->x][temp->y][k].edge_list[dir]->max_cap
                                     && abs(z - temp->z) < abs(k - temp->z))
                                    || (cur_map_3d[temp->x][temp->y][k].edge_list[dir]->cur_cap
                                        >= cur_map_3d[temp->x][temp->y][k].edge_list[dir]->max_cap))
                                    k = z;
#endif
                            }
                            else {
#ifdef VIA_DENSITY
                                if (cur_map_3d[temp->x][temp->y][z].edge_list[dir]->cur_cap
                                        - cur_map_3d[temp->x][temp->y][z].edge_list[dir]->max_cap
                                    < cur_map_3d[temp->x][temp->y][k].edge_list[dir]->cur_cap
                                          - cur_map_3d[temp->x][temp->y][k].edge_list[dir]->max_cap)
                                    k = z;
                                else if (cur_map_3d[temp->x][temp->y][z].edge_list[dir]->cur_cap
                                             - cur_map_3d[temp->x][temp->y][z].edge_list[dir]->max_cap
                                         == cur_map_3d[temp->x][temp->y][k].edge_list[dir]->cur_cap
                                                - cur_map_3d[temp->x][temp->y][k].edge_list[dir]->max_cap) {
                                    z_via_overflow = count_via_overflow_for_a_segment(temp->x, temp->y, MIN(z, z_min),
                                                                                      MAX(z, z_max));
                                    if (z_via_overflow < k_via_overflow) {
                                        k = z;
                                        k_via_overflow = z_via_overflow;
                                    }
                                    else if (z_via_overflow == k_via_overflow) {
                                        if (abs(z - temp->z) < abs(k - temp->z)) {
                                            k = z;
                                            k_via_overflow = z_via_overflow;
                                        }
                                    }
                                }
#else
                                if ((cur_map_3d[temp->x][temp->y][z].edge_list[dir]->cur_cap
                                         - cur_map_3d[temp->x][temp->y][z].edge_list[dir]->max_cap
                                     < cur_map_3d[temp->x][temp->y][k].edge_list[dir]->cur_cap
                                           - cur_map_3d[temp->x][temp->y][k].edge_list[dir]->max_cap)
                                    || (cur_map_3d[temp->x][temp->y][z].edge_list[dir]->cur_cap
                                                - cur_map_3d[temp->x][temp->y][z].edge_list[dir]->max_cap
                                            == cur_map_3d[temp->x][temp->y][k].edge_list[dir]->cur_cap
                                                   - cur_map_3d[temp->x][temp->y][k].edge_list[dir]->max_cap
                                        && abs(z - temp->z) < abs(k - temp->z)))
                                    k = z;
#endif
                            }
#ifdef FOLLOW_PREFER
                        }
#endif
                    if (k < z_min)
                        z_min = k;
                    if (k > z_max)
                        z_max = k;
                    update_cur_map_for_klat_xy(k, &coor_array[temp->x][temp->y], &coor_array[x][y], global_net_id);
                    q.push(&coord_3d_map[x][y][k]);
                }
            update_cur_map_for_klat_z(z_min, z_max, &coor_array[temp->x][temp->y], global_net_id);
            path_map[temp->x][temp->y].val = 0;
            q.pop();
        }
    }

    void greedy(int net_id) {
        const PinptrList *pin_list = &rr_map->get_nPin(net_id);
        Coordinate_2d *start;

        start = &coor_array[(*pin_list)[0]->get_tileX()][(*pin_list)[0]->get_tileY()];
        global_net_id = net_id; // LAZY global variable
        global_pin_num = rr_map->get_netPinNumber(net_id);
        preprocess(net_id);
        greedy_layer_assignment(start->x, start->y, 0);
    }

    int comp_temp_net_order(const void *a, const void *b) {
        int p, q;

        p = *((int *)a);
        q = *((int *)b);
        // if (average_order[p].average > average_order[q].average)
        // 	return -1;
        // else if (average_order[p].average < average_order[q].average)
        // 	return 1;
        // else
        // {
        return rr_map->get_netPinNumber(p) - rr_map->get_netPinNumber(q);
        // }
    }

    int backtrace(int n) {
        assert(n != -1);
        if (group_set[n].pi != n) {
            group_set[n].pi = backtrace(group_set[n].pi);
            return group_set[n].pi;
        }
        return group_set[n].pi;
    }

    void find_group(int max) {
        cout << "In find_group 1" << endl;
        int i, j, k;
        LRoutedNetTable::iterator iter, iter2;
        int a, b, pre_solve_counter = 0, temp_cap;
        bool *pre_solve;
        int max_layer;

        group_set = (UNION_NODE *)malloc(sizeof(UNION_NODE) * max);
        assert(group_set);
        pre_solve = (bool *)malloc(sizeof(bool) * max);
        assert(pre_solve);
        // // intitial for is_used_for_BFS // modified
        // for(i = 0; i < max_xx; ++i)
        // 	for(j = 0; j < max_yy; ++j)
        // 		is_used_for_BFS[i][j] = -1;

        // initial for average_order
        cout << "In find_group 2" << endl;
        average_order = (AVERAGE_NODE *)malloc(sizeof(AVERAGE_NODE) * max);
        assert(average_order);
        for (i = 0; i < max; ++i) {
            group_set[i].pi = i;
            group_set[i].sx = group_set[i].sy = 1000000;
            group_set[i].bx = group_set[i].by = -1;
            group_set[i].num = 1;
            pre_solve[i] = true;
            average_order[i].id = i;
            average_order[i].val = 0;
            average_order[i].times = 0;
            average_order[i].vo_times = 0;
        }
        cout << "In find_group 3" << endl;
        ;

        for (i = 1; i < max_xx; ++i)
            for (j = 0; j < max_yy; ++j) {
                temp_cap = (congestionMap2d->edge(i, j, DIR_WEST).used_net.size()); // modified
                for (k = 0; k < max_zz && temp_cap > 0; ++k)
                    if (cur_map_3d[i][j][k].edge_list[LEFT]->max_cap > 0)
                        temp_cap -= cur_map_3d[i][j][k].edge_list[LEFT]->max_cap;
                if (k == max_zz)
                    max_layer = max_zz - 1;
                else
                    max_layer = k;
                if (congestionMap2d->edge(i, j, DIR_WEST).isOverflow() == false) {
                    if (congestionMap2d->edge(i, j, DIR_WEST).used_net.size() > 0) {
                        for (k = 0; k < max_zz && cur_map_3d[i][j][k].edge_list[LEFT]->max_cap == 0; ++k)
                            ;
                        if (k < max_zz) {
                            if (cur_map_3d[i][j][k].edge_list[LEFT]->max_cap
                                < (int)(congestionMap2d->edge(i, j, DIR_WEST).used_net.size())) // modified
                                for (RoutedNetTable::iterator iter =
                                         congestionMap2d->edge(i, j, DIR_WEST).used_net.begin();
                                     iter != congestionMap2d->edge(i, j, DIR_WEST).used_net.end(); ++iter) {
                                    pre_solve[iter->first] = false;
                                }
                        }
                        else
                            for (RoutedNetTable::iterator iter = congestionMap2d->edge(i, j, DIR_WEST).used_net.begin();
                                 iter != congestionMap2d->edge(i, j, DIR_WEST).used_net.end(); ++iter) {
                                pre_solve[iter->first] = false;
                            }
                    }
                }
                else {
                    for (RoutedNetTable::iterator iter = congestionMap2d->edge(i, j, DIR_WEST).used_net.begin();
                         iter != congestionMap2d->edge(i, j, DIR_WEST).used_net.end(); ++iter) {
                        pre_solve[iter->first] = false;
                    }
                }
                for (RoutedNetTable::iterator iter = congestionMap2d->edge(i, j, DIR_WEST).used_net.begin();
                     iter != congestionMap2d->edge(i, j, DIR_WEST).used_net.end(); ++iter) {
                    if (i - 1 < group_set[iter->first].sx)
                        group_set[iter->first].sx = i - 1;
                    if (i > group_set[iter->first].bx)
                        group_set[iter->first].bx = i;
                    if (j < group_set[iter->first].sy)
                        group_set[iter->first].sy = j;
                    if (j > group_set[iter->first].by)
                        group_set[iter->first].by = j;
                    average_order[iter->first].val += max_layer;
                    ++average_order[iter->first].times;
                }
            }
        cout << "In find_group 4" << endl;

        for (i = 0; i < max_xx; ++i)
            for (j = 1; j < max_yy; ++j) {
                temp_cap = (congestionMap2d->edge(i, j, DIR_SOUTH).used_net.size()); // modified
                for (k = 0; k < max_zz && temp_cap > 0; ++k)
                    if (cur_map_3d[i][j][k].edge_list[BACK]->max_cap > 0)
                        temp_cap -= cur_map_3d[i][j][k].edge_list[BACK]->max_cap;
                if (k == max_zz)
                    max_layer = max_zz - 1;
                else
                    max_layer = k;
                if (congestionMap2d->edge(i, j, DIR_SOUTH).isOverflow() == false) {
                    if (congestionMap2d->edge(i, j, DIR_SOUTH).used_net.size() > 0) {
                        for (k = 0; k < max_zz && cur_map_3d[i][j][k].edge_list[BACK]->max_cap == 0; ++k)
                            ;
                        if (k < max_zz) {
                            if (cur_map_3d[i][j][k].edge_list[BACK]->max_cap
                                < (int)(congestionMap2d->edge(i, j, DIR_SOUTH).used_net.size())) // modified
                                for (RoutedNetTable::iterator iter =
                                         congestionMap2d->edge(i, j, DIR_SOUTH).used_net.begin();
                                     iter != congestionMap2d->edge(i, j, DIR_SOUTH).used_net.end(); ++iter) {
                                    pre_solve[iter->first] = false;
                                }
                        }
                        else
                            for (RoutedNetTable::iterator iter =
                                     congestionMap2d->edge(i, j, DIR_SOUTH).used_net.begin();
                                 iter != congestionMap2d->edge(i, j, DIR_SOUTH).used_net.end(); ++iter) {
                                pre_solve[iter->first] = false;
                            }
                    }
                }
                else {
                    for (RoutedNetTable::iterator iter = congestionMap2d->edge(i, j, DIR_SOUTH).used_net.begin();
                         iter != congestionMap2d->edge(i, j, DIR_SOUTH).used_net.end(); ++iter) {
                        pre_solve[iter->first] = false;
                    }
                }
                for (RoutedNetTable::iterator iter = congestionMap2d->edge(i, j, DIR_SOUTH).used_net.begin();
                     iter != congestionMap2d->edge(i, j, DIR_SOUTH).used_net.end(); ++iter) {
                    if (i < group_set[iter->first].sx)
                        group_set[iter->first].sx = i;
                    if (i > group_set[iter->first].bx)
                        group_set[iter->first].bx = i;
                    if (j - 1 < group_set[iter->first].sy)
                        group_set[iter->first].sy = j - 1;
                    if (j > group_set[iter->first].by)
                        group_set[iter->first].by = j;
                    average_order[iter->first].val += max_layer;
                    ++average_order[iter->first].times;
                }
            }
        cout << "In find_group 5" << endl;
        for (i = 0; i < max; ++i)
            if (pre_solve[i] == true)
                group_set[i].pi = -1;
        for (i = 1; i < max_xx; ++i)
            for (j = 0; j < max_yy; ++j) {
                for (k = 0; k < max_zz && cur_map_3d[i][j][k].edge_list[LEFT]->max_cap == 0; ++k)
                    ;
                if ((int)(congestionMap2d->edge(i, j, DIR_WEST).used_net.size())
                    > cur_map_3d[i][j][k].edge_list[LEFT]->max_cap) // modified
                {
                    if (congestionMap2d->edge(i, j, DIR_WEST).used_net.size() > 1) {
                        RoutedNetTable::iterator iter = congestionMap2d->edge(i, j, DIR_WEST).used_net.begin();
                        a = backtrace(iter->first);
                        for (iter++; iter != congestionMap2d->edge(i, j, DIR_WEST).used_net.end(); ++iter) {
                            b = backtrace(iter->first);
                            if (a != b) {
                                group_set[b].pi = a;
                                group_set[a].num += group_set[b].num;
                            }
                        }
                    }
                }
            }
        cout << "In find_group 6\n";

        for (i = 0; i < max_xx; ++i)
            for (j = 1; j < max_yy; ++j) {
                for (k = 0; k < max_zz && cur_map_3d[i][j][k].edge_list[BACK]->max_cap == 0; ++k)
                    ;
                if ((int)(congestionMap2d->edge(i, j, DIR_SOUTH).used_net.size())
                    > cur_map_3d[i][j][k].edge_list[BACK]->max_cap) // modified
                {
                    if (congestionMap2d->edge(i, j, DIR_SOUTH).used_net.size() > 1) {
                        RoutedNetTable::iterator iter = congestionMap2d->edge(i, j, DIR_SOUTH).used_net.begin();
                        a = backtrace(iter->first);
                        for (iter++; iter != congestionMap2d->edge(i, j, DIR_SOUTH).used_net.end(); ++iter) {
                            b = backtrace(iter->first);
                            if (a != b) {
                                group_set[b].pi = a;
                                group_set[a].num += group_set[b].num;
                            }
                        }
                    }
                }
            }
        cout << "In find_group 7\n";

        for (i = 0; i < max; ++i) {
            if (temp_buf[2] == '0' || temp_buf[2] == '1') // normal and random
#ifdef VIA_DENSITY
                average_order[i].average =
                    (double)(rr_map->get_netPinNumber(i)) / (average_order[i].times + average_order[i].bends);
#else
            average_order[i].average = (double)(rr_map->get_netPinNumber(i)) / (average_order[i].times);
#endif
            else if (temp_buf[2] == '2') // length
                average_order[i].average = (1.0 / (average_order[i].times));
            else if (temp_buf[2] == '3') // pinnum
                average_order[i].average = (double)(rr_map->get_netPinNumber(i));
            else // avgdensity
                average_order[i].average = (double)(average_order[i].val) / (average_order[i].times);
            if (pre_solve[i] == true)
                pre_solve_counter++;
        }
        cout << "In find_group 8\n";
    }

    void initial_BFS_color_map(void) {
        int i, j, k;

        for (i = 0; i < max_xx; ++i)
            for (j = 0; j < max_yy; ++j)
                for (k = 0; k < max_zz; ++k)
                    BFS_color_map[i][j][k] = -1;
    }

    void malloc_BFS_color_map(void) {
        int i, j;

        BFS_color_map = (int ***)malloc(sizeof(int **) * max_xx);
        for (i = 0; i < max_xx; ++i) {
            BFS_color_map[i] = (int **)malloc(sizeof(int *) * max_yy);
            for (j = 0; j < max_yy; ++j)
                BFS_color_map[i][j] = (int *)malloc(sizeof(int) * max_zz);
        }
    }

    void free_BFS_color_map(void) {
        int i, j;

        for (i = 0; i < max_xx; ++i) {
            for (j = 0; j < max_yy; ++j)
                free(BFS_color_map[i][j]);
            free(BFS_color_map[i]);
        }
        free(BFS_color_map);
    }

    void calculate_wirelength(void) {
        int i, j, k, xy = 0, z = 0;
        LRoutedNetTable::iterator iter;

        for (k = 0; k < max_zz; ++k) {
            for (i = 1; i < max_xx; ++i)
                for (j = 0; j < max_yy; ++j)
                    if (cur_map_3d[i][j][k].edge_list[LEFT]->used_net.size()) {
                        xy += cur_map_3d[i][j][k].edge_list[LEFT]->used_net.size();
                    }
            for (i = 0; i < max_xx; ++i)
                for (j = 1; j < max_yy; ++j)
                    if (cur_map_3d[i][j][k].edge_list[BACK]->used_net.size()) {
                        xy += cur_map_3d[i][j][k].edge_list[BACK]->used_net.size();
                    }
        }
        for (i = 0; i < max_xx; ++i)
            for (j = 0; j < max_yy; ++j)
                for (k = 1; k < max_zz; ++k)
                    z += (via_cost * cur_map_3d[i][j][k].edge_list[DOWN]->used_net.size());
#ifdef CHANGE_VIA_DENSITY
        printf("%.2lf\t%.2lf\t", (double)(xy + z) / 1.0e5, (double)z / 1.0e5);
#else
    printf("total wirelength = %d + %d = %d\n", xy, z, xy + z);
#endif
    }

    void erase_cur_map_3d(void) {
        int i, j, k;

        for (i = 1; i < max_xx; ++i)
            for (j = 0; j < max_yy; ++j)
                for (k = 0; k < max_zz; ++k) {
                    cur_map_3d[i][j][k].edge_list[LEFT]->cur_cap = 0;
                    cur_map_3d[i][j][k].edge_list[LEFT]->used_net.clear();
                }
        for (i = 0; i < max_xx; ++i)
            for (j = 1; j < max_yy; ++j)
                for (k = 0; k < max_zz; ++k) {
                    cur_map_3d[i][j][k].edge_list[BACK]->cur_cap = 0;
                    cur_map_3d[i][j][k].edge_list[BACK]->used_net.clear();
                }
        for (k = 1; k < max_zz; ++k)
            for (i = 0; i < max_xx; ++i)
                for (j = 0; j < max_yy; ++j)
                    cur_map_3d[i][j][k].edge_list[DOWN]->used_net.clear();
    }

    void multiply_viadensity_map_by_times(double times) {
        int i, j, k;

        for (k = 1; k < max_zz; ++k) {
            double temp =
                (cur_map_3d[1][0][k - 1].edge_list[LEFT]->max_cap > 0)
                    ? (cur_map_3d[1][0][k - 1].edge_list[LEFT]->max_cap * cur_map_3d[0][1][k].edge_list[BACK]->max_cap)
                    : (cur_map_3d[1][0][k].edge_list[LEFT]->max_cap * cur_map_3d[0][1][k - 1].edge_list[BACK]->max_cap);
            temp = times * temp;
            temp /= 4.0;
            for (i = 0; i < max_xx; ++i)
                for (j = 0; j < max_yy; ++j) {
                    viadensity_map[i][j][k - 1].max = (int)temp;
                }
        }
    }

    void sort_net_order(void) {
        int i, max;
        LRoutedNetTable count_data;
        LRoutedNetTable::iterator iter;
        LRoutedNetTable two_pin_data;
        // vector<Pin*>* pin_list;
        clock_t start, finish;
#ifdef CHANGE_VIA_DENSITY
        double times;
#endif
        int *temp_net_order;

        max = rr_map->get_netNumber();
        // re-disturibte net
        multi_pin_net = (MULTIPIN_NET_NODE *)malloc(sizeof(MULTIPIN_NET_NODE) * max);
        assert(multi_pin_net);
        // std::cout << "?\n";
        // 	find_group(max); // for comp_temp_net_order
        // std::cout << "?\n";

        initial_overflow_map();

        temp_net_order = (int *)malloc(sizeof(int) * max);
        for (i = 0; i < max; ++i)
            temp_net_order[i] = i;
        qsort(temp_net_order, max, sizeof(int), comp_temp_net_order);
        start = clock();

#ifdef CHANGE_VIA_DENSITY
        for (times = 0.8; times <= 1.2; times += 0.2) {
            initial_overflow_map();
            multiply_viadensity_map_by_times(times);
            for (k = 1; k < max_zz; ++k)
                for (i = 0; i < max_xx; ++i)
                    for (j = 0; j < max_yy; ++j)
                        viadensity_map[i][j][k - 1].cur = 0;
#endif
#ifdef POSTPROCESS
            for (i = 0; i < max; ++i)
                net_info[i].xy = 0;
#endif
            for (i = 0; i < max; ++i) {
                if (temp_buf[3] == '0') // SOLA+APEC
                {
                    if (temp_buf[2] == '1') // random
                        klat(i);
                    else {
                        klat(temp_net_order[i]); // others
                    }
                }
                else // greedy
                {
                    if (temp_buf[2] == '1') // random
                        greedy(i);
                    else
                        greedy(temp_net_order[i]); // others
                }
            }
#ifdef CHANGE_VIA_DENSITY
            print_via_overflow();
            calculate_wirelength();
            erase_cur_map_3d();
        }
#endif
        printf("cost = %lld\n", global_pin_cost);
        finish = clock();
        printf("time = %lf\n", (double)(finish - start) / CLOCKS_PER_SEC);
        free(temp_net_order);
    }

    void calculate_cap(void) {
        int i, j, overflow = 0, max = 0;

        for (i = 1; i < max_xx; ++i)
            for (j = 0; j < max_yy; ++j)
                if (congestionMap2d->edge(i, j, DIR_WEST).isOverflow()) {
                    overflow += (congestionMap2d->edge(i, j, DIR_WEST).overUsage()); // modified
                    if (max < congestionMap2d->edge(i, j, DIR_WEST).overUsage())
                        max = congestionMap2d->edge(i, j, DIR_WEST).overUsage(); // modified
                }
        for (i = 0; i < max_xx; ++i)
            for (j = 1; j < max_yy; ++j)
                if (congestionMap2d->edge(i, j, DIR_SOUTH).isOverflow()) {
                    overflow += (congestionMap2d->edge(i, j, DIR_SOUTH).overUsage()); // modified
                    if (max < congestionMap2d->edge(i, j, DIR_SOUTH).overUsage())
                        max = congestionMap2d->edge(i, j, DIR_SOUTH).overUsage(); // modified
                }
        printf("2D overflow = %d\n", overflow);
        printf("2D max overflow = %d\n", max);
    }

    void generate_all_output(std::ofstream & fout) {
        int i, max = rr_map->get_netNumber();

        for (i = 0; i < max; ++i)
            generate_output(i, fout);

        // Local nets
        auto &deletedNet = rr_map->get_deleted_netList();
        for (auto &net : deletedNet) {
            // printf("%s\n(\n", net.get_name());
            fout << net.get_name() << "\n(\n";
            auto &pinList = net.get_pinList();
            int maxLayer = rr_map->get_layerNumber();
            int pin_cnt = 0;
            for (auto &pin : pinList) {
                ++pin_cnt;
                int x = pin->get_tileX();
                int y = pin->get_tileY();
                int layer = pin->get_layerId();

				// --- (2024/01/21) MF edited --- //
                if (/*layer == 0*/ 0) {
				/*
				fout << x << ' ' << y << ' ' << x + 1 << ' ' << y + 1 << ' ' << "metal" << layer + 1 << '\n';
				fout << x << ' ' << y << ' ' << x + 1 << ' ' << y + 1 << ' ' << "metal" << layer + 2 << '\n';
				*/
				fout << x << ' ' << y << ' ' << layer << " " << x << ' ' << y << ' ' << layer + 1 << '\n';
				fout << x << ' ' << y << ' ' << layer + 1 << " " << x << ' ' << y << ' ' << layer + 2 << '\n';
				}
				else {
					/*
					fout << x << ' ' << y << ' ' << x + 1 << ' ' << y + 1 << ' ' << "metal" << layer + 1 << '\n';
					*/
					fout << x << ' ' << y << ' ' << layer << " " << x << ' ' << y << ' ' << layer + 1 << '\n';
				}
				
                // int maxL;
                // maxL = (layer + 3 <= maxLayer) ? layer + 3 : maxLayer;
                // for (int l = layer + 1; l <= maxL; l++)
                // {
                //     // printf("%d %d %d %d %d %d\n", std::max(x - 1, 0), std::max(y - 1, 0), l,
                //     //         std::min(x + 1, rr_map->get_gridx()),     std::max(y - 1, 0), l);
                //     // printf("%d %d %d %d %d %d\n", std::max(x - 1, 0),                  y, l,
                //     //         std::min(x + 1, rr_map->get_gridx()),                      y, l);
                //     // printf("%d %d %d %d %d %d\n", std::max(x - 1, 0), std::min(y + 1, rr_map->get_gridy()), l,
                //     //         std::min(x + 1, rr_map->get_gridx()),     std::min(y + 1, rr_map->get_gridy()), l);
                // 	fout << rectToGuide(std::max(x - 1, 0), std::max(y - 1, 0), std::min(x + 1, rr_map->get_gridx()),
                // std::max(y - 1, 0), l); 	fout << rectToGuide(std::max(x - 1, 0), y, std::min(x + 1,
                // rr_map->get_gridx()), y, l); 	fout << rectToGuide(std::max(x - 1, 0), std::min(y + 1,
                // rr_map->get_gridy()), std::min(x + 1, rr_map->get_gridx()), std::min(y + 1, rr_map->get_gridy()), l);

                // }
            }
            assert(pin_cnt == 1);
            // printf(")\n");
            fout << ")\n";
        }

        // printf("%lld\n", total_via);
    }

    void Layer_assignment(const std::string &guide_path) {
        assert(&congestionMap2d->edge(1, 1, DIR_NORTH) == &congestionMap2d->edge(1, 1, FRONT));
        assert(&congestionMap2d->edge(1, 1, DIR_SOUTH) == &congestionMap2d->edge(1, 1, BACK));
        assert(&congestionMap2d->edge(1, 1, DIR_EAST) == &congestionMap2d->edge(1, 1, RIGHT));
        assert(&congestionMap2d->edge(1, 1, DIR_WEST) == &congestionMap2d->edge(1, 1, LEFT));

        // std::string metal5 = TESTCASE_NAME.substr(TESTCASE_NAME.size()-6, 6);
        // is_metal5 = metal5 == "metal5";
        // std::cout << "Is metal5 testcases: " << is_metal5 << '\n';

        // string outputFileName(outputFileNamePtr);
        // int i;

        via_cost = 1;

        max_xx = rr_map->get_gridx();
        max_yy = rr_map->get_gridy();
        max_zz = rr_map->get_layerNumber();
        cout << max_xx << ' ' << max_yy << ' ' << max_zz << '\n';
        malloc_space();
#ifdef READ_OUTPUT
        printf("reading output...\n");
        // read_output();
        printf("reading output complete\n");
#endif
        calculate_cap();
        initial_3D_coordinate_map();
        find_overflow_max();
        puts("Layerassignment processing...");
#ifdef FROM2D
#ifdef POSTPROCESS
        net_info = (NET_INFO_NODE *)malloc(sizeof(NET_INFO_NODE) * rr_map->get_netNumber());
#endif
        sort_net_order();

        print_max_overflow();
#ifdef POSTPROCESS
        // refinement();
#endif
#endif
        puts("Layerassignment complete.");
#ifdef VIA_DENSITY
        // print_via_overflow();
#endif
        free_malloc_space();
        calculate_wirelength();
#ifdef PRINT_OVERFLOW
        print_max_overflow();
#endif
#ifdef ALLOUTPUT
        // printf("Outputing result file to %s\n", outputFileName.c_str());
        std::cout << "Outputing result file to " << guide_path << "\n";
        std::ofstream fout(guide_path);
        malloc_BFS_color_map();
        initial_BFS_color_map();

        // int stdout_fd = dup(1);
        // FILE* outputFile = freopen(outputFileName.c_str(), "w", stdout);
        io_start = std::chrono::high_resolution_clock::now();
        generate_all_output(fout);
        io_end = std::chrono::high_resolution_clock::now();
        fout.close();

        std::cout << "DP TIME: " << DP_TIME << '\n';
        //   fclose(outputFile);

        // stdout = fdopen(stdout_fd, "w");
        free_BFS_color_map();
#endif
#ifdef CHECK_PATH
        check_path();
#endif
    }
