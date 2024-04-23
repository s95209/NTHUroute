#include "Range_router.h"
#include "Construct_2d_tree.h"

#include "Post_processing.h"
#include "MM_mazeroute.h"

#include "misc/geometry.h"
#include "util/traversemap.h"

#include <cmath>
#include <algorithm>

using namespace Jm;
using namespace std;

static vector<Range_element*> range_vector;
static Interval_element interval_list[INTERVAL_NUM];

static int num_of_grid_edge = 0;
static double min_congestion = 0.; 
static double max_congestion = 0.;
static double avg_congestion = 0.;
static int intervalCount = INTERVAL_NUM;

double MAZE_TIME = 0;

static VertexColorMap<int>* expandMap;   //This color map is used by 
                                //expand_range()
                                //for recording which edges have expanded
                                //
static VertexColorMap<int>* routeStateMap; //This color map is used by 
                                //query_range_2pin()
                                //for recording if all the 2-pin nets which
                                //locate on the same tile are routed

static bool double_equal(double a,double b)
{
    double diff = a - b;
	if (diff>0.00001 || diff<-0.00001)
		return false;
    else return true;
}

/*sort grid_edge in decending order*/
static bool comp_grid_edge(const Grid_edge_element* a, const Grid_edge_element* b)
{
	return congestionMap2d->edge(a->grid->x, a->grid->y, a->dir).congestion() >
           congestionMap2d->edge(b->grid->x, b->grid->y, b->dir).congestion();
}

/*
	determine INTERVAL_NUM(10) intervals between min and max,
	and also compute average congestion value (sum of demand / sum of capacity)
*/
void define_interval()
{
	min_congestion = 1000000;
	max_congestion = -1000000;
	avg_congestion = 0;
	num_of_grid_edge = 0;
    double edgeCongestion;

	// 找出最大最小和平均的congention number
	for (int i=rr_map->get_gridx()-1; i >= 0; --i) {
		for (int j=rr_map->get_gridy()-2; j >= 0; --j) {

            edgeCongestion = congestionMap2d->edge(i, j, DIR_NORTH).congestion();

			if (edgeCongestion > 1.0) {
                if (edgeCongestion < min_congestion)
                    min_congestion = edgeCongestion;
                if (edgeCongestion > max_congestion)
                    max_congestion = edgeCongestion;
                avg_congestion += edgeCongestion;
                ++num_of_grid_edge;
            }
		}
    }

	for (int i=rr_map->get_gridx()-2; i >= 0; --i) {
		for (int j=rr_map->get_gridy()-1; j >= 0; --j) {
            edgeCongestion = congestionMap2d->edge(i, j, DIR_EAST).congestion();
            if (edgeCongestion > 1.0) {
                if (edgeCongestion < min_congestion)
                    min_congestion = edgeCongestion;
                if (edgeCongestion > max_congestion)
                    max_congestion = edgeCongestion;
                avg_congestion += edgeCongestion;
                ++num_of_grid_edge;
            }
		}
    }
		
	avg_congestion /= num_of_grid_edge;
	
	interval_list[0].begin_value = max_congestion; // 切區間

    if(max_congestion >= 2) max_congestion_factor = 2.0; // max_congestion_factor 最大值為0
    else max_congestion_factor = max_congestion;
	
	
	for (int i = 1; i < intervalCount; ++i)// 前面定義為10
	{
		interval_list[i-1].end_value = 
		interval_list[i].begin_value =
        max_congestion - ( (double)i/intervalCount ) * (max_congestion-1.0);
	}
	interval_list[intervalCount-1].end_value = 1;
	// {[2, 1.9], [1.9, 1.8], [1.8, 1.7], ....., [1.1, 1]}

	for (int i = 0; i < intervalCount; ++i)
	{
        for(int j = interval_list[i].grid_edge_vector.size()-1; j >=0; --j) {
            delete interval_list[i].grid_edge_vector[j];
        }
		interval_list[i].grid_edge_vector.clear();
	}
}

static void insert_to_interval(double cong_value, Coordinate_2d* coor_2d, int dir)
{
	int i;
	for (i = intervalCount - 1; i >= 0; --i)
	{
        if ( ((cong_value < interval_list[i].begin_value) || (double_equal(cong_value, interval_list[i].begin_value) == true)) 
             && cong_value > interval_list[i].end_value)
        {
            interval_list[i].grid_edge_vector.push_back(new Grid_edge_element(coor_2d, dir));
            return;
        } 
	}
}

void divide_grid_edge_into_interval()
{
	double tmp;
	for (int i = 0; i < rr_map->get_gridx() - 1; ++i) {
		for (int j = 0; j < rr_map->get_gridy(); ++j)
		{
            tmp = congestionMap2d->edge(i, j, DIR_EAST).congestion();
			if (tmp > 1)
				insert_to_interval(tmp, &coor_array[i][j], RIGHT);
		}
    }
		
	for (int i=0; i<rr_map->get_gridx(); ++i) {
		for (int j=0; j<rr_map->get_gridy()-1; ++j)
		{
            tmp = congestionMap2d->edge(i, j, DIR_NORTH).congestion();
			if (tmp > 1)
				insert_to_interval(tmp, &coor_array[i][j],FRONT);
		}
    }
}

static void expand_range(int x1, int y1, int x2, int y2, int interval_index)
{
	Coordinate_2d start,end,cur_start,cur_end;
	int i,j,edge_num;
	double total_cong,avg_cong;
	
    //(x1, y1) (x2, y2) are neighbor, so we can do the following operation
	x1 = min(x1,x2);
	x2 = max(x1,x2);
	y1 = min(y1,y2);
	y2 = max(y1,y2);

    //obtain the expanded boundary. This is the very first expand, and the expand unit is 10.
	// 一次擴大10的grid
	start.x = max(x1 - EXPAND_RANGE_SIZE, 0);
	end.x = min(x2 + EXPAND_RANGE_SIZE, rr_map->get_gridx() - 1);
	start.y = max(y1 - EXPAND_RANGE_SIZE, 0);
	end.y = min(y2 + EXPAND_RANGE_SIZE, rr_map->get_gridy() - 1);
	
	total_cong = 0;
	edge_num = 0;

    //Obtain the total congestion value and edge number of RIGHT edges. 
	for (i = start.x; i < end.x; ++i) {
		for (j = start.y; j <= end.y; ++j)
		{
			total_cong += congestionMap2d->edge(i, j, DIR_EAST).congestion();
			edge_num++;
            expandMap->color(i, j) = interval_index;
		}
    }
	// 上面用到的最上排還沒標記所以用這行補
    //Set "visited" flag to the grid cells on the right most colum
	for (j=start.y; j<=end.y; ++j) {
        expandMap->color(end.x, j) = interval_index;
    }
		
    //Obtain the total congestion value and edge number of FRONT edges. 
	for (i=start.x; i<=end.x; ++i) {
		for (j=start.y; j<end.y; ++j)
		{
			total_cong += congestionMap2d->edge(i, j, DIR_NORTH).congestion();
			edge_num++;
		}
    }
		
	avg_cong = total_cong / edge_num;

	while ((avg_cong > interval_list[interval_index].end_value) && (avg_cong - interval_list[interval_index].end_value > 0.01))
	{
        cur_start.x = max(start.x - EXPAND_RANGE_INC, 0);
        cur_end.x = min(end.x + EXPAND_RANGE_INC, rr_map->get_gridx()-1);
        cur_start.y = max(start.y - EXPAND_RANGE_INC, 0);
        cur_end.y = min(end.y + EXPAND_RANGE_INC, rr_map->get_gridy()-1);


		if (cur_start.x == 0  &&  cur_end.x == rr_map->get_gridx() - 1  &&  cur_start.y == 0  &&  cur_end.y == rr_map->get_gridy() - 1)
		{
			start = cur_start;
			end = cur_end;
			break;
		}
		
        //Below here are four for loops, those for loops update the congestion condition
		for (i=cur_start.x; i<cur_end.x; ++i)
		{
			if (cur_start.y != start.y)
			{
				j = cur_start.y;
				total_cong += congestionMap2d->edge(i, j, DIR_EAST).congestion();
				edge_num++;
                expandMap->color(i, j) = interval_index;
			}
			if (cur_end.y != end.y)
			{
				j = cur_end.y;
				total_cong += congestionMap2d->edge(i, j, DIR_EAST).congestion();
				edge_num++;
                expandMap->color(i, j) = interval_index;
			}
		}
		for (j=start.y; j<=end.y; ++j)
		{
			if (cur_start.x != start.x)
			{
				i = cur_start.x;
				total_cong += congestionMap2d->edge(i, j, DIR_EAST).congestion();
				edge_num++;
			}
			if (cur_end.x != end.x)
			{
				i = end.x;
				total_cong += congestionMap2d->edge(i, j, DIR_EAST).congestion();
				edge_num++;
			}
		}
		for (j=cur_start.y; j<cur_end.y; ++j)
		{
			if (cur_start.x != start.x)
			{
				i = cur_start.x;
				total_cong += congestionMap2d->edge(i, j, DIR_NORTH).congestion();
				edge_num++;
                expandMap->color(i, j) = interval_index;
			}
			if (cur_end.x != end.x)
			{
				i = cur_end.x;
				total_cong += congestionMap2d->edge(i, j, DIR_NORTH).congestion();
				edge_num++;
                expandMap->color(i, j) = interval_index;
			}
		}
		for (i=start.x; i<=end.x; ++i)
		{
			if (cur_start.y != start.y)
			{
				j = cur_start.y;
				total_cong += congestionMap2d->edge(i, j, DIR_NORTH).congestion();
				edge_num++;
			}
			if (cur_end.y != end.y)
			{
				j = end.y;
				total_cong += congestionMap2d->edge(i, j, DIR_NORTH).congestion();
				edge_num++;
			}
		}
        expandMap->color(cur_end.x, cur_end.y) = interval_index; //右上角落那塊
		
		start = cur_start;
		end = cur_end;
		avg_cong = total_cong / edge_num;
	}// end of while loop

    int extraExpandRange = cur_iter / 10; //10個iteration了，就再expand一格
    start.x = max(start.x-extraExpandRange,0);
    end.x = min(end.x+extraExpandRange,rr_map->get_gridx()-1);
    start.y = max(start.y-extraExpandRange,0);
    end.y = min(end.y+extraExpandRange,rr_map->get_gridy()-1);

	range_vector.push_back(new Range_element(start.x, start.y, end.x, end.y));
}

//Rip-up the path that pass any overflowed edges, then route with monotonic 
//routing or multi-source multi-sink routing.
//If there is no overflowed path by using the two methods above, then remain 
//the original path.
static void range_router(Two_pin_element_2d * two_pin)
{
	if ( !check_path_no_overflow(&two_pin->path,two_pin->net_id, false) )
	{
        //Coordinate_2d pin1 = two_pin->pin1;
        //Coordinate_2d pin2 = two_pin->pin2;

        NetDirtyBit[two_pin->net_id] = true;

        update_congestion_map_remove_two_pin_net(two_pin);

		vector<Coordinate_2d*>* bound_path = new vector<Coordinate_2d*>(two_pin->path);

		// use monotonic routing first
        Monotonic_element mn;

		// 先算原始的cost
        compute_path_total_cost_and_distance(two_pin, &mn);


		double bound_cost = mn.total_cost;
		int bound_distance = mn.distance;
		int bound_via_num = mn.via_num;

		two_pin->path.clear();
        bool find_path_flag = false;

        if( routing_parameter->get_monotonic_en() ) {
            bool find_path_flag = monotonic_pattern_route(two_pin->pin1.x, two_pin->pin1.y, 
                    two_pin->pin2.x, two_pin->pin2.y, 
                    two_pin,
					two_pin->net_id,
                    bound_cost,
                    bound_distance,
                    bound_via_num,
                    true);

            if (find_path_flag)
            {	
                delete bound_path;
                bound_path = new vector<Coordinate_2d*>(two_pin->path);
                bound_cost = cong_monotonic[two_pin->path[0]->x][two_pin->path[0]->y].total_cost;
                bound_distance = cong_monotonic[two_pin->path[0]->x][two_pin->path[0]->y].distance;
                bound_via_num = cong_monotonic[two_pin->path[0]->x][two_pin->path[0]->y].via_num;
            }
        }

		two_pin->done = done_iter;

		// if after monotonic routing, there still contain overflow
		if ( (find_path_flag == false) || !check_path_no_overflow(bound_path,two_pin->net_id, true)) {
			
			Coordinate_2d start, end;

            start.x = min(two_pin->pin1.x,two_pin->pin2.x);
            end.x = max(two_pin->pin1.x,two_pin->pin2.x);
            start.y = min(two_pin->pin1.y,two_pin->pin2.y);
            end.y = max(two_pin->pin1.y,two_pin->pin2.y);

            int size = BOXSIZE_INC;	// 10
            start.x = max(0, start.x - size);
            start.y = max(0, start.y - size);
            end.x = min(rr_map->get_gridx()-1, end.x+size);
            end.y = min(rr_map->get_gridy()-1, end.y+size);

    		using namespace std::chrono;
			auto maze_start = std::chrono::high_resolution_clock::now();

			find_path_flag = mazeroute_in_range->mm_maze_route_p2(two_pin,
                                                               bound_cost,
                                                               bound_distance,
                                                               bound_via_num,
                                                               start,
                                                               end);

    		auto maze_end = std::chrono::high_resolution_clock::now();

    		MAZE_TIME += duration_cast<duration<double>>(maze_end - maze_start).count();
			if (find_path_flag == false)
			{
                two_pin->path.insert(two_pin->path.begin(),
                                     bound_path->begin(),
                                     bound_path->end());
			}
		}

		update_congestion_map_insert_two_pin_net(two_pin);
		delete(bound_path);
	}
}

static bool inside_range(int left_x,int bottom_y,int right_x,int top_y,Coordinate_2d *pt)
{
	if(pt->x>=left_x && pt->x<=right_x && pt->y>=bottom_y && pt->y<=top_y)
		return true;
	else
		return false;
}

static void query_range_2pin(int left_x, int bottom_y, int right_x, int top_y, vector<Two_pin_element_2d *> *twopin_list)
{
	vector<Point_fc *> cell_list;
	int len;
	static int done_counter = 0;	//only initialize once
	
	// 將range中的 grid存在cell_list中
	for(int i=left_x;i<=right_x;++i)
	{
		for(int j=bottom_y;j<=top_y;++j)
		{
			cell_list.push_back( &(gridcell->vertex(i, j)) );
		}
	}
	len = cell_list.size();

	for(int i = 0;i < len; ++i)	//for each gCell
	{
		// points 裡是放 經過該grid的 net 的 pin 的 Two_pin_element_2d
		for(vector<Two_pin_element_2d*>::iterator it = cell_list[i]->points.begin(); it != cell_list[i]->points.end(); ++it)	//for each pin or steiner point
		{
            if( (*it)->done != done_iter ) { // 還沒繞好
				//routeStateMap初始值為 -1
                if(routeStateMap->color((*it)->pin1.x, (*it)->pin1.y) != done_counter &&
                   routeStateMap->color((*it)->pin2.x, (*it)->pin2.y) != done_counter)
                {
                    if( inside_range(left_x,bottom_y,right_x,top_y,&((*it)->pin1)) ||
                        inside_range(left_x,bottom_y,right_x,top_y,&((*it)->pin2)) )
                    {
                        (*it)->done = done_iter; //避免重複，因為有兩個pin
                        twopin_list->push_back(*it);
                    }
                }
            }
		}
        routeStateMap->color(cell_list[i]->x, cell_list[i]->y) = done_counter;
	}
	++done_counter;
}

void specify_all_range(void)
{
	vector<Two_pin_element_2d *> twopin_list;
	vector<int> twopin_range_index_list;

    expandMap = new VertexColorMap<int>(rr_map->get_gridx(), rr_map->get_gridy(), -1);
    routeStateMap = new VertexColorMap<int>(rr_map->get_gridx(), rr_map->get_gridy(), -1);
	
	// intervalCount 前面 define 為 10
	for (int i = intervalCount - 1; i >= 0; --i) {

		//initialize range_vector
        for(int s = range_vector.size() - 1; s >= 0; --s) { 
            delete range_vector[s];
        }
		range_vector.clear();

		// congestion 愈大愈前面
		sort(interval_list[i].grid_edge_vector.begin(),
             interval_list[i].grid_edge_vector.end(),
             comp_grid_edge);

		// 對每個區間中的所有 grid_edge
		for (int j = 0; j < (int)interval_list[i].grid_edge_vector.size(); ++j)
		{
			int x = interval_list[i].grid_edge_vector[j]->grid->x;
			int y = interval_list[i].grid_edge_vector[j]->grid->y;
			int dir = interval_list[i].grid_edge_vector[j]->dir;

            int nei_x = x;
            int nei_y = y;
			
            if (dir == RIGHT)
                ++nei_x;
            else ++nei_y;
            
            if ( (expandMap->color(x, y) != i) || (expandMap->color(nei_x, nei_y) != i) )
            {
                expandMap->color(x, y) = i;
                expandMap->color(nei_x, nei_y) = i;
                expand_range(x, y, nei_x, nei_y, i);
            }
		}
		// 對一條two pin net 產生一個 range_element 放入 range vector

		twopin_list.clear();
		twopin_range_index_list.clear();
		for (int j = 0; j<(int)range_vector.size(); ++j)
		{
			query_range_2pin(range_vector[j]->x1,
                             range_vector[j]->y1,
                             range_vector[j]->x2,
                             range_vector[j]->y2,
                             &twopin_list);	
		}
		// 有經過 zero_edge 的放前面，boxSize大的放前面
		sort(twopin_list.begin(),twopin_list.end(), comp_stn_2pin);

		// 繞那些pin有在range中的線
        for (int j = 0; j < (int)twopin_list.size(); ++j) { 
            range_router(twopin_list[j]);
        }
	}

    delete expandMap;
    delete routeStateMap;
	twopin_list.clear();



	// 沒在congestion region 的線現在來繞, range_route會檢查是否有overflow，沒有就不繞
	int length = two_pin_list.size();
	for (int i=0; i<length; ++i)
	{
		if (two_pin_list[i]->done != done_iter)
		{
			twopin_list.push_back(two_pin_list[i]);
		}
	}
	sort(twopin_list.begin(),twopin_list.end(),comp_stn_2pin);
	for (int i = 0; i < (int)twopin_list.size(); ++i)
	{
        if(twopin_list[i]->boxSize() == 1) break;
		range_router(twopin_list[i]);
	}
	mazeroute_in_range->clear_net_tree();
}
