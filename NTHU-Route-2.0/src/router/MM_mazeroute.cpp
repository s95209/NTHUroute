#include "MM_mazeroute.h"
#include "misc/geometry.h"
#include "misc/debug.h"

#include <iostream>
#include <climits>
#include <stack>
#include <queue>
#include <omp.h>
#include <chrono>

#define GAMER

using namespace Jm;
using namespace std;

double GAMER_TIME = 0;
double PATH_SEARCH_TIME = 0;

Multisource_multisink_mazeroute::Multisource_multisink_mazeroute()
:gridxMinusOne(rr_map->get_gridx() - 1),
 gridyMinusOne(rr_map->get_gridy() - 1)
{
	/*allocate space for mmm_map*/
	this->net_tree = new vector<vector<Vertex_mmm *> >(rr_map->get_netNumber(),
                                                       vector<Vertex_mmm *>(0));

    mmm_map = new VertexPlane<MMM_element>(rr_map->get_gridx(), rr_map->get_gridy(),
                                          MMM_element());
#ifndef GAMER
    pqueue = new MMMPriortyQueue();
#endif
	//initialization
	for (int i = 0; i < rr_map->get_gridx(); ++i) {
		for (int j = 0; j < rr_map->get_gridy(); ++j)
		{
			mmm_map->vertex(i, j).coor = &coor_array[i][j];
		}
    }
			
	this->visit_counter = 0;	
	this->dst_counter = 0;
}

Multisource_multisink_mazeroute::~Multisource_multisink_mazeroute()
{
    delete this->net_tree;
    delete this->mmm_map;
#ifndef GAMER
    delete pqueue;
#endif
}

/*recursively traverse parent in maze_routing_map to find path*/
void Multisource_multisink_mazeroute::trace_back_to_find_path_2d(MMM_element *end_point)
{
	MMM_element *cur_pos, *prev_pos = nullptr;
    int numTurns = 0;
    bool is_start = true;
    int pre_dir, now_dir;
	
	cur_pos = end_point;
    // std::cout << "Path: end";
	while (cur_pos)
	{
		this->element->path.push_back(cur_pos->coor);
		cur_pos = cur_pos->parent;
	}
}

//store new 2pins and adjust dfs tree
void Multisource_multisink_mazeroute::adjust_twopin_element()
{
	
	Coordinate_2d* new_pin1 = this->element->path.front();
	Coordinate_2d* new_pin2 = this->element->path.back();
	this->element->pin1 = coor_array[new_pin1->x][new_pin1->y];
	this->element->pin2 = coor_array[new_pin2->x][new_pin2->y];

    // std::cout << "New pin1: (" << new_pin2->x << "," << new_pin2->y << ")\n";
    // std::cout << "New pin2: (" << new_pin1->x << "," << new_pin1->y << ")\n";
	
	vector<Vertex_mmm *>::iterator it;
	//int flag = 0;
	for(it = this->pin1_v->neighbor.begin(); 
        it != this->pin1_v->neighbor.end();
        ++it)
    {
		if((*it) == this->pin2_v)
		{
			this->pin1_v->neighbor.erase(it);
			//flag = 1;
			break;
		}
    }
    //assert(flag == 1);

	//flag=0;
	for(it = this->pin2_v->neighbor.begin();
        it != this->pin2_v->neighbor.end();
        ++it)
    {
		if((*it) == this->pin1_v)
		{
			this->pin2_v->neighbor.erase(it);
			//flag = 1;
			break;
		}
    }
    //assert(flag == 1);

	int net_id = this->element->net_id;
	Vertex_mmm* v1 = NULL;
    Vertex_mmm* v2 = NULL;;
	for(vector<Vertex_mmm *>::iterator it = (*this->net_tree)[net_id].begin();
        it != (*this->net_tree)[net_id].end() && (v1 == NULL || v2 == NULL);
        ++it)
	{
		if((*it)->coor == new_pin1)
		{
			v1 = (*it);
		}
		else if((*it)->coor == new_pin2)
		{
			v2 = (*it);
		}
	}
    assert(v1 != NULL);
    assert(v2 != NULL);
	
	v1->neighbor.push_back(v2);
	v2->neighbor.push_back(v1);
}

void Multisource_multisink_mazeroute::find_subtree(Vertex_mmm *v, int mode, vector<MMM_element*> &sinks)
{
	v->visit = this->visit_counter;
    // std::cout << "find_subtree v = (" << v->coor->x << "," << v->coor->y << ") visit = " << v->visit << "\n";
	
	if(mode == 0)   
	{
        MMM_element *cur = &mmm_map->vertex(v->coor->x, v->coor->y);
        cur->reachCost = 0;
        cur->distance = 0;
        cur->via_num = 0;
        cur->parent = NULL;
        cur->visit = this->visit_counter;
#ifndef GAMER
        pqueue->insert(cur);
#endif
        // std::cout << "Source: (" << v->coor->x << "," << v->coor->y << ")\n";
	}
	else	
	{
		mmm_map->vertex(v->coor->x, v->coor->y).dst = this->dst_counter;
#ifdef GAMER
        sinks.emplace_back(&mmm_map->vertex(v->coor->x, v->coor->y));
#endif
        // std::cout << "Dst: (" << v->coor->x << "," << v->coor->y << ")\n";
	}
    // std::cout << "v's neighbor:";
	for(vector<Vertex_mmm *>::iterator it=v->neighbor.begin();
        it!=v->neighbor.end();
        ++it)
    {
        // std::cout << "(" <<(*it)->coor->x << "," << (*it)->coor->y << ") visit = " << (*it)->visit << "\n";
		if((*it)->visit != this->visit_counter)	
            find_subtree((*it), mode, sinks);
    }
}

void Multisource_multisink_mazeroute::clear_net_tree()
{
	for (int i = 0; i < rr_map->get_netNumber(); ++i)
	{
		int length = (*this->net_tree)[i].size();
		for (int j = 0; j < length; ++j)
			delete((*this->net_tree)[i][j]);
	}

	delete net_tree;
	this->net_tree = new vector<vector<Vertex_mmm *> >(rr_map->get_netNumber(),
                                                       vector<Vertex_mmm *>(0));
}

void Multisource_multisink_mazeroute::setup_pqueue(vector<MMM_element*> &sinks)
{
	
	int cur_net = this->element->net_id;
	if((*this->net_tree)[cur_net].empty())	
	{
        Tree* t = &net_flutetree[cur_net];
		for(int i=0;i<t->number;++i)
		{
			(*this->net_tree)[cur_net].push_back(new Vertex_mmm((int)t->branch[i].x,(int)t->branch[i].y));
		}
		for(int i=0;i<t->number;++i)	
		{
            Vertex_mmm* a = (*this->net_tree)[cur_net][i];
			Vertex_mmm* b = (*this->net_tree)[cur_net][t->branch[i].n];
			if(a->coor->x==b->coor->x && a->coor->y==b->coor->y)
				continue;
			a->neighbor.push_back(b);
			b->neighbor.push_back(a);
            // std::cout << "Neighbor: (" << a->coor->x << "," << a->coor->y << ") <-> (" << b->coor->x << "," << b->coor->y << ")\n";
		}
	}
#ifndef GAMER	
    pqueue->clear();
#endif
	//find pin1 and pin2 in tree
	this->pin1_v = this->pin2_v = NULL;
	for(vector<Vertex_mmm *>::iterator it=(*this->net_tree)[cur_net].begin();
        (it != (*this->net_tree)[cur_net].end()) &&
        (this->pin1_v == NULL || this->pin2_v == NULL);
        ++it)
	{
		if((*it)->coor->x == this->element->pin1.x &&
           (*it)->coor->y == this->element->pin1.y)
		{
			this->pin1_v = (*it);
			this->pin1_v->visit = this->visit_counter;	
            // std::cout << "Pin1 (" << this->element->pin1.x << "," << this->element->pin1.y << ")\n";
		}
		else if((*it)->coor->x == this->element->pin2.x &&
                (*it)->coor->y==this->element->pin2.y)
		{
			this->pin2_v = (*it);
			this->pin2_v->visit = this->visit_counter;	
            // std::cout << "Pin2 (" << this->element->pin2.x << "," << this->element->pin2.y << ")\n";
		}
	}

    assert(this->pin1_v != NULL);
    assert(this->pin2_v != NULL);
	
	this->find_subtree(this->pin1_v, 0, sinks);	//source
	this->find_subtree(this->pin2_v, 1, sinks);	//destination
}

void Multisource_multisink_mazeroute::bfsSetColorMap(int x, int y)
{
	int net_id = this->element->net_id;
    stack< pair<int, int> > Q;

    Q.push(make_pair(x, y));
    while(!Q.empty()){
        x = Q.top().first;
        y = Q.top().second;
        // this->min_l = min(this->min_l, x);
        // this->max_r = max(this->max_r, x);
        // this->min_b = min(this->min_b, y);
        // this->max_t = max(this->max_t, y);
        Q.pop();
        mmm_map->vertex(x, y).walkableID = visit_counter;
        // std::cout << "bfsSetColorMap (" << x << "," << y << ") walkable\n";

		for(int dir = 3; dir >= 0; --dir) {
            if((dir == 3 && x >= gridxMinusOne) ||
               (dir == 2 && x <= 0)             ||
               (dir == 1 && y <= 0)             ||
               (dir == 0 && y >= gridyMinusOne) ) continue;
            else {
                DirectionType dirType = static_cast<DirectionType>( Jr2JmDirArray[dir] );
                if(cache->edge(x, y, dirType).MMVisitFlag != visit_counter &&
                    congestionMap2d->edge(x, y, dirType).lookupNet(net_id)) 
                {
                    cache->edge(x, y, dirType).MMVisitFlag = visit_counter;
                    Q.push(make_pair(x + dir_array[dir][0],
                                     y + dir_array[dir][1]));
                }
            }
        }
    }
}

Multisource_multisink_mazeroute::MMM_element* Multisource_multisink_mazeroute::find_min_cost_sink(vector<MMM_element*> &sinks)
{
    MMM_element* ret = nullptr;
    double cost = 1e9;
    int dist = 1e9;
    int via_num = 1e9;
    for (auto &s : sinks) {
        assert(s->dst == this->visit_counter);
        if (s->visit == this->visit_counter) {
            if (ret == nullptr) {
                ret = s;
                cost = s->reachCost;
                dist = s->distance;
                via_num = s->via_num;
            } else {
                if (smaller_than_lower_bound(s->reachCost, s->distance, s->via_num, cost, dist, via_num)) {
                    ret = s;
                    cost = s->reachCost;
                    dist = s->distance;
                    via_num = s->via_num;
                }
            }
        }
    }
    return ret;
}

void Multisource_multisink_mazeroute::col_sweep_p3(int col)
{
    int x = col + this->min_l, y;
    int numRows = this->max_t-this->min_b+1;
    // const int Jr2JmDirArray[4] = {0, 1, 3, 2};

    //forward
    for (int i=1; i<numRows; ++i) {
        y = i + this->min_b;
        auto cur_pos = &mmm_map->vertex(x, y-1);
        auto next_pos = &mmm_map->vertex(x, y);
        if (cur_pos->visit == this->visit_counter /*&& next_pos->walkableID == this->visit_counter && next_pos != cur_pos->parent*/) {
            double reachCost = cur_pos->reachCost;
            int total_distance = cur_pos->distance;
            int via_num = cur_pos->via_num;
            bool addDistance = false;
            bool needUpdate = false;
            int pre_dir;
            DirectionType dirType = static_cast<DirectionType>(0); //Jr2JmDirArray[0] = 0
            if (cur_pos->parent != NULL) {
                pre_dir = get_direction_2d_simple(cur_pos->coor, cur_pos->parent->coor);
            }
            if(cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).MMVisitFlag != this->visit_counter) {
                reachCost += cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).cost;
                ++total_distance;
                addDistance = true;
            }
            if( (cur_pos->parent != NULL) && ((0 & 0x02) != pre_dir) ) {
                via_num += 3;
            }
            if(next_pos->visit != this->visit_counter) {
                if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                            this->bound_cost, this->bound_dist, this->bound_via)) 
                {
                    needUpdate = true;
                }
            } else {
                if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                            next_pos->reachCost, next_pos->distance, next_pos->via_num)) 
                {
                    needUpdate = true;
                }
            }
            if (needUpdate) {
                next_pos->parent = cur_pos;
                next_pos->reachCost = reachCost;
                next_pos->distance = total_distance;
                next_pos->via_num = via_num;
                next_pos->visit = this->visit_counter;
            }
        }
    }

    //backward
    for (int i=numRows-2; i>=0; --i) {
        y = i + this->min_b;
        auto cur_pos = &mmm_map->vertex(x, y+1);
        auto next_pos = &mmm_map->vertex(x, y);
        if (cur_pos->visit == this->visit_counter /*&& next_pos->walkableID == this->visit_counter && next_pos != cur_pos->parent*/) {
            double reachCost = cur_pos->reachCost;
            int total_distance = cur_pos->distance;
            int via_num = cur_pos->via_num;
            bool addDistance = false;
            bool needUpdate = false;
            int pre_dir;
            DirectionType dirType = static_cast<DirectionType>(1); //Jr2JmDirArray[1] = 1
            if (cur_pos->parent != NULL) {
                pre_dir = get_direction_2d_simple(cur_pos->coor, cur_pos->parent->coor);
            }
            if(cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).MMVisitFlag != this->visit_counter) {
                reachCost += cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).cost;
                ++total_distance;
                addDistance = true;
            }
            if( (cur_pos->parent != NULL) && ((1 & 0x02) != pre_dir) ) {
                via_num += 3;
            }
            if(next_pos->visit != this->visit_counter) {
                if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                            this->bound_cost, this->bound_dist, this->bound_via)) 
                {
                    needUpdate = true;
                }
            } else {
                if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                            next_pos->reachCost, next_pos->distance, next_pos->via_num)) 
                {
                    needUpdate = true;
                }
            }
            if (needUpdate) {
                next_pos->parent = cur_pos;
                next_pos->reachCost = reachCost;
                next_pos->distance = total_distance;
                next_pos->via_num = via_num;
                next_pos->visit = this->visit_counter;
            }
        }
    }
}

void Multisource_multisink_mazeroute::row_sweep_p3(int row)
{
    int x, y = row + this->min_b;
    int numCols = this->max_r-this->min_l+1; 
    // const int Jr2JmDirArray[4] = {0, 1, 3, 2};
    //forward
    for (int j=1; j<numCols; ++j) {
        x = j + this->min_l;
        auto cur_pos = &mmm_map->vertex(x-1, y);
        auto next_pos = &mmm_map->vertex(x, y);
        if (cur_pos->visit == this->visit_counter /*&& next_pos->walkableID == this->visit_counter && next_pos != cur_pos->parent*/) {
            double reachCost = cur_pos->reachCost;
            int total_distance = cur_pos->distance;
            int via_num = cur_pos->via_num;
            bool addDistance = false;
            bool needUpdate = false;
            int pre_dir;
            DirectionType dirType = static_cast<DirectionType>(2); // Jr2JmDirArray[3] = 2
            if (cur_pos->parent != NULL) {
                pre_dir = get_direction_2d_simple(cur_pos->coor, cur_pos->parent->coor);
            }
            if(cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).MMVisitFlag != this->visit_counter) {
                reachCost += cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).cost;
                ++total_distance;
                addDistance = true;
            }
            if( (cur_pos->parent != NULL) && ((3 & 0x02) != pre_dir) ) {
                via_num += 3;
            }
            if(next_pos->visit != this->visit_counter) {
                if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                            this->bound_cost, this->bound_dist, this->bound_via)) 
                {
                    needUpdate = true;
                }
            } else {
                if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                            next_pos->reachCost, next_pos->distance, next_pos->via_num)) 
                {
                    needUpdate = true;
                }
            }
            if (needUpdate) {
                next_pos->parent = cur_pos;
                next_pos->reachCost = reachCost;
                next_pos->distance = total_distance;
                next_pos->via_num = via_num;
                next_pos->visit = this->visit_counter;
            }
        }
    }

    //backward
    for (int j=numCols-2; j>=0; --j) {
        x = j + this->min_l;
        auto cur_pos = &mmm_map->vertex(x+1, y);
        auto next_pos = &mmm_map->vertex(x, y);
        if (cur_pos->visit == this->visit_counter /*&& next_pos->walkableID == this->visit_counter && next_pos != cur_pos->parent*/) {
            double reachCost = cur_pos->reachCost;
            int total_distance = cur_pos->distance;
            int via_num = cur_pos->via_num;
            bool addDistance = false;
            bool needUpdate = false;
            int pre_dir;
            DirectionType dirType = static_cast<DirectionType>(3); // Jr2JmDirArray[2] = 3
            if (cur_pos->parent != NULL) {
                pre_dir = get_direction_2d_simple(cur_pos->coor, cur_pos->parent->coor);
            }
            if(cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).MMVisitFlag != this->visit_counter) {
                reachCost += cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).cost;
                ++total_distance;
                addDistance = true;
            }
            if( (cur_pos->parent != NULL) && ((2 & 0x02) != pre_dir) ) {
                via_num += 3;
            }
            if(next_pos->visit != this->visit_counter) {
                if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                            this->bound_cost, this->bound_dist, this->bound_via)) 
                {
                    needUpdate = true;
                }
            } else {
                if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                            next_pos->reachCost, next_pos->distance, next_pos->via_num)) 
                {
                    needUpdate = true;
                }
            }
            if (needUpdate) {
                next_pos->parent = cur_pos;
                next_pos->reachCost = reachCost;
                next_pos->distance = total_distance;
                next_pos->via_num = via_num;
                next_pos->visit = this->visit_counter;
            }
        }
    }
}

void Multisource_multisink_mazeroute::col_sweep_p2(int col)
{
    int x = col + this->min_l, y;
    int numRows = this->max_t-this->min_b+1;
    // const int Jr2JmDirArray[4] = {0, 1, 3, 2};

    //forward
    for (int i=1; i<numRows; ++i) {
        y = i + this->min_b;
        auto cur_pos = &mmm_map->vertex(x, y-1);
        auto next_pos = &mmm_map->vertex(x, y);
        if (cur_pos->visit == this->visit_counter /*&& next_pos->walkableID == this->visit_counter && next_pos != cur_pos->parent*/) {
            double reachCost = cur_pos->reachCost;
            int total_distance = cur_pos->distance;
            int via_num = cur_pos->via_num;
            bool addDistance = false;
            bool needUpdate = false;
            int pre_dir;
            DirectionType dirType = static_cast<DirectionType>(0); //Jr2JmDirArray[0] = 0
            if (cur_pos->parent != NULL) {
                pre_dir = get_direction_2d_simple(cur_pos->coor, cur_pos->parent->coor);
            }
            if(cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).MMVisitFlag != this->visit_counter) {
                reachCost += cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).cost;
                ++total_distance;
                addDistance = true;
            }
            if( (cur_pos->parent != NULL) && ((0 & 0x02) != pre_dir) ) {
                via_num += 3; 
                if (addDistance) {
                    total_distance += 3;
                    reachCost += via_cost;
                }
            }
            if(next_pos->visit != this->visit_counter) {
                if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                            this->bound_cost, this->bound_dist, this->bound_via)) 
                {
                    needUpdate = true;
                }
            } else {
                if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                            next_pos->reachCost, next_pos->distance, next_pos->via_num)) 
                {
                    needUpdate = true;
                }
            }
            if (needUpdate) {
                next_pos->parent = cur_pos;
                next_pos->reachCost = reachCost;
                next_pos->distance = total_distance;
                next_pos->via_num = via_num;
                next_pos->visit = this->visit_counter;
            }
        }
    }

    //backward
    for (int i=numRows-2; i>=0; --i) {
        y = i + this->min_b;
        auto cur_pos = &mmm_map->vertex(x, y+1);
        auto next_pos = &mmm_map->vertex(x, y);
        if (cur_pos->visit == this->visit_counter /*&& next_pos->walkableID == this->visit_counter && next_pos != cur_pos->parent*/) {
            double reachCost = cur_pos->reachCost;
            int total_distance = cur_pos->distance;
            int via_num = cur_pos->via_num;
            bool addDistance = false;
            bool needUpdate = false;
            int pre_dir;
            DirectionType dirType = static_cast<DirectionType>(1); //Jr2JmDirArray[1] = 1
            if (cur_pos->parent != NULL) {
                pre_dir = get_direction_2d_simple(cur_pos->coor, cur_pos->parent->coor);
            }
            if(cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).MMVisitFlag != this->visit_counter) {
                reachCost += cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).cost;
                ++total_distance;
                addDistance = true;
            }
            if( (cur_pos->parent != NULL) && ((1 & 0x02) != pre_dir) ) {
                via_num += 3; 
                if (addDistance) {
                    total_distance += 3;
                    reachCost += via_cost;
                }
            }
            if(next_pos->visit != this->visit_counter) {
                if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                            this->bound_cost, this->bound_dist, this->bound_via)) 
                {
                    needUpdate = true;
                }
            } else {
                if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                            next_pos->reachCost, next_pos->distance, next_pos->via_num)) 
                {
                    needUpdate = true;
                }
            }
            if (needUpdate) {
                next_pos->parent = cur_pos;
                next_pos->reachCost = reachCost;
                next_pos->distance = total_distance;
                next_pos->via_num = via_num;
                next_pos->visit = this->visit_counter;
            }
        }
    }
}

void Multisource_multisink_mazeroute::row_sweep_p2(int row)
{
    int x, y = row + this->min_b;
    int numCols = this->max_r-this->min_l+1; 
    // const int Jr2JmDirArray[4] = {0, 1, 3, 2};
    //forward
    for (int j=1; j<numCols; ++j) {
        x = j + this->min_l;
        auto cur_pos = &mmm_map->vertex(x-1, y);
        auto next_pos = &mmm_map->vertex(x, y);
        if (cur_pos->visit == this->visit_counter /*&& next_pos->walkableID == this->visit_counter && next_pos != cur_pos->parent*/) {
            double reachCost = cur_pos->reachCost;
            int total_distance = cur_pos->distance;
            int via_num = cur_pos->via_num;
            bool addDistance = false;
            bool needUpdate = false;
            int pre_dir;
            DirectionType dirType = static_cast<DirectionType>(2); // Jr2JmDirArray[3] = 2
            if (cur_pos->parent != NULL) {
                pre_dir = get_direction_2d_simple(cur_pos->coor, cur_pos->parent->coor);
            }
            if(cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).MMVisitFlag != this->visit_counter) {
                reachCost += cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).cost;
                ++total_distance;
                addDistance = true;
            }
            if( (cur_pos->parent != NULL) && ((3 & 0x02) != pre_dir) ) {
                via_num += 3; 
                if (addDistance) {
                    total_distance += 3;
                    reachCost += via_cost;
                }
            }
            if(next_pos->visit != this->visit_counter) {
                if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                            this->bound_cost, this->bound_dist, this->bound_via)) 
                {
                    needUpdate = true;
                }
            } else {
                if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                            next_pos->reachCost, next_pos->distance, next_pos->via_num)) 
                {
                    needUpdate = true;
                }
            }
            if (needUpdate) {
                next_pos->parent = cur_pos;
                next_pos->reachCost = reachCost;
                next_pos->distance = total_distance;
                next_pos->via_num = via_num;
                next_pos->visit = this->visit_counter;
            }
        }
    }

    //backward
    for (int j=numCols-2; j>=0; --j) {
        x = j + this->min_l;
        auto cur_pos = &mmm_map->vertex(x+1, y);
        auto next_pos = &mmm_map->vertex(x, y);
        if (cur_pos->visit == this->visit_counter /*&& next_pos->walkableID == this->visit_counter && next_pos != cur_pos->parent*/) {
            double reachCost = cur_pos->reachCost;
            int total_distance = cur_pos->distance;
            int via_num = cur_pos->via_num;
            bool addDistance = false;
            bool needUpdate = false;
            int pre_dir;
            DirectionType dirType = static_cast<DirectionType>(3); // Jr2JmDirArray[2] = 3
            if (cur_pos->parent != NULL) {
                pre_dir = get_direction_2d_simple(cur_pos->coor, cur_pos->parent->coor);
            }
            if(cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).MMVisitFlag != this->visit_counter) {
                reachCost += cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).cost;
                ++total_distance;
                addDistance = true;
            }
            if( (cur_pos->parent != NULL) && ((2 & 0x02) != pre_dir) ) {
                via_num += 3; 
                if (addDistance) {
                    total_distance += 3;
                    reachCost += via_cost;
                }
            }
            if(next_pos->visit != this->visit_counter) {
                if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                            this->bound_cost, this->bound_dist, this->bound_via)) 
                {
                    needUpdate = true;
                }
            } else {
                if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                            next_pos->reachCost, next_pos->distance, next_pos->via_num)) 
                {
                    needUpdate = true;
                }
            }
            if (needUpdate) {
                next_pos->parent = cur_pos;
                next_pos->reachCost = reachCost;
                next_pos->distance = total_distance;
                next_pos->via_num = via_num;
                next_pos->visit = this->visit_counter;
            }
        }
    }
}

bool Multisource_multisink_mazeroute::gamer_p2(vector<MMM_element*> &sinks)
{
    auto gamer_start = std::chrono::high_resolution_clock::now();
    
    const int Iter = 5;
    bool do_horizontal = false;
    int numRows = this->max_t-this->min_b+1;
    int numCols = this->max_r-this->min_l+1;
    #pragma omp parallel 
    {
        for (int iter = 0; iter<Iter; ++iter) {
            if (do_horizontal) {
                #pragma omp for schedule(guided) nowait
                for (int i=0; i<numRows; ++i) {
                    row_sweep_p2(i);
                }
            } else {
                #pragma omp for schedule(guided) nowait
                for (int j=0; j<numCols; ++j) {
                    col_sweep_p2(j);
                }
            }
            #pragma omp master
            {
                do_horizontal = !do_horizontal;
            }
            #pragma omp barrier
        }
    }
    auto best_sink = find_min_cost_sink(sinks);

    auto gamer_end = std::chrono::high_resolution_clock::now();
    GAMER_TIME += std::chrono::duration_cast<std::chrono::duration<double>>(gamer_end - gamer_start).count();

    if (best_sink != nullptr) {
        this->trace_back_to_find_path_2d(best_sink);
        this->adjust_twopin_element();
    }
    ++this->visit_counter;
	++this->dst_counter;
    return (best_sink != nullptr);
}

bool Multisource_multisink_mazeroute::gamer_p3(vector<MMM_element*> &sinks)
{
    auto gamer_start = std::chrono::high_resolution_clock::now();
    
    const int Iter = 5;
    bool do_horizontal = false;
    int numRows = this->max_t-this->min_b+1;
    int numCols = this->max_r-this->min_l+1;
    #pragma omp parallel
    {
        for (int iter = 0; iter<Iter; ++iter) {
            if (do_horizontal) {
                #pragma omp for schedule(guided) nowait
                for (int i=0; i<numRows; ++i) {
                    row_sweep_p3(i);
                }
            } else {
                #pragma omp for schedule(guided) nowait
                for (int j=0; j<numCols; ++j) {
                    col_sweep_p3(j);
                }
            }
            #pragma omp master
            {
                do_horizontal = !do_horizontal;
            }
            #pragma omp barrier
        }
    }
    auto best_sink = find_min_cost_sink(sinks);

    auto gamer_end = std::chrono::high_resolution_clock::now();
    GAMER_TIME += std::chrono::duration_cast<std::chrono::duration<double>>(gamer_end - gamer_start).count();

    if (best_sink != nullptr) {
        this->trace_back_to_find_path_2d(best_sink);
        this->adjust_twopin_element();
    }
    ++this->visit_counter;
	++this->dst_counter;
    return (best_sink != nullptr);
}

// use in main stage (mm maze routing)
bool Multisource_multisink_mazeroute::mm_maze_route_p2(Two_pin_element_2d *element,
        double bound_cost, int bound_distance, int bound_via_num,
        Coordinate_2d start, Coordinate_2d end)
{
    vector<MMM_element*> sinks;
    // this->min_l = this->min_b = INT_MAX;
    // this->max_r = this->max_t = INT_MIN;
    int pre_dir = 0;
    bool find_path_flag = false;
    MMM_element* cur_pos = NULL;
    MMM_element* next_pos = NULL;
    MMM_element* sink_pos = NULL;
    element->path.clear();
    this->element = element;
    this->boundary_l = start.x;
    this->boundary_b = start.y;
    this->boundary_r = end.x;
	this->boundary_t = end.y;
    this->bound_cost = bound_cost;
    this->bound_dist = bound_distance;
    this->bound_via = bound_via_num;
	this->setup_pqueue(sinks);
    this->putNetOnColorMap();
#ifdef GAMER
    assert(sinks.size() >= 1);
#endif

    for(int x = boundary_l; x <= boundary_r; ++x) {
        for(int y = boundary_b; y <= boundary_t; ++y) {
            mmm_map->vertex(x, y).walkableID = visit_counter;
        }
    }

#ifdef GAMER
    // this->min_l = min(this->min_l, boundary_l);
    // this->min_b = min(this->min_b, boundary_b);
    // this->max_r = max(this->max_r, boundary_r);
    // this->max_t = max(this->max_t, boundary_t);

    this->min_l = boundary_l;
    this->min_b = boundary_b;
    this->max_r = boundary_r;
    this->max_t = boundary_t;

    return gamer_p2(sinks);
#endif

double BACKTRACK_TIME;
auto path_search_start = std::chrono::high_resolution_clock::now();
	while(!this->pqueue->empty()) {
		cur_pos = pqueue->top();
        pqueue->pop();

        if (cur_pos->parent != NULL) {
            pre_dir = get_direction_2d_simple(cur_pos->coor, cur_pos->parent->coor);
            //We only need horizontal or vertical information of the direction,
            //so we can &0x02 first.
            //pre_dir &= 0x02;
        }

		for(int i = 3; i >= 0; --i) {
            if((i == 3 && cur_pos->coor->x >= gridxMinusOne) ||
               (i == 2 && cur_pos->coor->x <= 0)             ||
               (i == 1 && cur_pos->coor->y <= 0)             ||
               (i == 0 && cur_pos->coor->y >= gridyMinusOne) ) continue;
            else {
                next_pos = &mmm_map->vertex(cur_pos->coor->x + dir_array[i][0],
                                            cur_pos->coor->y + dir_array[i][1]);

                if(next_pos != cur_pos->parent && next_pos->walkableID == visit_counter) 
                {
                    DirectionType dirType = static_cast<DirectionType>( Jr2JmDirArray[i] );
                    double reachCost = cur_pos->reachCost;
                    int total_distance = cur_pos->distance;
                    int via_num = cur_pos->via_num;
                    bool addDistance = false;
                    if(cache->edge(cur_pos->coor->x,
                                cur_pos->coor->y, dirType).MMVisitFlag
                                    != this->visit_counter)
                    {
                        reachCost += cache->edge(cur_pos->coor->x,
                                                cur_pos->coor->y, dirType).cost;
                        ++total_distance;
                        addDistance = true;
                    }

                    if( ((i & 0x02) != pre_dir) && (cur_pos->parent != NULL) ){
                        via_num += 3; 
                        if (addDistance) {
                            total_distance += 3;
                            reachCost += via_cost;
                        }
                    }

                    bool needUpdate = false;
                    if(next_pos->visit != this->visit_counter) {
                        if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                                    bound_cost, bound_distance, bound_via_num)) 
                        {
                            needUpdate = true;
                        }
                    } else {
                        if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                                    next_pos->reachCost,
                                    next_pos->distance,
                                    next_pos->via_num)) 
                        {
                            needUpdate = true;
                        }
                    }

                    if(needUpdate == true) {
                        next_pos->parent = cur_pos;
                        next_pos->reachCost = reachCost;
                        next_pos->distance = total_distance;
                        next_pos->via_num = via_num;
                        next_pos->visit = this->visit_counter;

                        if(next_pos->dst == this->dst_counter)	
                        {
                            bound_cost = reachCost;
                            bound_distance = total_distance;
                            bound_via_num = via_num;
                            sink_pos = next_pos;
                        } else {
                            pqueue->update(next_pos);
                        }
                    }
                }
            }
		}//end of direction for-loop

		// if(sink_pos != NULL)	
		// {
		// 	find_path_flag = true;
        //     auto backtrack_start = std::chrono::high_resolution_clock::now();
		// 	this->trace_back_to_find_path_2d(sink_pos);
		// 	this->adjust_twopin_element();
        //     auto backtrack_end = std::chrono::high_resolution_clock::now();
        //     BACKTRACK_TIME = std::chrono::duration_cast<std::chrono::duration<double>>(backtrack_end - backtrack_start).count();
		// 	break;
		// }
	}
    if(sink_pos != NULL)	
    {
        find_path_flag = true;
        auto backtrack_start = std::chrono::high_resolution_clock::now();
        this->trace_back_to_find_path_2d(sink_pos);
        this->adjust_twopin_element();
        auto backtrack_end = std::chrono::high_resolution_clock::now();
        BACKTRACK_TIME = std::chrono::duration_cast<std::chrono::duration<double>>(backtrack_end - backtrack_start).count();
    }
auto path_search_end = std::chrono::high_resolution_clock::now();
PATH_SEARCH_TIME += (std::chrono::duration_cast<std::chrono::duration<double>>(path_search_end - path_search_start).count() - BACKTRACK_TIME);

	++this->visit_counter;
	++this->dst_counter;
	return find_path_flag;
}

// use in post processing (refinement stage)
bool Multisource_multisink_mazeroute::mm_maze_route_p3(Two_pin_element_2d *element,
        double bound_cost, int bound_distance, int bound_via_num,
        Coordinate_2d start, Coordinate_2d end)
{
    vector<MMM_element*> sinks;
    // this->min_l = this->min_b = INT_MAX;
    // this->max_r = this->max_t = INT_MIN;
    int pre_dir = 0;
    bool find_path_flag = false;
    MMM_element* cur_pos = NULL;
    MMM_element* next_pos = NULL;
    MMM_element* sink_pos = NULL;
    element->path.clear();
    this->element = element;
    this->boundary_l = start.x;
    this->boundary_b = start.y;
    this->boundary_r = end.x;
	this->boundary_t = end.y;
    this->bound_cost = bound_cost;
    this->bound_dist = bound_distance;
    this->bound_via = bound_via_num;
	this->setup_pqueue(sinks);
    this->putNetOnColorMap();
#ifdef GAMER
    assert(sinks.size() >= 1);
#endif

    for(int x = boundary_l; x <= boundary_r; ++x) {
        for(int y = boundary_b; y <= boundary_t; ++y) {
            mmm_map->vertex(x, y).walkableID = visit_counter;
        }
    }

#ifdef GAMER
    // this->min_l = min(this->min_l, boundary_l);
    // this->min_b = min(this->min_b, boundary_b);
    // this->max_r = max(this->max_r, boundary_r);
    // this->max_t = max(this->max_t, boundary_t);

    this->min_l = boundary_l;
    this->min_b = boundary_b;
    this->max_r = boundary_r;
    this->max_t = boundary_t;

    return gamer_p3(sinks);
    // return gamer(sinks, false);
#endif

double BACKTRACK_TIME;
auto path_search_start = std::chrono::high_resolution_clock::now();
	while(!this->pqueue->empty()) {
		cur_pos = pqueue->top();
        pqueue->pop();

        if (cur_pos->parent != NULL) {
            pre_dir = get_direction_2d_simple(cur_pos->coor, cur_pos->parent->coor);
            //We only need horizontal or vertical information of the direction,
            //so we can &0x02 first.
            //pre_dir &= 0x02;
        }

		for(int i = 3; i >= 0; --i) {
            if((i == 3 && cur_pos->coor->x >= gridxMinusOne) ||
               (i == 2 && cur_pos->coor->x <= 0)             ||
               (i == 1 && cur_pos->coor->y <= 0)             ||
               (i == 0 && cur_pos->coor->y >= gridyMinusOne) ) continue;
            else {
            next_pos = &mmm_map->vertex(cur_pos->coor->x + dir_array[i][0],
                                        cur_pos->coor->y + dir_array[i][1]);

            if(next_pos != cur_pos->parent && 
               next_pos->walkableID == visit_counter) 
            {
                DirectionType dirType = static_cast<DirectionType>( Jr2JmDirArray[i] );
                double reachCost = cur_pos->reachCost;
                int total_distance = cur_pos->distance;
                int via_num = cur_pos->via_num;

                if( (cache->edge(cur_pos->coor->x, 
                                 cur_pos->coor->y, dirType).MMVisitFlag != this->visit_counter) &&
                    (sign(cache->edge(cur_pos->coor->x, 
                                  cur_pos->coor->y, dirType).cost) != 0) )
                {
                    reachCost += cache->edge(
                                        cur_pos->coor->x, 
                                        cur_pos->coor->y, dirType).cost;
                    ++total_distance;
                }

                if( ((i & 0x02) != pre_dir) && (cur_pos->parent != NULL) ){
                    via_num += 3; 
                }

                bool needUpdate = false;
                if(next_pos->visit != this->visit_counter) {
                    if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                                bound_cost, bound_distance, bound_via_num)) 
                    {
                        needUpdate = true;
                    }
                } else {
                    if(smaller_than_lower_bound(reachCost, total_distance, via_num,
                                next_pos->reachCost,
                                next_pos->distance,
                                next_pos->via_num)) 
                    {
                        needUpdate = true;
                    }
                }

                if(needUpdate == true) {
                    next_pos->parent = cur_pos;
                    next_pos->reachCost = reachCost;
                    next_pos->distance = total_distance;
                    next_pos->via_num = via_num;
                    next_pos->visit = this->visit_counter;

                    if(next_pos->dst == this->dst_counter)	
                    {
                        bound_cost = reachCost;
			            // if (element->net_id == 892)
                        //     cout << "In maze route bound_cost changes to " << bound_cost << '\n';
                        bound_distance = total_distance;
                        bound_via_num = via_num;
                        sink_pos = next_pos;
                    } else {
                        pqueue->update(next_pos);
                    }
                }
            }
            }
		}//end of direction for-loop

		// if(sink_pos != NULL)	
		// {
		// 	find_path_flag = true;
        //     auto backtrack_start = std::chrono::high_resolution_clock::now();
		// 	this->trace_back_to_find_path_2d(sink_pos);
		// 	this->adjust_twopin_element();
        //     auto backtrack_end = std::chrono::high_resolution_clock::now();
        //     BACKTRACK_TIME = std::chrono::duration_cast<std::chrono::duration<double>>(backtrack_end - backtrack_start).count();
		// 	break;
		// }
	}
    if(sink_pos != NULL)	
    {
        find_path_flag = true;
        auto backtrack_start = std::chrono::high_resolution_clock::now();
        this->trace_back_to_find_path_2d(sink_pos);
        this->adjust_twopin_element();
        auto backtrack_end = std::chrono::high_resolution_clock::now();
        BACKTRACK_TIME = std::chrono::duration_cast<std::chrono::duration<double>>(backtrack_end - backtrack_start).count();
    }
auto path_search_end = std::chrono::high_resolution_clock::now();
PATH_SEARCH_TIME += (std::chrono::duration_cast<std::chrono::duration<double>>(path_search_end - path_search_start).count() - BACKTRACK_TIME);

	++this->visit_counter;
	++this->dst_counter;
	return find_path_flag;
}

/****************
  MMMPriortyQueue
 ***************/
Multisource_multisink_mazeroute::MMMPriortyQueue::MMMPriortyQueue ()
:storage_(NULL)
{  
    init();
}

Multisource_multisink_mazeroute::MMMPriortyQueue::~MMMPriortyQueue ()
{
    close();
}

void Multisource_multisink_mazeroute::MMMPriortyQueue::close()
{
    if(storage_ != NULL) {
        for( int ite = size_ - 1; ite >= 0; --ite)
        {
            (*storage_)[ite]->pqIndex = -1;
        }
        size_ = 0;
        delete storage_;
    }
}

void Multisource_multisink_mazeroute::MMMPriortyQueue::pop()
{
    assert(empty() == false);

    int storageSizeMinusOne = size_ - 1;

    (*storage_)[0]->pqIndex = -1;

    if(0 < storageSizeMinusOne) {
        (*storage_)[0] = (*storage_)[storageSizeMinusOne];
        (*storage_)[0]->pqIndex = 0;
    }

    int parentIndex = 0;
    while(parentIndex < storageSizeMinusOne) {
        register int LChildIndex = (parentIndex << 1) + 1;

        if( LChildIndex >= storageSizeMinusOne) break;

        int updateIndex = LChildIndex;
        register int RChildIndex = LChildIndex + 1;

        if(RChildIndex >= storageSizeMinusOne) {
            if( !compareMMM((*storage_)[LChildIndex], (*storage_)[parentIndex]) ) {
                break;
            }
        } else {
            if( !compareMMM((*storage_)[LChildIndex], (*storage_)[RChildIndex]) ) {
                updateIndex = RChildIndex;
            } 
        }

        swap( (*storage_)[parentIndex], (*storage_)[updateIndex] );
        (*storage_)[parentIndex]->pqIndex = parentIndex;
        (*storage_)[updateIndex]->pqIndex = updateIndex;

        parentIndex = updateIndex;
    }

    //Remove the last node from the array
    --size_;
}

void Multisource_multisink_mazeroute::MMMPriortyQueue::update(int indexToUpdate)
{
    while(indexToUpdate > 0) {
        int parsentIndex = (indexToUpdate-1) >> 1;
        if( compareMMM((*storage_)[parsentIndex], (*storage_)[indexToUpdate]) ) break;
        swap( (*storage_)[parsentIndex], (*storage_)[indexToUpdate] );
        (*storage_)[parsentIndex]->pqIndex = parsentIndex;
        (*storage_)[indexToUpdate]->pqIndex = indexToUpdate;
        indexToUpdate = parsentIndex;
    }
}
