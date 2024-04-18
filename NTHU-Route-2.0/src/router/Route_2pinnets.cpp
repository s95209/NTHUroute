#include "Route_2pinnets.h"
#include "Range_router.h"
#include "Construct_2d_tree.h"
#include "flute/flute-ds.h"
#include "misc/geometry.h"
#include "util/traversemap.h"

#include "cmath"
#include "algorithm"

using namespace Jm;
using namespace std;

int two_pin_list_size = 0;

VertexPlane<Point_fc>* gridcell;//This is some kind of color map, for recording
                               //which 2-pin net passed which gCell

static VertexColorMap<int>* traverseMap;   //This color map is used by 
                                //bfs_for_find_two_pin_list()
                                //for recording which tiles are traversed

static VertexColorMap<int>* terminalMap;  //This color map is used by
                               //determine_is_terminal_or_steiner_point()
                               //for recording which tiles contains terminal

static const int dirTransferTable[4] = {1, 0, 3, 2}; //FRONT <-> BACK, LEFT <-> RIGHT
                                //Used by determine_is_terminal_or_steiner_point()

extern int sign(double x);

void allocate_gridcell()
{
    gridcell = new VertexPlane<Point_fc>(rr_map->get_gridx(),
                                          rr_map->get_gridy(),
                                          Point_fc());
    for(int x = rr_map->get_gridx() - 1; x >=0; --x) {
        for(int y = rr_map->get_gridy() - 1; y >=0; --y) {
            gridcell->vertex(x, y).x = x;
            gridcell->vertex(x, y).y = y;
        }
    }
	
#ifdef MESSAGE
	printf("initialize gridcell successfully\n");
#endif
}

static void init_gridcell()
{
	for (int i=0; i<rr_map->get_gridx();++i) {
		for (int j=0; j<rr_map->get_gridy(); ++j)
		{
			gridcell->vertex(i, j).points.clear();
		}
    }
	int length = two_pin_list.size();
	for (int i=0; i < length; ++i)
	{
		//add pin1
		int cur_x = two_pin_list[i]->pin1.x;
		int cur_y = two_pin_list[i]->pin1.y;
        gridcell->vertex(cur_x, cur_y).points.push_back(two_pin_list[i]);
		//add pin2
		cur_x = two_pin_list[i]->pin2.x;
		cur_y = two_pin_list[i]->pin2.y;
        gridcell->vertex(cur_x, cur_y).points.push_back(two_pin_list[i]);
	}
}

void route_all_2pin_net(void )
{
	vector<Point_fc *> all_cells;
	
	init_gridcell(); 
	
	define_interval();
	divide_grid_edge_into_interval();
	
	specify_all_range();
}

static void reset_c_map_used_net_to_one()
{
	RoutedNetTable::iterator iter;
	
	for (int i = rr_map->get_gridx()-2; i >= 0; --i)
    {
		for (int j = rr_map->get_gridy() - 1; j >= 0; --j)
		{
            RoutedNetTable* table = (congestionMap2d->edge(i, j, DIR_EAST).used_net);
			if(table != nullptr){
				for(iter = table->begin(); iter != table->end(); ++iter) {
					(*table)[iter->first] = 1;
				}
			}
		}
    }

	for (int i = rr_map->get_gridx()-1; i >= 0; --i)
    {
		for (int j = rr_map->get_gridy() - 2; j >= 0; --j)
		{
            RoutedNetTable* table = (congestionMap2d->edge(i, j, DIR_NORTH).used_net);
			if(table != nullptr){
				for(iter = table->begin(); iter != table->end(); ++iter) {
					(*table)[iter->first] = 1;
				}
			}
		}
    }
}

//set terminal type to c_map_2d
//void set_c_map_terminal_type(int net_id)
static void put_terminal_color_on_colormap(int net_id)
{
	for (int i = rr_map->get_nPin(net_id).size() -1; i >= 0; --i) {
		int xx = rr_map->get_nPin(net_id)[i]->get_tileX();
		int yy = rr_map->get_nPin(net_id)[i]->get_tileY();
        terminalMap->color(xx, yy) = net_id;
	}
}

// one-degree terminal: -4, non-one-degree terminal: -3, 
// one-degree nontermina: -2, steiner point: find_dir, more than two-degree: -1
static int determine_is_terminal_or_steiner_point(int xx, int yy, int dir, int net_id)
{
	int find_dir = 0;
    static int gridxMinusOne = rr_map->get_gridx() - 1;
    static int gridyMinusOne = rr_map->get_gridy() - 1;

    dir = dirTransferTable[dir]; // 0:BACK 1:FRONT 2:RIGHT 3:LEFT
	
    if(terminalMap->color(xx, yy) == net_id) {  
        for (int i = 3; i >= 0; --i) {
            if (i != dir)  
            {
                 if((i == 3 && xx >= gridxMinusOne) || 
                    (i == 2 && xx <= 0)             ||
                    (i == 1 && yy <= 0)             ||
                    (i == 0 && yy >= gridyMinusOne) ) continue;
                 else
                {
                    if(congestionMap2d->edge(xx, yy, i).lookupNet(net_id))
                    {
                        return -3;
                    }
                }
            }
        }
        return -4;  
    }else {
        int other_passed_edge = 0;
        for (int i = 3; i >= 0; --i) {
            if (i != dir)  
            {
                 if((i == 3 && xx >= gridxMinusOne) || 
                    (i == 2 && xx <= 0)             ||
                    (i == 1 && yy <= 0)             ||
                    (i == 0 && yy >= gridyMinusOne) ) continue;
                 else
                {
                    if(congestionMap2d->edge(xx, yy, i).lookupNet(net_id))
                    {
                        ++other_passed_edge;				
                        find_dir = i;
                        if((other_passed_edge & 0x02) != 0) return -1;
                    }
                }
            }
        }

        if(other_passed_edge == 0)
            return -2;      
        else return find_dir;   
    }
}

void bfs_for_find_two_pin_list(Coordinate_2d *start_coor, int net_id, bool insert_to_branch)
{
	vector<Coordinate_2d *> queue;
	vector<int> parent;
	vector<int> branch_ind;
	vector<Coordinate_2d *> branch_xy;
	vector<int> branch_n;
	int head_index,tail_index,dir,ori_dir;
	int x,y,xx,yy,i;

	queue.push_back(start_coor);
	parent.push_back(-1);
	head_index=0;
	tail_index=1;

    traverseMap->color(queue[head_index]->x, queue[head_index]->y) = net_id;

	if (insert_to_branch)
	{
		branch_xy.push_back(queue[head_index]);
		branch_n.push_back(0);
		branch_ind.push_back(0);
	}

	while (head_index != tail_index)
	{
		// 0:FRONT 1:BACK 2:LEFT 3:RIGHT
		for (i = 0; i <= 3; ++i)
		{
			if (i == parent[head_index])
				continue;
			x = queue[head_index]->x;
			y = queue[head_index]->y;
			dir = i;

			//是否還在範圍內
			if (/*x*/ x+dir_array[dir][0] >= 0 && x+dir_array[dir][0] < rr_map->get_gridx() &&
                /*y*/ y+dir_array[dir][1] >= 0 && y+dir_array[dir][1] < rr_map->get_gridy() )
			{
				// 這條net是否經過後這個tile
                if(congestionMap2d->edge(x, y, dir).lookupNet(net_id))
				{
					Two_pin_element_2d* two_pin;
					// two_pin_list_size: 
					if(two_pin_list_size >= (int)two_pin_list.size())
					{
						two_pin = new Two_pin_element_2d();
					}else{
						two_pin = two_pin_list[two_pin_list_size];
                    }
					two_pin->pin1.x = queue[head_index]->x;
					two_pin->pin1.y = queue[head_index]->y;
					two_pin->net_id = net_id;
					two_pin->path.clear();
					two_pin->path.push_back(queue[head_index]);
					while(1)
					{
                        traverseMap->color(x, y) = net_id;
						xx = x+dir_array[dir][0];
						yy = y+dir_array[dir][1];
						ori_dir = dir;
						dir = determine_is_terminal_or_steiner_point(xx,yy,dir,net_id);
						two_pin->path.push_back(&coor_array[xx][yy]);

						// one-degree terminal: -4, non-one-degree terminal: -3, 
						// one-degree nonterminal: -2, steiner point: find_dir, more than two-degree: -1

						if (dir<0 && dir!=-2) // 如果是 terminal 或 more than two-degree nontermina
						{
							if (traverseMap->color(xx, yy) != net_id) // 如果未被這個NET標記
							{
								two_pin->pin2.x = xx;
								two_pin->pin2.y = yy;
								if (two_pin_list_size >= (int)two_pin_list.size()){
									two_pin_list.push_back(two_pin);
								}
								two_pin_list_size++;
								
								if(insert_to_branch)
								{
									branch_xy.push_back(&coor_array[xx][yy]);
									branch_n.push_back(branch_ind[head_index]);
								}
								
								if (dir!=-4)  // more than two-degree nonterminal: -1 non-one-degree terminal: -3,
								{
									queue.push_back(&coor_array[xx][yy]);
									if (ori_dir==FRONT)
										parent.push_back(BACK);
									else if (ori_dir==BACK)
										parent.push_back(FRONT);
									else if (ori_dir==RIGHT)
										parent.push_back(LEFT);
									else parent.push_back(RIGHT);
									
									if (insert_to_branch)
										branch_ind.push_back(branch_xy.size()-1);
										
									tail_index++;
								}
                                traverseMap->color(xx, yy) = net_id;
							}else //其他branch已經走過了會有cycle所以步行走
							{
								update_congestion_map_remove_two_pin_net(two_pin);
                                NetDirtyBit[two_pin->net_id] = true;
								if (two_pin_list_size>=(int)two_pin_list.size())
								{
									two_pin->path.clear();
									delete(two_pin);
								}
							}
							break;
						}
						else if (dir==-2)
						{
							update_congestion_map_remove_two_pin_net(two_pin);
                            NetDirtyBit[two_pin->net_id] = true;
							if (two_pin_list_size >= (int)two_pin_list.size())
							{
								two_pin->path.clear();
								delete(two_pin);
							}
							break;
						}
						else
						{
							x = xx;
							y = yy;
						}
					}
				}
			}
		}
		head_index++;
	}

	if (insert_to_branch)
	{
		if (net_flutetree[net_id].number < (int)branch_xy.size())
		{
			free(net_flutetree[net_id].branch);
			net_flutetree[net_id].branch = (Branch *) malloc(branch_xy.size()*sizeof(Branch));
		}

		net_flutetree[net_id].number = branch_xy.size();
		for (i = net_flutetree[net_id].number - 1; i >= 0; --i)
		{
			net_flutetree[net_id].branch[i].x = branch_xy[i]->x;
			net_flutetree[net_id].branch[i].y = branch_xy[i]->y;
			net_flutetree[net_id].branch[i].n = branch_n[i];
		}
	}
}

void reallocate_two_pin_list(bool insert_to_branch)
{
    int dirty_count = 0;
    traverseMap = new VertexColorMap<int>(rr_map->get_gridx(), rr_map->get_gridy(), -1);
    terminalMap = new VertexColorMap<int>(rr_map->get_gridx(), rr_map->get_gridy(), -1);
	
	reset_c_map_used_net_to_one();

	two_pin_list_size = 0;


	// 把還沒繞的線放到two_pin_list的前面
    int usedTwoPinListSize = 0;
	for (int twoPinListPos = 0; twoPinListPos < (int)two_pin_list.size(); ++twoPinListPos) {
        if( NetDirtyBit[ two_pin_list[twoPinListPos]->net_id ] == false ) //false 這個net還沒被繞好
		{
            if(usedTwoPinListSize != twoPinListPos) {
                swap(two_pin_list[twoPinListPos], two_pin_list[usedTwoPinListSize]);
                ++usedTwoPinListSize;
            } else {
                ++usedTwoPinListSize;
            }
        }
    }
    
	two_pin_list_size = usedTwoPinListSize;

	for (int netId = 0; netId < rr_map->get_netNumber(); ++netId) {
        if( NetDirtyBit[netId] == true ) {
            put_terminal_color_on_colormap(netId); // 把net_id填到 "terminalMap" 上對應他pin的tile上
            int xx = rr_map->get_nPin(netId)[0]->get_tileX();	//(netId)[0] pin的x
            int yy = rr_map->get_nPin(netId)[0]->get_tileY();	//(netId)[0] pin的y
            Coordinate_2d* start_coor = &coor_array[xx][yy];

            bfs_for_find_two_pin_list(start_coor, netId, insert_to_branch);

            NetDirtyBit[netId] = false;
            ++dirty_count;
        }
	}

	for (int i = two_pin_list.size()-1; i >= two_pin_list_size; --i)
	{
		delete(two_pin_list[i]);
		two_pin_list.pop_back();
	}

	// added
	for(int i = two_pin_list.size() -1; i >= 0; --i)
	{
		two_pin_list[i]->path_through_zero_edge = false;
		for(int j = two_pin_list[i]->path.size()-1; j > 0; --j)
		{
			int x_dir = two_pin_list[i]->path[j-1]->x - two_pin_list[i]->path[j]->x;
			int y_dir = two_pin_list[i]->path[j-1]->y - two_pin_list[i]->path[j]->y;
			int edge_idx;
			if(x_dir)
				edge_idx = (x_dir == 1) ? RIGHT : LEFT;
			else	// y_dir
				edge_idx = (y_dir == 1) ? FRONT : BACK;
			int x = two_pin_list[i]->path[j]->x;
			int y = two_pin_list[i]->path[j]->y;
			if (!two_pin_list[i]->path_through_zero_edge && sign(congestionMap2d->edge(x, y, edge_idx).max_cap) == 0)
				two_pin_list[i]->path_through_zero_edge = true;
		}
	}

    delete traverseMap;
    delete terminalMap;
}
