#include "CoalaDB.h"
#include <queue>
#include <stack>
#include <omp.h>
#include <chrono>
#include <random>
// from nthu-route old layerassignment
class pathNode
{
public:
    pathNode(int value) : val(value) {}
    int val;
    bool inSeg = false;
};
// (netIdx,pin) pair on each layers
void CoalaDB::CoalaDataBase::initialLayerPinList(int netNumber)
{
    for (int i = 0; i < netNumber; ++i)
    {
        const PinptrList *pin_list = &rr_map->get_nPin(i);
        for (const auto &pin : *pin_list)
        {
            layerPinList[pin->get_layerId()].emplace_back(std::make_pair(i, pin));
        }
    }
}
bool CoalaDB::CoalaDataBase::coordinate3DValid(const Jm::Coordinate_3d &p, int factor)
{
    if (p.x < 0 || p.x > (rr_map->get_gridx() - 1) / factor ||
        p.y < 0 || p.y > (rr_map->get_gridy() - 1) / factor ||
        p.z < 0 || p.z > rr_map->get_layerNumber() - 1)
        return false;
    return true;
}
bool CoalaDB::CoalaDataBase::coordinate2DValid(const Jm::Coordinate_2d &p)
{
    if (p.x < 0 || p.x > rr_map->get_gridx() - 1 ||
        p.y < 0 || p.y > rr_map->get_gridy() - 1)
        return false;
    return true;
}
bool CoalaDB::CoalaDataBase::segmentValid(const Jm::Segment_2d &s)
{
    if (s.layer < 0 || s.layer > rr_map->get_layerNumber() - 1 ||
        !coordinate2DValid(s.p1) || !coordinate2DValid(s.p2))
        return false;
    return true;
}
void CoalaDB::CoalaDataBase::setDataBase()
{
    int x_size = rr_map->get_gridx();
    int y_size = rr_map->get_gridy();
    int layer_size = rr_map->get_layerNumber();
    int net_size = rr_map->get_netNumber();
    coalamap.resize(x_size * y_size);
    layerPinList.resize(layer_size);
    netSegs.resize(net_size);
    netSegsNum.resize(net_size);
    viaUsage.resize(x_size * y_size * layer_size, 0);
    pointPlaced.resize(net_size);
    initialLayerPinList(net_size);
    std::vector<std::vector<int>> tile_cap_map_x;
    std::vector<std::vector<int>> tile_cap_map_y;
    tile_cap_map_x.resize(x_size, std::vector<int>(y_size, 0));
    tile_cap_map_y.resize(x_size, std::vector<int>(y_size, 0));
    // calculate 2d tile capacity
    for (int i = x_size - 2; i >= 0; --i)
    {
        for (int j = y_size - 1; j >= 0; --j)
        {

            int i_j_2d_cap_x = congestionMap2d->edge(i, j, Jm::DIR_EAST).cur_cap;
            tile_cap_map_x[i][j] += i_j_2d_cap_x;
            tile_cap_map_x[i + 1][j] += i_j_2d_cap_x;
        }
    }

    for (int i = x_size - 1; i >= 0; --i)
    {
        for (int j = y_size - 2; j >= 0; --j)
        {

            int i_j_2d_cap_y = congestionMap2d->edge(i, j, Jm::DIR_NORTH).cur_cap;
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
            if (layer != 0)
                cur_map_3d[x][y][layer].cap -= 2;
            cur_map_3d[x][y][layer + 1].cap -= 2;
            cur_map_3d[x][y][layer + 2].cap -= 2;
        }
    }
    std::cout << " get net path ! \n";
    // initial net path with BFS and break it into segments
    getNetPath();
}
bool compare2DPoints(const Jm::Coordinate_2d &p1, const Jm::Coordinate_2d &p2)
{
    if (p1.x == p2.x)
    {
        return p1.y < p2.y;
    }
    else
    {
        return p1.x < p2.x;
    }
}

void CoalaDB::CoalaDataBase::getNetPath()
{
    int x_size = rr_map->get_gridx();
    int y_size = rr_map->get_gridy();
    int layer_size = rr_map->get_layerNumber();
    // borrow from old layerassignment preprocess
    const int dir_2d[4][2] = {{0, 1}, {0, -1}, {-1, 0}, {1, 0}};
    int net_num = rr_map->get_netNumber();
    //#pragma omp parallel for
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::vector<std::vector<pathNode>> path_map;
    path_map.resize(x_size, std::vector<pathNode>(y_size, pathNode(0)));
    std::vector<pathNode *> visited;
    for (int i = 0; i < net_num; ++i)
    {
        netPinBound.emplace_back(std::vector<int>({INT_MAX, INT_MIN, INT_MAX, INT_MIN, INT_MAX, INT_MIN}));
        Jm::Coordinate_2d *current;
        int pin_num = rr_map->get_netPinNumber(i);
        const PinptrList *pin_list = &rr_map->get_nPin(i);
        std::stack<Jm::Coordinate_2d *> q;
        std::unordered_map<int, int> pinLayer;
        std::vector<std::shared_ptr<Jm::Segment_2d>> curSeg;
        for (int p = 0; p < pin_num; ++p)
        {
            int px = (*pin_list)[p]->get_tileX();
            int py = (*pin_list)[p]->get_tileY();
            int pz = (*pin_list)[p]->get_layerId();
            //srand( time(NULL) ); //Randomize seed initialization
	        //int randNum = rand() % 2; 
            //auto pinSeg = make_shared<Jm::Segment_2d>(i, Jm::Coordinate_2d(px, py), Jm::Coordinate_2d(px, py), static_cast<Jm::SegmentDir>(randNum));
            //curSeg.emplace_back(pinSeg);
            path_map[px][py].val = -2;
            pinLayer[Coala2Dto1D(px, py)] = pz;
            visited.emplace_back(&path_map[px][py]);
            if (px < netPinBound[i][0])
                netPinBound[i][0] = px;
            if (px > netPinBound[i][1])
                netPinBound[i][1] = px;
            if (py < netPinBound[i][2])
                netPinBound[i][2] = py;
            if (py > netPinBound[i][3])
                netPinBound[i][3] = py;
            if (pz < netPinBound[i][4])
                netPinBound[i][4] = pz;
            if (pz > netPinBound[i][5])
                netPinBound[i][5] = pz;
        }
        int x = (*pin_list)[0]->get_tileX();
        int y = (*pin_list)[0]->get_tileY();
        path_map[x][y].val = 2;
        q.push(&coor_array[x][y]);
        std::vector<std::shared_ptr<Jm::Segment_2d>> finalSeg;
        auto start = make_shared<Jm::Segment_2d>(i, Jm::Coordinate_2d(x, y), Jm::Coordinate_2d(x, y), Jm::None);
        start->sortPoint();
        curSeg.emplace_back(start);
        std::shared_ptr<Jm::Segment_2d> lastSeg = start;
        Jm::Coordinate_2d lastPoint = Jm::Coordinate_2d(-1, -1);
        int lastPointNeighborCount = 0;
        while (!q.empty())
        {
            current = q.top();
            q.pop();
            bool success_add = false;
            if (lastPoint.x >= 0)
            {
                if (lastPointNeighborCount == 1)
                {
                    if (path_map[lastPoint.x][lastPoint.y].val == 2 && lastSeg->length() > 1)
                    {
                        success_add = false;
                    }
                    else if (lastSeg->p1.L1Distance(*current) == 1)
                        success_add = lastSeg->addPoint(lastSeg->p1, *current);
                    else if (lastSeg->p2.L1Distance(*current) == 1)
                        success_add = lastSeg->addPoint(lastSeg->p2, *current);
                    else
                        success_add = false;
                }
            }
            else
            {
                success_add = true;
            }
            lastPointNeighborCount = 0;
            if (!success_add)
            {
                for (int d = 0; d < 4; ++d)
                {
                    int cx = current->x + dir_2d[d][0];
                    int cy = current->y + dir_2d[d][1];
                    if (cx >= 0 && cx < x_size && cy >= 0 && cy < y_size && congestionMap2d->edge(current->x, current->y, d).lookupNet(i))
                    {
                        if (path_map[cx][cy].inSeg == true)
                        {
                            if (d < 2)
                                lastSeg = make_shared<Jm::Segment_2d>(i, Jm::Coordinate_2d(cx, cy), Jm::Coordinate_2d(current->x, current->y), Jm::Vertical);
                            else
                                lastSeg = make_shared<Jm::Segment_2d>(i, Jm::Coordinate_2d(cx, cy), Jm::Coordinate_2d(current->x, current->y), Jm::Horizontal);

                            lastSeg->sortPoint();
                            curSeg.emplace_back(lastSeg);
                            break;
                        }
                    }
                }
            }
            lastPoint = *current;
            path_map[current->x][current->y].inSeg = true;
            for (int d = 0; d < 4; ++d)
            {
                x = current->x + dir_2d[d][0];
                y = current->y + dir_2d[d][1];
                if (x >= 0 && x < x_size && y >= 0 && y < y_size && congestionMap2d->edge(current->x, current->y, d).lookupNet(i))
                {
                    if (path_map[x][y].val == 0 || path_map[x][y].val == -2)
                    {
                        lastPointNeighborCount++;
                        if (path_map[x][y].val == 0)
                        {
                            path_map[x][y].val = 1;
                        }
                        else
                        {
                            path_map[x][y].val = 2;
                        }
                        visited.emplace_back(&path_map[x][y]);
                        q.push(&coor_array[x][y]);
                    }
                }
            }
        }
        while (!visited.empty())
        {
            auto cur = visited.back();
            cur->val = 0;
            cur->inSeg = false;
            visited.pop_back();
        }

        for (auto &s : curSeg)
        {
            s->sortPoint();
            finalSeg.emplace_back(s);
        }
        for (auto &s : curSeg)
        {
            setNewSeg(s);
        }
        netSegsNum[i] = netSegs[i].size();
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::cout << "getNetPath took "
              << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count()
              << "us.\n";
}
void CoalaDB::CoalaDataBase::setNewSeg(std::shared_ptr<Jm::Segment_2d> newSeg)
{
    netSegs[newSeg->netIdx].emplace_back(newSeg);
    coalamap[Coala2Dto1D(newSeg->p1.x, newSeg->p1.y)].segments.emplace_back(newSeg);
    coalamap[Coala2Dto1D(newSeg->p2.x, newSeg->p2.y)].segments.emplace_back(newSeg);
    return;
}
// from old nthu layer assignment
