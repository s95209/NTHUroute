#include "CoalaReroute.h"

#include <limits>

#include <chrono>

#include <algorithm>

using openListStatus = std::unordered_map<int, std::vector<std::shared_ptr<Coala::AstarNode>>>;

void Coala::CoalaReroute::capacityRecovery(std::shared_ptr<Jm::Segment_2d> target)

{

    if (target->dir == Jm::Horizontal)

    {

        int start = target->p1.x;

        int end = target->p2.x;

        for (int i = start; i <= end; ++i)

        {

            if (i != start && i != end)

            {
                cur_map_3d[i][target->p1.y][target->layer].cap += 2;
            }

            else

            {

                cur_map_3d[i][target->p1.y][target->layer].cap++;
            }
        }
    }

    else

    {

        int start = target->p1.y;

        int end = target->p2.y;

        for (int i = start; i <= end; ++i)

        {

            if (i != start && i != end)

            {

                cur_map_3d[target->p1.x][i][target->layer].cap += 2;

            }

            else

            {

                cur_map_3d[target->p1.x][i][target->layer].cap++;
            }
        }
    }
}

void Coala::CoalaReroute::deleteSegFromFinalPath(std::shared_ptr<Jm::Segment_2d> target, std::vector<std::vector<std::shared_ptr<Jm::Segment_2d>>> &fp)

{

    auto &finalSeg = fp[target->netIdx];

    auto it = finalSeg.begin();

    while (it != finalSeg.end())

    {

        if ((*it) == target)

        {

            it = finalSeg.erase(it);
        }

        else

            it++;
    }

    return;
}

void Coala::CoalaReroute::viaRecovery(Jm::Coordinate_2d &p, int netIdx, std::vector<std::unordered_map<int, std::vector<bool>>> &viasMap, int layer, bool t)

{
    auto coord = coalaDB.Coala2Dto1D(p.x, p.y);

    auto &viaMap = viasMap[netIdx][coord];
    if (viaMap.empty())
    {
        return;
    }

    int start = layer;
    int end = layer;
    bool deleteVia = true;
    int rem = -1;
    for (int j = rr_map->get_layerNumber() - 1; j > layer; j--)
    {
        if (viaMap[j])
        {
            if (t)
                end = j;
            rem = j;
            break;
        }
    }
    while (start <= end)
    {
        if (t)
            viasMap[netIdx][coord][start] = false;
        int via_cap = (start < 5) ? 1 : 3;
        //cur_map_3d[p.x][p.y][start].cap += via_cap;
        start++;
    }
    /*
    if (viasMap.size() > 1)
    {
        std::sort(viaMap.begin(), viaMap.end());
        int start = layer;
        int end = (t == true) ? viasMap[netIdx][coord].back() : layer;
        bool first = true;
        while (start <= end)
        {
            auto position = std::find(viaMap.begin(), viaMap.end(), start);
            if (position != viaMap.end())
                viaMap.erase(position);
            int via_cap = (start < 5) ? 1 : 3;
            cur_map_3d[p.x][p.y][start].cap += via_cap;
            start++;
        }
    }*/
    return;
}

void Coala::CoalaReroute::addIntoReroutePoints(const Jm::Coordinate_3d &p, int netIdx)

{

    bool addp = true;

    if (p.z != 0)

        if (cur_map_3d[p.x][p.y][p.z].max_cap < 1)

            addp = false;

    if (p.z == 0)

        if (cur_map_3d[p.x][p.y][1].max_cap < 1)

            addp = false;

    if (addp)

    {

        for (auto &tp : reroutePoints[netIdx])

        {

            if (p.x == tp.x && p.y == tp.y)

            {

                return;
            }
        }

        reroutePoints[netIdx].emplace_back(Jm::Coordinate_3d(p.x, p.y, std::max(p.z, 1)));
    }

    else
    {
        const PinptrList *pin_list = &rr_map->get_nPin(netIdx);
        for (const auto &pin : *pin_list)
        {
            if (p.x == pin->get_tileX() && p.y == pin->get_tileY())
            {

                for (auto &tp : reroutePoints[netIdx])

                {

                    if (p.x == tp.x && p.y == tp.y)

                    {

                        return;
                    }
                }

                reroutePoints[netIdx].emplace_back(Jm::Coordinate_3d(p.x, p.y, std::max(pin->get_layerId(), 1)));
                return;
            }
        }

        int dist = 100000000;

        auto addPin = (*pin_list)[0];

        for (const auto &pin : *pin_list)

        {

            int tdist = std::abs(pin->get_tileX() - p.x) + std::abs(pin->get_tileY() - p.y) + 1000 * std::abs(pin->get_layerId() - p.z);

            if (tdist < dist)

            {

                dist = tdist;

                addPin = pin;
            }
        }

        for (auto &tp : reroutePoints[netIdx])

        {

            if (addPin->get_tileX() == tp.x && addPin->get_tileY() == tp.y)

            {

                return;
            }
        }
        reroutePoints[netIdx].emplace_back(Jm::Coordinate_3d(addPin->get_tileX(), addPin->get_tileY(), std::max(addPin->get_layerId(), 1)));
    }

    return;
}

void Coala::CoalaReroute::removeRedundantPS(std::vector<std::vector<std::shared_ptr<Jm::Segment_2d>>> &fp, std::vector<std::unordered_map<int, std::vector<bool>>> &viasMap, std::vector<std::pair<Jm::Coordinate_2d, std::pair<int, int>>> &pointToPin)

{

    auto &remainSeg = nextlayerSegSet;

    for (auto &s : remainSeg)

    {

        int p1z = 0, p2z = 0;

        auto p1Index = coalaDB.Coala2Dto1D(s->p1.x, s->p1.y);

        auto p2Index = coalaDB.Coala2Dto1D(s->p2.x, s->p2.y);
        if (s->completeness == Jm::Fragmented)

        {
            int sLayer = s->layer;
            int broLayer = s->brotherSeg->layer;
            deleteSegFromFinalPath(s->brotherSeg, fp);
            deleteSegFromFinalPath(s, fp);
            capacityRecovery(s->brotherSeg);
            capacityRecovery(s);
            if (s->p1 == s->brotherSeg->p2)

            {
                auto brop1Index = coalaDB.Coala2Dto1D(s->brotherSeg->p1.x, s->brotherSeg->p1.y);
                coalaDB.pointPlaced[s->netIdx][coalaDB.Coala2Dto1D(s->brotherSeg->p2.x, s->brotherSeg->p2.y)]--;
                viaRecovery(s->p1, s->netIdx, viasMap, broLayer, true);
                viaRecovery(s->brotherSeg->p1, s->netIdx, viasMap, broLayer, false);
            }

            else
            {
                auto brop2Index = coalaDB.Coala2Dto1D(s->brotherSeg->p2.x, s->brotherSeg->p2.y);
                coalaDB.pointPlaced[s->netIdx][coalaDB.Coala2Dto1D(s->brotherSeg->p1.x, s->brotherSeg->p1.y)]--;
                viaRecovery(s->p2, s->netIdx, viasMap, broLayer, true);
                viaRecovery(s->brotherSeg->p2, s->netIdx, viasMap, broLayer, false);
            }
            s->addSeg(*(s->brotherSeg));
        }

        p1Index = coalaDB.Coala2Dto1D(s->p1.x, s->p1.y);

        p2Index = coalaDB.Coala2Dto1D(s->p2.x, s->p2.y);
        if (!viasMap[s->netIdx][p1Index].empty())
        {
            for (int j = rr_map->get_layerNumber() - 1; j > 0; j--)
            {
                if (viasMap[s->netIdx][p1Index][j])
                {
                    p1z = j;
                    break;
                }
            }
        }
        if (!viasMap[s->netIdx][p2Index].empty())
        {
            for (int j = rr_map->get_layerNumber() - 1; j > 0; j--)
            {
                if (viasMap[s->netIdx][p2Index][j])
                {
                    p2z = j;
                    break;
                }
            }
        }
        addIntoReroutePoints(Jm::Coordinate_3d(s->p1.x, s->p1.y, p1z), s->netIdx);

        addIntoReroutePoints(Jm::Coordinate_3d(s->p2.x, s->p2.y, p2z), s->netIdx);
    }

    for (auto &point : pointToPin)

    {

        addIntoReroutePoints(Jm::Coordinate_3d(point.first.x, point.first.y, point.first.pinLayer), point.second.second);

        addIntoReroutePoints(Jm::Coordinate_3d(point.first.x, point.first.y, point.first.pinLayer - point.second.first), point.second.second);
    }
}

int minKeyExtract(std::vector<int> &key, std::vector<bool> &visited, int netSize)

{

    int min = std::numeric_limits<int>::max(), min_idx = 0;

    for (int i = 0; i < netSize; i++)

    {

        if (visited[i] == false && key[i] < min)

        {

            min = key[i];

            min_idx = i;
        }
    }

    return min_idx;
}

std::vector<std::vector<int>> Coala::CoalaReroute::initAdjancentMatrix(int netIdx, int netSize)

{

    auto &reroutePoint = reroutePoints[netIdx];

    std::vector<std::vector<int>> adjancentMap;

    adjancentMap.resize(netSize, std::vector<int>(netSize, 0));

    for (int i = 0; i < netSize; ++i)

    {

        for (int j = 0; j < netSize; ++j)

        {

            adjancentMap[i][j] = reroutePoint[i].l1Distance(reroutePoint[j]);
        }
    }

    return adjancentMap;
}

int Coala::CoalaReroute::calculate3DCon(const Jm::Coordinate_3d &p1, const Jm::Coordinate_3d &p2)

{

    int maxX = std::max(p1.x, p2.x), minX = std::min(p1.x, p2.x);

    int maxY = std::max(p1.y, p2.y), minY = std::min(p1.y, p2.y);

    int maxZ = std::max(p1.z, p2.z), minZ = std::min(p1.z, p2.z);

    int remainCapaicity = 0;

    for (int x = minX; x <= maxX; x++)

    {

        for (int y = minY; y <= maxY; y++)

        {

            for (int z = minZ; z <= maxZ; z++)

            {

                remainCapaicity += cur_map_3d[x][y][z].cap;
            }
        }
    }

    return remainCapaicity;
}

void Coala::CoalaReroute::pinDecomposition(int netIdx)

{

    int netSize = reroutePoints[netIdx].size();

    if (!netSize)

        return;

    netSize = reroutePoints[netIdx].size();

    std::vector<int> predecessor(netSize, -1);

    std::vector<int> key(netSize, std::numeric_limits<int>::max());

    std::vector<bool> visited(netSize, false);

    auto adjancentMatrix = initAdjancentMatrix(netIdx, netSize);

    key[0] = 0;

    for (int c = 0; c < netSize - 1; ++c)

    {

        int u = minKeyExtract(key, visited, netSize);

        visited[u] = true;

        for (int v = 0; v < netSize; ++v)

        {

            if (visited[v] == false && adjancentMatrix[u][v] < key[v] && adjancentMatrix[u][v] != 0)

            {

                predecessor[v] = u;

                key[v] = adjancentMatrix[u][v];
            }
        }
    }

    for (int i = 1; i < netSize; ++i)

    {

        int rc = calculate3DCon(reroutePoints[netIdx][i], reroutePoints[netIdx][predecessor[i]]);

        twoPinNets.push(TwoPinNet(reroutePoints[netIdx][i], reroutePoints[netIdx][predecessor[i]], rc, netIdx));
    }
}

void Coala::CoalaReroute::setCoarseningGraph()

{

    for (int i = 0; i < rr_map->get_gridx(); i++)

    {

        for (int j = 0; j < rr_map->get_gridy(); j++)

        {

            for (int k = 0; k < rr_map->get_layerNumber(); k++)

            {

                int cx = i / 5;

                int cy = j / 5;

                coarseningGraph[cx][cy][k].cap += cur_map_3d[i][j][k].cap;

                coarseningGraph[cx][cy][k].max_cap += cur_map_3d[i][j][k].max_cap;
            }
        }
    }

    for (int i = 0; i < coarseningGraph.size(); i++)

    {

        for (int j = 0; j < coarseningGraph[0].size(); j++)

        {

            for (int k = 0; k < rr_map->get_layerNumber(); k++)

            {

                coarseningGraph[i][j][k].cap /= 25;

                coarseningGraph[i][j][k].max_cap /= 25;
            }
        }
    }
}

void Coala::CoalaReroute::aStarCoarsening(std::vector<std::vector<std::shared_ptr<Jm::Segment_2d>>> &fp)

{

    std::vector<bool> closedList(coarseningGraph.size() * coarseningGraph[0].size() * rr_map->get_layerNumber(), false);

    std::vector<int> closedListHistory;

    for (int n = 0; n < rr_map->get_netNumber(); ++n)

    {

        pinDecomposition(n);
    }

    auto twopinNet = twoPinNets;

    while (!twopinNet.empty())

    {

        auto routingNet = twopinNet.top();

        twopinNet.pop();

        std::unordered_map<int, double> bestCost;

        std::priority_queue<std::shared_ptr<AstarNode>, std::vector<std::shared_ptr<AstarNode>>, CompareAstarNode> openList;

        auto p1 = routingNet.p1 / 5;

        auto p2 = routingNet.p2 / 5;

        auto first = std::make_shared<AstarNode>(p1, 0);

        openList.push(first);

        int firstIndex = coalaDB.Coala3Dto1DC(first->p.x, first->p.y, first->p.z, 5);

        bestCost[firstIndex] = 10000000;

        const int dir3D[6][3] = {{0, 1, 0}, {0, -1, 0}, {-1, 0, 0}, {1, 0, 0}, {0, 0, 1}, {0, 0, -1}};

        bool routeSuccess = false;

        std::unordered_map<int, bool> routeGuide;

        int searchpoint = 0;

        while (!openList.empty())

        {

            auto cur = openList.top();

            openList.pop();

            searchpoint++;

            int curIndex = coalaDB.Coala3Dto1DC(cur->p.x, cur->p.y, cur->p.z, 5);

            if (bestCost.find(curIndex) != bestCost.end())

                if (cur->cost > bestCost[curIndex])

                    continue;

            closedList[curIndex] = true;

            closedListHistory.emplace_back(curIndex);

            if (cur->p == p2)

            {

                routeSuccess = true;

                routeGuide[coalaDB.Coala3Dto1DC(p1.x, p1.y, p1.z, 5)] = true;

                coarseningGraph[p1.x][p1.y][p1.z].cap -= 2;

                while (cur->parent)

                {

                    // if (searchpoint > 10000){

                    //  cur->p.printpoint();

                    //}

                    int rindex = coalaDB.Coala3Dto1DC(cur->p.x, cur->p.y, cur->p.z, 5);

                    routeGuide[rindex] = true;

                    cur = cur->parent;

                    // if (coarseningGraph[cur->p.x][cur->p.y][cur->p.z].cap -2 >= 0)

                    // coarseningGraph[cur->p.x][cur->p.y][cur->p.z].cap -= 2;
                }

                coarseningGuide.emplace_back(routeGuide);

                break;
            }

            for (int d = 0; d < 6; d++)

            {

                if (cur->p.z == 0 && d < 4)

                {

                    continue;
                }

                double tileCap = 0.0, tileMaxCap = 0.0, cost = 0.0;

                if (d < 2 && database.getLayerDir(cur->p.z) == Y)

                {

                    continue;
                }

                else if (d >= 2 && d < 4 && database.getLayerDir(cur->p.z) == X)

                {

                    continue;
                }

                Jm::Coordinate_3d neighborCoord = Jm::Coordinate_3d(cur->p.x + dir3D[d][0], cur->p.y + dir3D[d][1], cur->p.z + dir3D[d][2]);

                if (!coalaDB.coordinate3DValid(neighborCoord, 5))

                    continue;

                int neighborIndex = coalaDB.Coala3Dto1DC(neighborCoord.x, neighborCoord.y, neighborCoord.z, 5);

                if (closedList[neighborIndex])

                {

                    continue;
                }

                tileCap = coarseningGraph[neighborCoord.x][neighborCoord.y][neighborCoord.z].cap;

                tileMaxCap = coarseningGraph[neighborCoord.x][neighborCoord.y][neighborCoord.z].max_cap;

                tileMaxCap = std::max(tileMaxCap, 0.1);

                double logistic = 1 / (1 + std::exp(2 * tileCap));

                cost += (cur->wirelength + 1) + (cur->wirelength + 1) * (double)((tileMaxCap - tileCap) / tileMaxCap) * logistic * 2;

                auto neighbor = std::make_shared<AstarNode>(neighborCoord, cost);

                neighbor->parent = cur;

                neighbor->wirelength = cur->wirelength + 1;

                bool skip = false;

                if (bestCost.find(neighborIndex) != bestCost.end())

                    if (neighbor->cost > bestCost[neighborIndex])

                    {

                        skip = true;
                    }

                if (!skip)

                {

                    neighbor->h_cost = neighborCoord.l1Distance(p2);

                    neighbor->dir = d;

                    openList.push(neighbor);

                    bestCost[neighborIndex] = cost;
                }
            }
        }

        if (!routeSuccess)

        {
            coarseningGuide.emplace_back(routeGuide);
        }

        while (!closedListHistory.empty())

        {

            int cindex = closedListHistory.back();

            closedListHistory.pop_back();

            closedList[cindex] = false;
        }
    }
}

bool isPin(const Jm::Coordinate_3d &p, int netIdx)

{

    const PinptrList *pin_list = &rr_map->get_nPin(netIdx);

    for (const auto &pin : *pin_list)

    {

        if (p.x == pin->get_tileX() && p.y == pin->get_tileY())

        {

            return true;
        }
    }

    return false;
}

void Coala::CoalaReroute::aStar3D(std::vector<std::vector<std::shared_ptr<Jm::Segment_2d>>> &fp, std::vector<std::unordered_map<int, std::vector<bool>>> &viasMap)

{
    std::cout<<" a* sd ! \n";
    for (int n = 0; n < rr_map->get_netNumber(); ++n)

    {

        pinDecomposition(n);
    }
    
    std::cout<<"stop decomposition !\n";
    // setCoarseningGraph();

    // aStarCoarsening(fp);

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    std::vector<int> closeList(rr_map->get_gridx() * rr_map->get_gridy() * rr_map->get_layerNumber(), 1000000000);

    std::vector<bool> closedList(rr_map->get_gridx() * rr_map->get_gridy() * rr_map->get_layerNumber(), false);

    std::vector<int> closeListHistory;

    std::vector<int> closedListHistory;

    int netsCount = 0;

    while (!twoPinNets.empty())

    {

        int pointCount = 0;

        auto routingNet = twoPinNets.top();

        routingNet.routeIter++;

        twoPinNets.pop();

        //auto &routeGuide = coarseningGuide[netsCount];

        netsCount++;

        // openListStatus openListStat;

        std::unordered_map<int, double> bestCost;

        std::priority_queue<std::shared_ptr<AstarNode>, std::vector<std::shared_ptr<AstarNode>>, CompareAstarNode> openList;

        auto first = std::make_shared<AstarNode>(routingNet.p1, 0);

        int firstIndex = coalaDB.Coala3Dto1D(first->p.x, first->p.y, first->p.z);

        // openListStat[firstIndex].emplace_back(first);

        bestCost[firstIndex] = 10000000;

        openList.push(first);

        const int dir3D[6][3] = {{0, 1, 0}, {0, -1, 0}, {-1, 0, 0}, {1, 0, 0}, {0, 0, 1}, {0, 0, -1}};

        int iterCount = 0;

        bool routeSuccess = false;
        std::cout << rr_map->get_net_name(routingNet.netIdx) << "   was rerouted   \n";

        std::cout << "From ";

        routingNet.p1.printpoint();

        std::cout << "To ";

        routingNet.p2.printpoint();

        std::cout << "threre are " << twoPinNets.size() << " nets need to be route ! \n";
        
        bool p1isPin = isPin(routingNet.p1, routingNet.netIdx);

        bool p2isPin = isPin(routingNet.p2, routingNet.netIdx);

        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

        routingNet.p1.z = std::max(routingNet.p1.z, 1);

        routingNet.p2.z = std::max(routingNet.p2.z, 1);

        while (!openList.empty())

        {

            pointCount++;

            auto cur = openList.top();

            openList.pop();

            int curIndex = coalaDB.Coala3Dto1D(cur->p.x, cur->p.y, cur->p.z);

            closedList[curIndex] = true;

            closedListHistory.emplace_back(curIndex);

            if (cur->p == routingNet.p2 /*|| (cur->p.l1Distance(routingNet.p2) <= 1 && p2isPin && abs(cur->p.z - routingNet.p2.z) < 2)*/)
            {

                int netId = routingNet.netIdx;

                auto firstSeg = std::make_shared<Jm::Segment_2d>(netId, Jm::Coordinate_2d(cur->p.x, cur->p.y),

                                                                 Jm::Coordinate_2d(cur->p.x, cur->p.y), Jm::None);

                firstSeg->layer = cur->p.z;

                routeSuccess = true;

                auto trace = cur;

                while (trace->parent)

                {

                    if (trace->p.z == trace->parent->p.z)

                    {

                        auto tp = Jm::Coordinate_2d(trace->p.x, trace->p.y);

                        auto tpp = Jm::Coordinate_2d(trace->parent->p.x, trace->parent->p.y);

                        if (!firstSeg->addPoint(tp, tpp))

                        {

                            Jm::SegmentDir nd = (trace->p.y == trace->parent->p.y) ? Jm::Horizontal : Jm::Vertical;

                            firstSeg->sortPoint();

                            fp[netId].emplace_back(firstSeg);

                            firstSeg = std::make_shared<Jm::Segment_2d>(netId, tp, tpp, nd);

                            firstSeg->layer = trace->p.z;
                        }

                        cur_map_3d[trace->p.x][trace->p.y][trace->p.z].cap -= 2;

                        cur_map_3d[trace->parent->p.x][trace->parent->p.y][trace->parent->p.z].cap -= 2;
                    }

                    else

                    {

                        if (firstSeg->dir)

                        {

                            firstSeg->sortPoint();

                            fp[netId].emplace_back(firstSeg);
                        }

                        firstSeg = std::make_shared<Jm::Segment_2d>(netId, Jm::Coordinate_2d(trace->p.x, trace->p.y),

                                                                    Jm::Coordinate_2d(trace->p.x, trace->p.y), Jm::None);

                        firstSeg->layer = trace->parent->p.z;

                        int viaIndex = coalaDB.Coala2Dto1D(trace->p.x, trace->p.y);

                        int viaIndex3D = coalaDB.Coala3Dto1D(trace->p.x, trace->p.y, trace->p.z);

                        int pviaIndex3D = coalaDB.Coala3Dto1D(trace->parent->p.x, trace->parent->p.y, trace->parent->p.z);
                        coalaDB.viaUsage[viaIndex3D]++;
                        coalaDB.viaUsage[pviaIndex3D]++;
                        if (viasMap[netId][viaIndex].empty())
                            viasMap[netId][viaIndex] = std::vector<bool>(rr_map->get_layerNumber(), false);
                        viasMap[netId][viaIndex][trace->parent->p.z] = true;
                        viasMap[netId][viaIndex][trace->p.z] = true;
                        int via_cap = (trace->p.z < 5) ? 1 : 4;
                        cur_map_3d[trace->p.x][trace->p.y][trace->p.z].cap -= via_cap;
                        via_cap = (trace->parent->p.z < 5) ? 1 : 4;
                        cur_map_3d[trace->parent->p.x][trace->parent->p.y][trace->parent->p.z].cap -= via_cap;
                    }

                    trace = trace->parent;
                }

                fp[netId].emplace_back(firstSeg);

                break;
            }

            if (bestCost.find(curIndex) != bestCost.end())

                if (cur->cost > bestCost[curIndex])

                    continue;

            for (int d = 0; d < 6; d++)

            {

                if (cur->p.z == 0 && d < 4)

                {

                    continue;
                }

                double edgeDem, edgeMaxCapcity, cost = 0.0;

                if (d < 2 && database.getLayerDir(cur->p.z) == Y)

                {

                    continue;
                }

                else if (d >= 2 && d < 4 && database.getLayerDir(cur->p.z) == X)

                {

                    continue;
                }

                Jm::Coordinate_3d neighborCoord = Jm::Coordinate_3d(cur->p.x + dir3D[d][0], cur->p.y + dir3D[d][1], cur->p.z + dir3D[d][2]);

                if (!coalaDB.coordinate3DValid(neighborCoord, 1))

                    continue;

                int neighborIndex = coalaDB.Coala3Dto1D(neighborCoord.x, neighborCoord.y, neighborCoord.z);

                if (closedList[neighborIndex])

                {

                    continue;
                }

                if (d < 4)

                {

                    edgeDem = cur_map_3d[cur->p.x][cur->p.y][cur->p.z].edge_list[d]->cur_dem;

                    edgeMaxCapcity = cur_map_3d[cur->p.x][cur->p.y][cur->p.z].edge_list[d]->max_cap;
                }

                else

                {

                    edgeMaxCapcity = 10;

                    edgeDem = 8;
                }

                /*if (neighborCoord.x < coalaDB.netPinBound[routingNet.netIdx][0] - 50 || neighborCoord.x > coalaDB.netPinBound[routingNet.netIdx][1] + 50 ||

                    neighborCoord.y < coalaDB.netPinBound[routingNet.netIdx][2] - 50 || neighborCoord.y > coalaDB.netPinBound[routingNet.netIdx][3] + 50 ||

                    neighborCoord.z < coalaDB.netPinBound[routingNet.netIdx][4] - 50 || neighborCoord.z > coalaDB.netPinBound[routingNet.netIdx][5] + 50){

                        //std::cout<<"bound is "<<coalaDB.netPinBound[routingNet.netIdx][0]<<" "<<coalaDB.netPinBound[routingNet.netIdx][2]<<" "<<coalaDB.netPinBound[routingNet.netIdx][4]<<" ~ ";

                        //std::cout<<"bound is "<<coalaDB.netPinBound[routingNet.netIdx][1]<<" "<<coalaDB.netPinBound[routingNet.netIdx][3]<<" "<<coalaDB.netPinBound[routingNet.netIdx][5]<<" \n";

                    continue;

                    }*/

                /*int cheackCap = (routingNet.routeIter < 2) ? cur_map_3d[neighborCoord.x][neighborCoord.y][neighborCoord.z].cap : cur_map_3d[neighborCoord.x][neighborCoord.y][neighborCoord.z].max_cap;

                 */

                if (edgeMaxCapcity > 0 && (cur_map_3d[neighborCoord.x][neighborCoord.y][neighborCoord.z].max_cap > 0))

                {

                    // double logistic = 1 / (1 + std::exp(2 * (edgeMaxCapcity - edgeDem)));

                    double logistic2 = 1 / (1 + std::exp(cur_map_3d[neighborCoord.x][neighborCoord.y][neighborCoord.z].cap));

                    if (neighborCoord.z == 0)

                    {

                        cur_map_3d[neighborCoord.x][neighborCoord.y][neighborCoord.z].max_cap = 1;
                    }

                    // cost += ((cur->wirelength + 1) /*2*/ + (cur->wirelength + 1) * (double)(edgeDem / edgeMaxCapcity) * logistic);

                    double demand = std::abs(cur_map_3d[neighborCoord.x][neighborCoord.y][neighborCoord.z].cap - cur_map_3d[neighborCoord.x][neighborCoord.y][neighborCoord.z].max_cap);

                    cost += (cur->wirelength + 1) + (demand / cur_map_3d[neighborCoord.x][neighborCoord.y][neighborCoord.z].max_cap) * logistic2 * 50  + neighborCoord.l1Distance(routingNet.p2);
                    // 50 can be replaced by wirelength !;
                    // cost += (cur->wirelength + 1) + (edgeDem / edgeMaxCapcity) * logistic * (cur->wirelength + 1) * 2 + neighborCoord.l1Distance(routingNet.p2);

                    auto neighbor = std::make_shared<AstarNode>(neighborCoord, cost);

                    neighbor->parent = cur;

                    neighbor->wirelength = cur->wirelength + 1;

                    bool skip = false;

                    if (neighborCoord == routingNet.p2)

                    {

                        // std::cout << "find target at " << pointCount << " !\n";

                        neighbor->cost = -100.0;

                        cost = -100.0;
                    }

                    if (bestCost.find(neighborIndex) != bestCost.end())

                        if (neighbor->cost >= bestCost[neighborIndex])

                        {

                            skip = true;
                        }

                    if (!skip)

                    {

                        neighbor->dir = d;

                        neighbor->h_cost = neighborCoord.l1Distance(routingNet.p2);

                        openList.push(neighbor);

                        bestCost[neighborIndex] = cost;
                    }
                }
            }
        }

        while (!closedListHistory.empty())

        {

            int cindex = closedListHistory.back();

            closedListHistory.pop_back();

            closedList[cindex] = false;
        }

        // std::cout << " total point = " << pointCount << " !\n";

        if (!routeSuccess)

        {

            // std::cout << "not success ! \n";

            /*if (routingNet.routeIter < 10 && routingNet.p1.l1Distance(routingNet.p2) > 2)

            {

                //std::cout << "expand ! \n";

                twoPinNets.push(routingNet);

                continue;

            }*/

            if (routingNet.p1.x == routingNet.p2.x || routingNet.p1.y == routingNet.p2.y)

            {

                if (routingNet.p1.x == routingNet.p2.x)

                {

                    if (routingNet.p1.y > routingNet.p2.y)

                        std::swap(routingNet.p1, routingNet.p2);

                    int midPoint = (routingNet.p1.y + routingNet.p2.y) / 2;

                    int verticalLayer = database.getLayerDir(1) == X ? 2 : 3;

                    // experimental

                    int yrang = routingNet.p2.y - routingNet.p1.y;

                    bool shiftFlag = false;

                    for (int r = 1; r < 6; r++)

                    {

                        if (cur_map_3d[routingNet.p1.x][routingNet.p1.y + (yrang / 5) * r][verticalLayer].max_cap <= 0)

                            shiftFlag = true;
                    }

                    if (shiftFlag)

                    {

                        /*//std::cout << "\ntry to move the edge  ! \n";

                        //std::cout << "from : ";

                        routingNet.p1.printpoint();

                        //std::cout << "to : ";

                        routingNet.p2.printpoint();

                        //std::cout << rr_map->get_net_name(routingNet.netIdx) << " actuall need reroute ! \n";

                        //std::cout << "mid is " << midPoint << " v layer is " << verticalLayer << "\n";*/

                        int shiftR = 0, shiftL = 0;

                        int og_x1, og_x2;

                        bool needShift = true;

                        if (routingNet.p1.l1Distance(routingNet.p2) > 2)

                        {

                            // //std::cout << " need shift ! ! ! \n";

                            while (routingNet.p1.x + shiftR < rr_map->get_gridx())

                            {

                                shiftR++;

                                bool breakflag = true;

                                for (int r = 1; r < 6; r++)

                                {

                                    if (cur_map_3d[routingNet.p1.x + shiftR][routingNet.p1.y + (yrang / 5) * r][verticalLayer].max_cap <= 0)

                                        breakflag = false;
                                }

                                if (breakflag)

                                {

                                    break;
                                }
                            }

                            while (routingNet.p1.x - shiftL >= 0)

                            {

                                shiftL++;

                                bool breakflag = true;

                                for (int r = 1; r < 6; r++)

                                {

                                    if (cur_map_3d[routingNet.p1.x - shiftL][routingNet.p1.y + (yrang / 5) * r][verticalLayer].max_cap <= 0)

                                        breakflag = false;
                                }

                                if (breakflag)

                                {

                                    break;
                                }
                            }

                            // //std::cout << "shift left = " << shiftL << " shift right = " << shiftR << " \n";

                            og_x1 = routingNet.p1.x;

                            og_x2 = routingNet.p2.x;

                            if (shiftR >= shiftL)

                            {

                                routingNet.p1.x = routingNet.p1.x - shiftL;

                                routingNet.p2.x = routingNet.p2.x - shiftL;
                            }

                            else

                            {

                                routingNet.p1.x = routingNet.p1.x + shiftR;

                                routingNet.p2.x = routingNet.p2.x + shiftR;
                            }

                            int hlayer = database.getLayerDir(1) == Y ? 1 : 0;

                            for (int hl = hlayer; hl < rr_map->get_layerNumber() - 1; hl += 2)

                            {

                                auto addSeg = std::make_shared<Jm::Segment_2d>(routingNet.netIdx, Jm::Coordinate_2d(og_x1, routingNet.p1.y),

                                                                               Jm::Coordinate_2d(routingNet.p1.x, routingNet.p1.y), Jm::Horizontal);

                                auto addSeg2 = std::make_shared<Jm::Segment_2d>(routingNet.netIdx, Jm::Coordinate_2d(og_x2, routingNet.p2.y),

                                                                                Jm::Coordinate_2d(routingNet.p2.x, routingNet.p2.y), Jm::Horizontal);

                                addSeg->sortPoint();

                                addSeg2->sortPoint();

                                addSeg->layer = hl;

                                addSeg2->layer = hl;

                                fp[routingNet.netIdx].emplace_back(addSeg);

                                fp[routingNet.netIdx].emplace_back(addSeg2);
                            }
                        }
                    }

                    // this should not be used in normal situation
                }

                else if (routingNet.p1.y == routingNet.p2.y)

                {

                    if (routingNet.p1.x > routingNet.p2.x)

                        std::swap(routingNet.p1, routingNet.p2);

                    int midPoint = (routingNet.p1.x + routingNet.p2.x) / 2;

                    int hLayer = database.getLayerDir(1) == Y ? 2 : 3;

                    // experimental

                    int xrang = routingNet.p2.x - routingNet.p1.x;

                    bool shiftFlag = false;

                    for (int r = 1; r < 6; r++)

                    {

                        if (cur_map_3d[routingNet.p1.x + (xrang / 5) * r][routingNet.p1.y][hLayer].max_cap <= 0)

                            shiftFlag = true;
                    }

                    if (shiftFlag)

                    {

                        int shiftU = 0, shiftB = 0;

                        int og_y1, og_y2;

                        bool needShift = true;

                        if (routingNet.p1.l1Distance(routingNet.p2) > 2)

                        {

                            while (routingNet.p1.y + shiftU < rr_map->get_gridy())

                            {

                                shiftU++;

                                bool breakflag = true;

                                for (int r = 1; r < 6; r++)

                                {

                                    if (cur_map_3d[routingNet.p1.x + (xrang / 5) * r][routingNet.p1.y + shiftU][hLayer].max_cap <= 0)

                                        breakflag = false;
                                }

                                if (cur_map_3d[midPoint][routingNet.p1.y + shiftU][hLayer].max_cap > 0 && cur_map_3d[routingNet.p1.x][routingNet.p1.y + shiftU][hLayer].max_cap > 0)

                                {

                                    break;
                                }
                            }

                            while (routingNet.p1.y - shiftB >= 0)

                            {

                                shiftB++;

                                bool breakflag = true;

                                for (int r = 1; r < 6; r++)

                                {

                                    if (cur_map_3d[routingNet.p1.x + (xrang / 5) * r][routingNet.p1.y - shiftB][hLayer].max_cap <= 0)

                                        breakflag = false;
                                }

                                if (cur_map_3d[midPoint][routingNet.p1.y - shiftB][hLayer].max_cap > 0 && cur_map_3d[routingNet.p1.x][routingNet.p1.y - shiftB][hLayer].max_cap > 0)

                                {

                                    break;
                                }
                            }

                            // //std::cout << "shift UP = " << shiftU << " shift DOWN = " << shiftB << " \n";

                            og_y1 = routingNet.p1.y;

                            og_y2 = routingNet.p2.y;

                            if (shiftU >= shiftB)

                            {

                                routingNet.p1.y = routingNet.p1.y - shiftB;

                                routingNet.p2.y = routingNet.p2.y - shiftB;
                            }

                            else

                            {

                                routingNet.p1.y = routingNet.p1.y + shiftU;

                                routingNet.p2.y = routingNet.p2.y + shiftU;
                            }

                            int vlayer = database.getLayerDir(1) == X ? 1 : 0;

                            for (int vl = vlayer; vl < rr_map->get_layerNumber() - 1; vl += 2)

                            {

                                auto addSeg = std::make_shared<Jm::Segment_2d>(routingNet.netIdx, Jm::Coordinate_2d(routingNet.p1.x, og_y1),

                                                                               Jm::Coordinate_2d(routingNet.p1.x, routingNet.p1.y), Jm::Vertical);

                                auto addSeg2 = std::make_shared<Jm::Segment_2d>(routingNet.netIdx, Jm::Coordinate_2d(routingNet.p2.x, og_y2),

                                                                                Jm::Coordinate_2d(routingNet.p2.x, routingNet.p2.y), Jm::Vertical);

                                addSeg->sortPoint();

                                addSeg2->sortPoint();

                                addSeg->layer = vl;

                                addSeg2->layer = vl;

                                fp[routingNet.netIdx].emplace_back(addSeg);

                                fp[routingNet.netIdx].emplace_back(addSeg2);
                            }
                        }
                    }

                    // this should not be used in normal situation
                }

                for (int i = 0; i < rr_map->get_layerNumber() - 1; i++)

                {

                    if (i == 1)

                        continue;

                    Jm::SegmentDir sd;

                    if (routingNet.p1.x == routingNet.p2.x)

                    {

                        sd = Jm::Vertical;
                    }

                    else

                    {

                        sd = Jm::Horizontal;
                    }

                    auto failedSeg = std::make_shared<Jm::Segment_2d>(routingNet.netIdx, Jm::Coordinate_2d(routingNet.p1.x, routingNet.p1.y),

                                                                      Jm::Coordinate_2d(routingNet.p2.x, routingNet.p2.y), sd);

                    failedSeg->layer = i;

                    failedSeg->sortPoint();

                    if ((database.getLayerDir(i) == X && failedSeg->dir == Jm::Vertical) || (database.getLayerDir(i) == Y && failedSeg->dir == Jm::Horizontal))

                        fp[routingNet.netIdx].emplace_back(failedSeg);
                }
            }

            else

            {

                for (int i = 0; i < rr_map->get_layerNumber(); i++)

                {

                    if (i == 1)

                        continue;

                    auto failedSeg1 = std::make_shared<Jm::Segment_2d>(routingNet.netIdx, Jm::Coordinate_2d(routingNet.p1.x, routingNet.p1.y),

                                                                       Jm::Coordinate_2d(routingNet.p2.x, routingNet.p1.y), Jm::Horizontal);

                    auto failedSeg2 = std::make_shared<Jm::Segment_2d>(routingNet.netIdx, Jm::Coordinate_2d(routingNet.p2.x, routingNet.p1.y),

                                                                       Jm::Coordinate_2d(routingNet.p2.x, routingNet.p2.y), Jm::Vertical);

                    failedSeg1->sortPoint();

                    failedSeg2->sortPoint();

                    if ((database.getLayerDir(i) == X && failedSeg1->dir == Jm::Vertical) || (database.getLayerDir(i) == Y && failedSeg1->dir == Jm::Horizontal))

                    {

                        fp[routingNet.netIdx].emplace_back(failedSeg1);

                        failedSeg1->layer = i;
                    }

                    if ((database.getLayerDir(i) == X && failedSeg2->dir == Jm::Vertical) || (database.getLayerDir(i) == Y && failedSeg2->dir == Jm::Horizontal))

                    {

                        failedSeg2->layer = i;

                        fp[routingNet.netIdx].emplace_back(failedSeg2);
                    }
                }
            }
        }

        else

        {

            /*std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();

            //std::cout << "routing net took "

                      << std::chrono::duration_cast<std::chrono::seconds>(t4 - t3).count()

                      << "us.\n";*/
        }
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    std::cout << "routing took "

              << std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count()

              << "us.\n";
}
