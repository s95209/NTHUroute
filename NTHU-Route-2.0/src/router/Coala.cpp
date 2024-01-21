#include "Coala.h"

#include "CoalaReroute.h"

#include <iostream>

#include <fstream>

#include <chrono>

#include <sstream>

#include <string>

#include <boost/lexical_cast.hpp>

void Coala::Layerassignment::pinInitialize(int layer)

{

    auto pinList = coalaDB.layerPinList[layer];

    // add all the pin on the current layer into endPointSet

    for (auto &pin : pinList)

    {
        int via_cap = (pin.second->get_layerId() < 5) ? 1 : 4;
        coalaDB.addIntoPointPlaced(Jm::Coordinate_2d(pin.second->get_tileX(), pin.second->get_tileY()), pin.first);

        int index = coalaDB.Coala2Dto1D(pin.second->get_tileX(), pin.second->get_tileY());

        // get seg relate to added pin

        auto &relativeSeg = coalaDB.coalamap[index];

        // if (pin.second->get_layerId() == 0)

        //{
        int pl = (pin.second->get_layerId() == 0) ? 1 : pin.second->get_layerId();
        for (int x = -1; x < 2; x++)

        {

            if (pin.second->get_tileX() + x >= 0 && pin.second->get_tileX() + x < rr_map->get_gridx())

            {

                if (pin.second->get_tileY() + 1 < rr_map->get_gridy())

                {

                    cur_map_3d[pin.second->get_tileX() + x][pin.second->get_tileY() + 1][pl].cap -= via_cap;
                }

                if (pin.second->get_tileY() - 1 >= 0)

                {

                    cur_map_3d[pin.second->get_tileX() + x][pin.second->get_tileY() - 1][pl].cap -= via_cap;
                }

                cur_map_3d[pin.second->get_tileX() + x][pin.second->get_tileY()][pl].cap -= via_cap;
            }
        }

        cur_map_3d[pin.second->get_tileX()][pin.second->get_tileY()][pl].cap += via_cap;
        //}

        for (auto &seg : relativeSeg.segments)

        {

            if (seg->netIdx == pin.first && seg->cstat == Jm::New)

            {

                seg->cstat = Jm::Added;

                seg->completenessRate = coalaDB.netSegsNum[seg->netIdx] / coalaDB.netSegs[seg->netIdx].size();

                candidateSegSet.push(seg);
            }
        }
        if (!isInViaMap(index, pin.first))
            viasMap[pin.first][index] = std::vector<bool>(rr_map->get_layerNumber(), false);

        viasMap[pin.first][index][layer] = true;
    }
}

void Coala::Layerassignment::viaAssignment(int layer)

{
    for (auto &ep : endPointMap)

    {
        int viaCap = (layer < 5) ? 1 : 4;
        if (!ep.second.empty())

        {

            auto coord = coalaDB.Coala1Dto2D(ep.first);

            auto &nets = ep.second;

            for (auto &net : nets)

            {

                if (layer != 0)

                    if (cur_map_3d[coord[0]][coord[1]][layer + 1].cap < viaCap || (cur_map_3d[coord[0]][coord[1]][layer].cap < viaCap && layer > 4))
                    {

                        for (auto &s : coalaDB.coalamap[ep.first].segments)

                        {

                            if (s->cstat == Jm::Added && s->stat == Jm::Unplaced)

                            {

                                s->vstat = Jm::Invalid;
                            }
                        }

                        deleteFromEndPointMap(Jm::Coordinate_2d(coord[0], coord[1]), net);

                        continue;
                    }

                cur_map_3d[coord[0]][coord[1]][layer + 1].cap -= viaCap;

                if (layer != 0)
                    cur_map_3d[coord[0]][coord[1]][layer].cap -= viaCap;

                viasMap[net][ep.first][layer + 1] = true;

                coalaDB.viaUsage[coalaDB.Coala3Dto1D(coord[0], coord[1], layer)]++;

                coalaDB.viaUsage[coalaDB.Coala3Dto1D(coord[0], coord[1], layer + 1)]++;
            }
        }
    }
}

void Coala::Layerassignment::completeSegAssignment(int layer)

{

    auto layerDir = database.getLayerDir(layer);

    std::cout << "-----------------" << layer << "------------------\n";

    while (!candidateSegSet.empty())

    {

        bool assignable = true;

        auto cseg = candidateSegSet.top();

        cseg->seen = Jm::SegmentSeen::Yes;

        candidateSegSet.pop();

        if (cseg->vstat == Jm::Invalid)

        {

            invalidSegSet.emplace_back(cseg);

            continue;
        }

        // if candidate segment dir is horizontal

        if (cseg->dir == Jm::Horizontal && layerDir == Y)

        {

            int start = cseg->p1.x;

            int end = cseg->p2.x;

            // traverse every tile in increasing order

            if (start - end == 1)

            {

                if (cur_map_3d[start][cseg->p1.y][layer].cap < 1 || cur_map_3d[end][cseg->p1.y][layer].cap < 1)

                    assignable = false;
            }

            else

            {

                for (int i = start; i <= end; ++i)

                {

                    // check if the capcity is enough

                    if (i != start && i != end)

                    {

                        if (cur_map_3d[i][cseg->p1.y][layer].cap < 2)

                        {

                            assignable = false;

                            break;
                        }
                    }

                    else

                    {

                        if (cur_map_3d[i][cseg->p1.y][layer].cap < 1)

                        {

                            assignable = false;

                            break;
                        }
                    }
                }
            }

            // is the seg is assignable

            if (assignable)

            {

                // adjust tile cap and dem_pred map

                if (start - end == 1)

                {

                    cur_map_3d[start][cseg->p1.y][layer].cap--;

                    cur_map_3d[start][cseg->p1.y][layer].edge_list[RIGHT]->cur_dem++;

                    cur_map_3d[start][cseg->p1.y][layer].dem_pred--;

                    cur_map_3d[end][cseg->p1.y][layer].cap--;

                    cur_map_3d[end][cseg->p1.y][layer].edge_list[RIGHT]->cur_dem++;

                    cur_map_3d[end][cseg->p1.y][layer].dem_pred--;
                }

                else

                {

                    for (int i = start; i <= end; ++i)

                    {

                        if (i != start && i != end)

                        {

                            cur_map_3d[i][cseg->p1.y][layer].cap -= 2;

                            cur_map_3d[i][cseg->p1.y][layer].edge_list[RIGHT]->cur_dem++;

                            cur_map_3d[i][cseg->p1.y][layer].dem_pred -= 2;
                        }

                        else

                        {

                            if (i == start)

                                cur_map_3d[i][cseg->p1.y][layer].edge_list[RIGHT]->cur_dem++;

                            cur_map_3d[i][cseg->p1.y][layer].cap--;

                            cur_map_3d[i][cseg->p1.y][layer].dem_pred--;
                        }
                    }
                }
            }
        }

        else if (cseg->dir == Jm::Vertical && layerDir == X)

        {

            int start = cseg->p1.y;

            int end = cseg->p2.y;

            if (start - end == 1)

            {

                if (cur_map_3d[cseg->p1.x][start][layer].cap < 1 || cur_map_3d[cseg->p1.x][end][layer].cap < 1)

                    assignable = false;
            }

            else

            {

                for (int i = start; i <= end; ++i)

                {

                    if (i != start && i != end)

                    {

                        if (cur_map_3d[cseg->p1.x][i][layer].cap < 2)

                        {

                            assignable = false;

                            break;
                        }
                    }

                    else

                    {

                        if (cur_map_3d[cseg->p1.x][i][layer].cap < 1)

                        {

                            assignable = false;

                            break;
                        }
                    }
                }
            }

            if (assignable)

            {

                if (start - end == 1)

                {

                    cur_map_3d[cseg->p1.x][start][layer].edge_list[FRONT]->cur_dem++;

                    cur_map_3d[cseg->p1.x][start][layer].cap--;

                    cur_map_3d[cseg->p1.x][start][layer].dem_pred--;

                    cur_map_3d[cseg->p1.x][end][layer].edge_list[FRONT]->cur_dem++;

                    cur_map_3d[cseg->p1.x][end][layer].cap--;

                    cur_map_3d[cseg->p1.x][end][layer].dem_pred--;
                }

                else

                {

                    for (int i = start; i <= end; ++i)

                    {

                        if (i != start && i != end)

                        {

                            cur_map_3d[cseg->p1.x][i][layer].edge_list[FRONT]->cur_dem += 1;

                            cur_map_3d[cseg->p1.x][i][layer].cap -= 2;

                            cur_map_3d[cseg->p1.x][i][layer].dem_pred -= 2;
                        }

                        else

                        {

                            if (i == start)

                                cur_map_3d[cseg->p1.x][i][layer].edge_list[FRONT]->cur_dem += 1;

                            cur_map_3d[cseg->p1.x][i][layer].cap--;

                            cur_map_3d[cseg->p1.x][i][layer].dem_pred--;
                        }
                    }
                }
            }
        }

        else if ((cseg->dir == Jm::Vertical && layerDir == Y) || (cseg->dir == Jm::Horizontal && layerDir == X))

        {

            assignable = false;

            nextlayerSegSet.emplace_back(cseg);

            continue;
        }

        // if the segment is assignable, we need to find all the segs relative to p1 and p2

        if (assignable)

        {

            cseg->stat = Jm::Placed;

            coalaDB.netSegsNum[cseg->netIdx]--;

            cseg->layer = layer;

            finalPath[cseg->netIdx].emplace_back(cseg);

            // if p1 not found add it into endpoint set

            if (!isInEndPointMap(cseg->p1, cseg->netIdx))

            {

                addToEndPointMap(cseg->p1, cseg->netIdx, layer);
            }

            if (!isInEndPointMap(cseg->p2, cseg->netIdx))

            {

                addToEndPointMap(cseg->p2, cseg->netIdx, layer);
            }

            // add all relative seg

            addRelativeSeg(cseg->p1, cseg->netIdx);

            addRelativeSeg(cseg->p2, cseg->netIdx);

            auto p1Delete = deleteRedundantEP(cseg->p1, cseg->netIdx);

            auto p2Delete = deleteRedundantEP(cseg->p2, cseg->netIdx);

            if (p1Delete && p2Delete)

            {

                if (cseg->p1.pinLayer != -1 && cseg->layer < cseg->p1.pinLayer)

                {

                    pointToPin.emplace_back(std::make_pair(Jm::Coordinate_2d(cseg->p1), std::make_pair(cseg->p1.pinLayer - cseg->layer, cseg->netIdx)));
                }

                if (cseg->p2.pinLayer != -1 && cseg->layer < cseg->p2.pinLayer)

                {

                    pointToPin.emplace_back(std::make_pair(Jm::Coordinate_2d(cseg->p2), std::make_pair(cseg->p2.pinLayer - cseg->layer, cseg->netIdx)));
                }
            }

            coalaDB.addIntoPointPlaced(cseg->p1, cseg->netIdx);

            coalaDB.addIntoPointPlaced(cseg->p2, cseg->netIdx);
        }

        else

        {

            if (cseg->dir == Jm::Horizontal)

            {

                int start = cseg->p1.x;

                int end = cseg->p2.x;

                for (int i = start; i <= end; ++i)

                {

                    if (cur_map_3d[i][cseg->p1.y][layer].dem_pred > 0)

                        cseg->demandTileNum++;

                    if (cur_map_3d[i][cseg->p1.y][layer].cap > 0)

                        cseg->availableTileNum++;
                }
            }

            else

            {

                int start = cseg->p1.y;

                int end = cseg->p2.y;

                for (int i = start; i <= end; ++i)

                {

                    if (cur_map_3d[cseg->p1.x][i][layer].dem_pred > 0)

                        cseg->demandTileNum++;

                    if (cur_map_3d[cseg->p1.x][i][layer].cap > 0)

                        cseg->availableTileNum++;
                }
            }

            remainSegSet.push(cseg);
        }
    }

    remainSegAssignment(layer);

    std::cout << "remain seg assignment end ! \n";

    if (layer != rr_map->get_layerNumber() - 1)

    {

        std::cout << "unplaced seg num = " << nextlayerSegSet.size() << " !\n";

        for (auto &seg : nextlayerSegSet)

        {

            seg->completenessRate = coalaDB.netSegsNum[seg->netIdx] / coalaDB.netSegs[seg->netIdx].size();

            seg->cstat = Jm::Added;

            candidateSegSet.push(seg);
        }

        nextlayerSegSet.clear();
    }
}

void Coala::Layerassignment::addRelativeSeg(Jm::Coordinate_2d &p, int netIdx)

{

    auto relativeSeg = coalaDB.coalamap[coalaDB.Coala2Dto1D(p.x, p.y)].segments;

    for (auto &seg : relativeSeg)

    {

        if (seg->stat == Jm::Unplaced && seg->netIdx == netIdx)

        {

            if (seg->cstat == Jm::New)

            {

                seg->completenessRate = coalaDB.netSegsNum[seg->netIdx] / coalaDB.netSegs[seg->netIdx].size();

                seg->cstat = Jm::Added;

                candidateSegSet.push(seg);
            }
        }
    }
}

bool Coala::Layerassignment::deleteRedundantEP(Jm::Coordinate_2d &p, int netIdx)

{

    bool deletep = true;

    // get all relative seg

    auto relativeSeg = coalaDB.coalamap[coalaDB.Coala2Dto1D(p.x, p.y)].segments;

    for (auto &seg : relativeSeg)

    {

        if (seg->stat == Jm::Unplaced && seg->netIdx == netIdx)

        {

            deletep = false;
        }
    }

    if (deletep)

    {

        deleteFromEndPointMap(p, netIdx);

        return true;
    }

    return false;
}

void Coala::Layerassignment::addToEndPointMap(Jm::Coordinate_2d &p, int netIdx, int layer)

{

    int pIndex = coalaDB.Coala2Dto1D(p.x, p.y);
    endPointMap[pIndex].emplace_back(netIdx);
    if (!isInViaMap(pIndex, netIdx))
        viasMap[netIdx][pIndex] = std::vector<bool>(rr_map->get_layerNumber(), false);
    viasMap[netIdx][pIndex][layer] = true;
}

void Coala::Layerassignment::deleteFromEndPointMap(Jm::Coordinate_2d p, int netIdx)

{

    auto pIndex = coalaDB.Coala2Dto1D(p.x, p.y);

    auto &nets = endPointMap[pIndex];

    auto it = nets.begin();

    while (it != nets.end())

    {

        if ((*it) == netIdx)

            it = nets.erase(it);

        else

            it++;
    }

    return;
}

bool Coala::Layerassignment::isInEndPointMap(Jm::Coordinate_2d &p, int netIdx)

{

    bool pFound = false;

    auto pNets = endPointMap[coalaDB.Coala2Dto1D(p.x, p.y)];

    for (auto &s : pNets)

    {

        if (s == netIdx)

            pFound = true;
    }

    return pFound;
}

void Coala::Layerassignment::remainSegAssignment(int layer)

{

    while (!remainSegSet.empty())

    {

        auto rseg = remainSegSet.top();

        remainSegSet.pop();

        // check if p1 and p2 in the endpointset

        bool p1Found = isInEndPointMap(rseg->p1, rseg->netIdx);

        bool p2Found = isInEndPointMap(rseg->p2, rseg->netIdx);

        bool fragmentable = false;

        std::shared_ptr<Jm::Segment_2d> newseg;

        if (rseg->dir == Jm::Horizontal)

        {

            if (p1Found)

            {

                auto oldP2 = rseg->p2;

                if (cur_map_3d[rseg->p1.x][rseg->p1.y][layer].cap >= 1 && cur_map_3d[rseg->p1.x + 1][rseg->p1.y][layer].cap >= 2)

                {

                    fragmentable = true;

                    cur_map_3d[rseg->p1.x][rseg->p1.y][layer].cap--;

                    cur_map_3d[rseg->p1.x][rseg->p1.y][layer].dem_pred--;

                    cur_map_3d[rseg->p1.x][rseg->p1.y][layer].edge_list[RIGHT]->cur_dem += 1;

                    for (int i = rseg->p1.x + 1; i <= rseg->p2.x; ++i)

                    {

                        if (cur_map_3d[i][rseg->p1.y][layer].cap >= 2)

                        {

                            cur_map_3d[i][rseg->p1.y][layer].edge_list[RIGHT]->cur_dem += 1;

                            cur_map_3d[i][rseg->p1.y][layer].cap -= 2;

                            cur_map_3d[i][rseg->p1.y][layer].dem_pred -= 2;
                        }

                        else if (cur_map_3d[i][rseg->p1.y][layer].cap >= 1)

                        {

                            cur_map_3d[i][rseg->p1.y][layer].cap--;

                            cur_map_3d[i][rseg->p1.y][layer].dem_pred--;

                            rseg->p2.x = i;

                            break;
                        }

                        else

                        {

                            cur_map_3d[i - 1][rseg->p1.y][layer].cap++;

                            cur_map_3d[i - 1][rseg->p1.y][layer].dem_pred++;

                            cur_map_3d[i - 1][rseg->p1.y][layer].edge_list[RIGHT]->cur_dem -= 1;

                            rseg->p2.x = i - 1;

                            break;
                        }
                    }

                    coalaDB.addIntoPointPlaced(rseg->p1, rseg->netIdx);

                    coalaDB.addIntoPointPlaced(rseg->p2, rseg->netIdx);

                    newseg = std::make_shared<Jm::Segment_2d>(Jm::Segment_2d(rseg->netIdx, rseg->p2, oldP2, rseg->dir));
                }
            }

            else

            {

                auto oldP1 = rseg->p1;

                if (cur_map_3d[rseg->p2.x][rseg->p2.y][layer].cap >= 1 && cur_map_3d[rseg->p2.x - 1][rseg->p2.y][layer].cap >= 2)

                {

                    fragmentable = true;

                    cur_map_3d[rseg->p2.x][rseg->p2.y][layer].cap--;

                    cur_map_3d[rseg->p2.x][rseg->p2.y][layer].dem_pred--;

                    cur_map_3d[rseg->p2.x][rseg->p2.y][layer].edge_list[LEFT]->cur_dem += 1;

                    for (int i = rseg->p2.x - 1; i >= rseg->p1.x; --i)

                    {

                        if (cur_map_3d[i][rseg->p1.y][layer].cap >= 2)

                        {

                            cur_map_3d[i][rseg->p1.y][layer].edge_list[LEFT]->cur_dem += 1;

                            cur_map_3d[i][rseg->p1.y][layer].cap -= 2;

                            cur_map_3d[i][rseg->p1.y][layer].dem_pred -= 2;
                        }

                        else if (cur_map_3d[i][rseg->p1.y][layer].cap >= 1)

                        {

                            cur_map_3d[i][rseg->p1.y][layer].cap--;

                            cur_map_3d[i][rseg->p1.y][layer].dem_pred--;

                            rseg->p1.x = i;

                            break;
                        }

                        else

                        {

                            cur_map_3d[i + 1][rseg->p1.y][layer].edge_list[LEFT]->cur_dem -= 1;

                            cur_map_3d[i + 1][rseg->p1.y][layer].cap++;

                            cur_map_3d[i + 1][rseg->p1.y][layer].dem_pred++;

                            rseg->p1.x = i + 1;

                            break;
                        }
                    }

                    coalaDB.addIntoPointPlaced(rseg->p1, rseg->netIdx);

                    coalaDB.addIntoPointPlaced(rseg->p2, rseg->netIdx);

                    newseg = std::make_shared<Jm::Segment_2d>(Jm::Segment_2d(rseg->netIdx, oldP1, rseg->p1, rseg->dir));
                }
            }
        }

        else

        {

            if (p1Found)

            {

                auto oldP2 = rseg->p2;

                if (cur_map_3d[rseg->p1.x][rseg->p1.y][layer].cap >= 1 && cur_map_3d[rseg->p1.x][rseg->p1.y + 1][layer].cap >= 2)

                {

                    fragmentable = true;

                    cur_map_3d[rseg->p1.x][rseg->p1.y][layer].cap--;

                    cur_map_3d[rseg->p1.x][rseg->p1.y][layer].dem_pred--;

                    cur_map_3d[rseg->p1.x][rseg->p1.y][layer].edge_list[FRONT]->cur_dem += 1;

                    coalaDB.addIntoPointPlaced(rseg->p1, rseg->netIdx);

                    for (int i = rseg->p1.y + 1; i <= rseg->p2.y; ++i)

                    {

                        if (cur_map_3d[rseg->p1.x][i][layer].cap >= 2)

                        {

                            cur_map_3d[rseg->p1.x][i][layer].cap -= 2;

                            cur_map_3d[rseg->p1.x][i][layer].dem_pred -= 2;

                            cur_map_3d[rseg->p1.x][i][layer].edge_list[FRONT]->cur_dem += 1;
                        }

                        else if (cur_map_3d[rseg->p1.x][i][layer].cap >= 1)

                        {

                            cur_map_3d[rseg->p1.x][i][layer].cap--;

                            cur_map_3d[rseg->p1.x][i][layer].dem_pred--;

                            rseg->p2.y = i;

                            break;
                        }

                        else

                        {

                            cur_map_3d[rseg->p1.x][i - 1][layer].cap++;

                            cur_map_3d[rseg->p1.x][i - 1][layer].dem_pred++;

                            cur_map_3d[rseg->p1.x][i - 1][layer].edge_list[FRONT]->cur_dem -= 1;

                            rseg->p2.y = i - 1;

                            break;
                        }
                    }

                    coalaDB.addIntoPointPlaced(rseg->p2, rseg->netIdx);

                    newseg = std::make_shared<Jm::Segment_2d>(Jm::Segment_2d(rseg->netIdx, rseg->p2, oldP2, rseg->dir));
                }
            }

            else

            {

                auto oldP1 = rseg->p1;

                if (cur_map_3d[rseg->p2.x][rseg->p2.y][layer].cap >= 1 && cur_map_3d[rseg->p2.x][rseg->p2.y - 1][layer].cap >= 2)

                {

                    fragmentable = true;

                    cur_map_3d[rseg->p2.x][rseg->p2.y][layer].cap--;

                    cur_map_3d[rseg->p2.x][rseg->p2.y][layer].dem_pred--;

                    cur_map_3d[rseg->p2.x][rseg->p2.y][layer].edge_list[BACK]->cur_dem += 1;

                    coalaDB.addIntoPointPlaced(rseg->p2, rseg->netIdx);

                    for (int i = rseg->p2.y - 1; i >= rseg->p1.y; --i)

                    {

                        if (cur_map_3d[rseg->p1.x][i][layer].cap >= 2)

                        {

                            cur_map_3d[rseg->p1.x][i][layer].edge_list[BACK]->cur_dem += 1;

                            cur_map_3d[rseg->p1.x][i][layer].cap -= 2;

                            cur_map_3d[rseg->p1.x][i][layer].dem_pred -= 2;
                        }

                        else if (cur_map_3d[rseg->p1.x][i][layer].cap >= 1)

                        {

                            cur_map_3d[rseg->p1.x][i][layer].cap--;

                            cur_map_3d[rseg->p1.x][i][layer].dem_pred--;

                            rseg->p1.y = i;

                            break;
                        }

                        else

                        {

                            cur_map_3d[rseg->p1.x][i + 1][layer].edge_list[BACK]->cur_dem -= 1;

                            cur_map_3d[rseg->p1.x][i + 1][layer].cap++;

                            cur_map_3d[rseg->p1.x][i + 1][layer].dem_pred++;

                            rseg->p1.y = i + 1;

                            break;
                        }
                    }

                    coalaDB.addIntoPointPlaced(rseg->p1, rseg->netIdx);

                    newseg = std::make_shared<Jm::Segment_2d>(Jm::Segment_2d(rseg->netIdx, oldP1, rseg->p1, rseg->dir));
                }
            }
        }

        if (fragmentable)

        {

            newseg->brotherSeg = rseg;

            newseg->sortPoint();

            newseg->completeness = Jm::Fragmented;

            nextlayerSegSet.emplace_back(newseg);

            rseg->layer = layer;

            rseg->stat = Jm::Placed;

            if (p1Found)

            {

                addRelativeSeg(rseg->p1, rseg->netIdx);

                addToEndPointMap(rseg->p2, rseg->netIdx, layer);

                bool p1Delete = deleteRedundantEP(rseg->p1, rseg->netIdx);
            }

            else

            {

                addRelativeSeg(rseg->p2, rseg->netIdx);

                addToEndPointMap(rseg->p1, rseg->netIdx, layer);

                bool p2Delete = deleteRedundantEP(rseg->p2, rseg->netIdx);
            }

            finalPath[rseg->netIdx].emplace_back(rseg);

            coalaDB.setNewSeg(newseg);
        }

        else

        {

            rseg->rstat = Jm::YES;

            nextlayerSegSet.emplace_back(rseg);
        }
    }
}

bool Coala::Layerassignment::isInViaMap(int index, int netIdx)

{

    if (viasMap[netIdx].find(index) == viasMap[netIdx].end())

    {

        return false;
    }

    return true;
}

void Coala::Layerassignment::updateDemandPredMap(int layer)

{

    int xMax = rr_map->get_gridx();

    int yMax = rr_map->get_gridy();

    for (int i = 0; i < xMax; i++)

    {

        for (int j = 0; j < yMax; j++)

        {

            cur_map_3d[i][j][layer].dem_pred += cur_map_3d[i][j][layer - 1].dem_pred;
        }
    }

    return;
}

const std::vector<std::string> split(const std::string &str, const char &delimiter)

{

    std::vector<std::string> result;

    std::stringstream ss(str);

    std::string tok;

    while (std::getline(ss, tok, delimiter))

    {

        result.push_back(tok);
    }

    return result;
}

void Coala::Layerassignment::readTileCongestionMap(string inputFileName)

{

    std::string line;

    ifstream myfile(inputFileName);

    if (myfile.is_open())

    {

        int layer = 0;

        while (std::getline(myfile, line))

        {

            int y_index = 0;

            std::vector<std::string> ret = split(line, ' ');

            if (line != "=")

            {

                int x_index = 0;

                for (auto &s : ret)

                {

                    if (cur_map_3d[x_index][y_index][layer].cap > 0)

                        cur_map_3d[x_index][y_index][layer].cap -= (4 * (cur_map_3d[x_index][y_index][layer].cap - std::stoi(s)) / cur_map_3d[x_index][y_index][layer].cap);

                    x_index++;
                }
            }

            else

            {

                layer++;
            }

            y_index++;
        }

        myfile.close();
    }

    else

        cout << "Unable to open file";
}

void Coala::Layerassignment::generateOutput()

{

    ofstream outPutFile;

    outPutFile.open("test2.output");

    int netId = 0;

    int netCount = 0;

    std::vector<std::vector<std::vector<int>>> checkCap;

    for (auto &net : finalPath)

    {

        netCount++;

        std::string netName = rr_map->get_netName(netId);

        auto pins = rr_map->get_nPin(netId);

        outPutFile << netName << "\n(\n";

        std::unordered_map<int, std::vector<int>> viaRange;

        for (auto &pin : pins)

        {

            viaRange[coalaDB.Coala2Dto1D(pin->get_tileX(), pin->get_tileY())].emplace_back(pin->get_layerId() + 1);

            viaRange[coalaDB.Coala2Dto1D(pin->get_tileX(), pin->get_tileY())].emplace_back(pin->get_layerId() + 1);

            if (pin->get_layerId() == 0)

            {

                for (int i = 1; i <= rr_map->get_layerNumber(); i++) //Different w/ nthu-route-gary: i<=3
                {
                    int patch = 1;
                    int pinLX = std::max(pin->get_tileX() - patch, 0);
                    int pinRX = std::min(pin->get_tileX() + patch, rr_map->get_gridx() - 1);
                    int pinLY = std::max(pin->get_tileY() - 1, 0);
                    int pinRY = std::min(pin->get_tileY() + 1, rr_map->get_gridy() - 1);
                    outPutFile << pinLX << " " << pin->get_tileY() << " " << i << " " << pinRX << " " << pin->get_tileY() << " " << i << "\n";
                    outPutFile << pinLX << " " << pinLY << " " << i << " " << pinRX << " " << pinLY << " " << i << "\n";
                    outPutFile << pinLX << " " << pinRY << " " << i << " " << pinRX << " " << pinRY << " " << i << "\n";
                }
            }

            else if (pin->get_layerId() != 0)

            {
                for (int i = 1; i <= rr_map->get_layerNumber(); i++) //Different w/ nthu-route-gary: i=pin->get_layerId()+1; i<=rr_map->get_layerNumber()
                {
                    //if (pin->get_layerId() + i <= rr_map->get_layerNumber() && pin->get_layerId() + i > 0)
                   // {
                        for (int j = -2; j < 3; j++)
                        {
                            int pinLX = std::max(pin->get_tileX() - 2, 0);
                            int pinRX = std::min(pin->get_tileX() + 2, rr_map->get_gridx() - 1);
                            int pinY = std::max(pin->get_tileY() + j, 0);
                            pinY = std::min(pinY, rr_map->get_gridy() - 1);
                            outPutFile << pinLX << " " << pinY << " " << i << " " << pinRX << " " << pinY << " " << i << "\n";
                            /*if (database.getLayerDir(pin->get_layerId() + i - 1) == Y)
                            {

                                int pinY = std::max(pin->get_tileY() + j, 0);

                                pinY = std::min(pinY, rr_map->get_gridy() - 1);

                                outPutFile << std::max(pin->get_tileX() - 2, 0) << " " << pinY << " " << pin->get_layerId() + i << " " << std::min(pin->get_tileX() + 2, rr_map->get_gridx() - 1) << " " << pinY << " " << pin->get_layerId() + i << "\n";
                            }

                            else

                            {

                                int pinX = std::max(pin->get_tileX() + j, 0);

                                pinX = std::min(pinX, rr_map->get_gridx() - 1);

                                outPutFile << pinX << " " << std::max(pin->get_tileY() - 2, 0) << " " << pin->get_layerId() + i << " " << pinX << " " << std::min(pin->get_tileY() + 2, rr_map->get_gridy() - 1) << " " << pin->get_layerId() + i << "\n";
                            }*/
                        }
                   // }
                }
            }
        }

        for (auto &s : net)

        {
            outPutFile << s->p1.x << " " << s->p1.y << " " << s->layer + 1 << " " << s->p2.x << " " << s->p2.y << " " << s->layer + 1 << "\n";

            int p1Coord = coalaDB.Coala2Dto1D(s->p1.x, s->p1.y);

            int p2Coord = coalaDB.Coala2Dto1D(s->p2.x, s->p2.y);

            if (s->dir == Jm::Horizontal)

            {

                int start = s->p1.x;

                int end = s->p2.x;

                for (int i = start; i <= end; i++)

                {

                    if (cur_map_3d[i][s->p1.y][s->layer].cap < 0)

                    {

                        for (int j = 2; j > -1; j--)

                        {

                            for (int k = -1; k < 2; k++)

                            {

                                if (s->layer + j <= rr_map->get_layerNumber() && s->layer + j > 0)

                                {
                                    int lX = std::max(i - 1, 0);

                                    int rX = std::min(i + 1, rr_map->get_gridx() - 1);

                                    int nY = std::max(s->p1.y + k, 0);

                                    nY = std::min(nY, rr_map->get_gridy() - 1);

                                    outPutFile << lX << " " << nY << " " << s->layer + j << " " << rX << " " << nY << " " << s->layer + j << "\n";
                                }
                            }
                        }
                    }
                }
            }

            else

            {

                int start = s->p1.y;

                int end = s->p2.y;

                for (int i = start; i <= end; i++)

                {

                    if (cur_map_3d[s->p1.x][i][s->layer].cap < 0)

                    {

                        for (int j = 2; j > -1; j--)

                        {

                            for (int k = -1; k < 2; k++)

                            {

                                if (s->layer + j <= rr_map->get_layerNumber() && s->layer + j > 0)

                                {

                                    int lY = std::max(i - 1, 0);

                                    int rY = std::min(i + 1, rr_map->get_gridy() - 1);

                                    int nX = std::max(s->p1.x + k, 0);

                                    nX = std::min(nX, rr_map->get_gridx() - 1);

                                    outPutFile << nX << " " << lY << " " << s->layer + j << " " << nX << " " << rY << " " << s->layer + j << "\n";
                                }
                            }
                        }
                    }
                }
            }

            if (viaRange.find(p1Coord) == viaRange.end())

            {

                viaRange[p1Coord].emplace_back(s->layer + 1);

                viaRange[p1Coord].emplace_back(s->layer + 1);
            }

            else

            {

                if (s->layer + 1 < viaRange[p1Coord][0])

                {

                    viaRange[p1Coord][0] = s->layer + 1;
                }

                if (s->layer + 1 > viaRange[p1Coord][1])

                {

                    viaRange[p1Coord][1] = s->layer + 1;
                }
            }

            if (viaRange.find(p2Coord) == viaRange.end())

            {

                viaRange[p2Coord].emplace_back(s->layer + 1);

                viaRange[p2Coord].emplace_back(s->layer + 1);
            }

            else

            {

                if (s->layer + 1 < viaRange[p2Coord][0])

                {

                    viaRange[p2Coord][0] = s->layer + 1;
                }

                if (s->layer + 1 > viaRange[p2Coord][1])

                {

                    viaRange[p2Coord][1] = s->layer + 1;
                }
            }
        }

        for (auto &via : viaRange)

        {

            auto coord = coalaDB.Coala1Dto2D(via.first);

            if (via.second[0] != via.second[1])

                outPutFile << coord[0] << " " << coord[1] << " " << via.second[0] << " " << coord[0] << " " << coord[1] << " " << via.second[1] << "\n";

            else

                outPutFile << coord[0] << " " << coord[1] << " " << 1 << " " << coord[0] << " " << coord[1] << " " << rr_map->get_layerNumber() << "\n";
        }

        outPutFile << ")\n";

        netId++;
    }

    auto &deletedNet = rr_map->get_deleted_netList();

    for (auto &net : deletedNet)

    {

        netCount++;

        outPutFile << net.get_name() << "\n(\n";
        auto &pinList = net.get_pinList();
        int maxLayer = rr_map->get_layerNumber();
        for (auto &pin : pinList)

        {

            int x = pin->get_tileX();

            int y = pin->get_tileY();

            int layer = pin->get_layerId();

            int maxL;

            maxL = (layer + 3 <= maxLayer) ? layer + 3 : maxLayer;

            for (int l = layer + 1; l <= maxL; l++)

            {
                outPutFile << std::max(x - 1, 0) << " " << std::max(y - 1, 0) << " " << l << " " << std::min(x + 1, rr_map->get_gridx()) << " " << std::max(y - 1, 0) << " " << l << "\n";
                outPutFile << std::max(x - 1, 0) << " " << y << " " << l << " " << std::min(x + 1, rr_map->get_gridx()) << " " << y << " " << l << "\n";
                outPutFile << std::max(x - 1, 0) << " " << std::min(y + 1, rr_map->get_gridy()) << " " << l << " " << std::min(x + 1, rr_map->get_gridx()) << " " << std::min(y + 1, rr_map->get_gridy()) << " " << l << "\n";
            }
        }

        outPutFile << ")\n";
    }
    /*
       auto &deletedNet = rr_map->get_deleted_netList();
       for (auto &net : deletedNet)
       {
           outPutFile << net.get_name() << "\n(\n";
           auto &pinList = net.get_pinList();
           int maxLayer = rr_map->get_layerNumber();
           for (auto &pin : pinList)
           {
               int x = pin->get_tileX();
               int y = pin->get_tileY();
               int layer = pin->get_layerId();
               int maxL;
               // if (layer == 0)
               //  maxL = (layer + 4 <= maxLayer) ? layer + 4 : maxLayer;
               // else
               maxL = (layer + 3 <= maxLayer) ? layer + 3 : maxLayer;
               // outPutFile << x << " " << y << " " << layer + 1 << " " << x << " " << y << " " << layer + 2 << "\n";
               for (int l = layer + 1; l <= maxL; l++)
               {
                   outPutFile << x << " " << y << " " << l << " " << x << " " << y << " " << l << "\n";
                   for (int i = -1; i < 2; i++)
                   {
                       if (i == 0)
                           continue;
                       if (x + i >= 0 && x + i < rr_map->get_gridx())
                           outPutFile << x + i << " " << y << " " << l << " " << x << " " << y << " " << l << "\n";
                       if (y + i >= 0 && y + i < rr_map->get_gridx())
                           outPutFile << x << " " << y + i << " " << l << " " << x << " " << y << " " << l << "\n";
                       if (x + i >= 0 && x + i < rr_map->get_gridx() && y + i >= 0 && y + i < rr_map->get_gridx())
                           outPutFile << x + i << " " << y + i << " " << l << " " << x << " " << y << " " << l << "\n";
                   }
                   if (y - 1 >= 0 && x + 1 < rr_map->get_gridy())
                       outPutFile << x + 1 << " " << y - 1 << " " << l << " " << x << " " << y << " " << l << "\n";
                   if (x - 1 >= 0 && y + 1 < rr_map->get_gridy())
                       outPutFile << x - 1 << " " << y + 1 << " " << l << " " << x << " " << y << " " << l << "\n";
               }
           }
           outPutFile << ")\n";
       }
       */
    std::cout << " finish output " << netCount << " nets ! \n";

    outPutFile.close();

    return;
}

void Coala::Layerassignment::generateTileCongestionMap(string outputFileName)
{
    for (int z = 0; z < rr_map->get_layerNumber(); z++)
    {
        ofstream outPutFile;
        ofstream outPutFile2;
        std::cout << "writing cap file ... to "

                  << "/home/gary/nthu-route-gary/congestionMap/" + outputFileName + boost::lexical_cast<std::string>(z) + ".output"

                  << " ! \n";

        std::cout << "writing reduced cap file ... to "

                  << "/home/gary/nthu-route-gary/congestionMap/reduced" + outputFileName + boost::lexical_cast<std::string>(z) + ".output"

                  << " ! \n";

        outPutFile.open("/home/gary/nthu-route-gary/congestionMap/" + outputFileName + boost::lexical_cast<std::string>(z) + ".output");
        outPutFile2.open("/home/gary/nthu-route-gary/congestionMap/reduced-" + outputFileName + boost::lexical_cast<std::string>(z) + ".output");
        std::cout << " layer " << z + 1 << " \n";

        for (int j = 0; j < rr_map->get_gridy(); j++)

        {

            for (int i = 0; i < rr_map->get_gridx(); i++)

            {

                outPutFile << cur_map_3d[i][j][z].cap << " ";
                outPutFile2 << cur_map_3d[i][j][z].discountRatio << " ";
            }

            outPutFile << "\n";
            outPutFile2 << "\n";
        }

        outPutFile << "\n";
        outPutFile2 << "\n";
    }
}

void Coala::Layerassignment::generateEdgeCongestionMap(string outputFileName)

{

    for (int z = 0; z < rr_map->get_layerNumber(); z++)

    {

        ofstream outPutFile;

        std::cout << "writing file ... to "

                  << "/home/gary/" + outputFileName + boost::lexical_cast<std::string>(z) + ".output"

                  << " ! \n";

        outPutFile.open("/home/gary/" + outputFileName + boost::lexical_cast<std::string>(z) + ".output");

        if (database.getLayerDir(z) == X)

        {

            for (int i = 0; i < rr_map->get_gridx(); i++)

            {

                for (int j = 0; j < rr_map->get_gridy() - 1; j++)

                {

                    outPutFile << cur_map_3d[i][j][z].edge_list[FRONT]->cur_dem << " ";
                }

                outPutFile << "\n";
            }
        }

        else

        {

            for (int j = 0; j < rr_map->get_gridy(); j++)

            {

                for (int i = 0; i < rr_map->get_gridx() - 1; i++)

                {

                    outPutFile << cur_map_3d[i][j][z].edge_list[RIGHT]->cur_dem << " ";
                }

                outPutFile << "\n";
            }
        }

        outPutFile.close();
    }
}

void Coala::Layerassignment::addUnseenSeg()

{

    int netSize = rr_map->get_netNumber();

    for (int i = 0; i < netSize; i++)

    {

        for (auto &seg : coalaDB.netSegs[i])

        {

            if (seg->seen == Jm::SegmentSeen::No)

            {

                nextlayerSegSet.emplace_back(seg);

                seg->rstat = Jm::RerouteState::YES;
            }
        }
    }

    std::cout << "un seen start end\n\n\n";
}

void Coala::Layerassignment::run()

{

    std::cout << "set data base ! \n";

    coalaDB.setDataBase();

    std::cout << "finish set data base ! \n";

    auto layerNumber = rr_map->get_layerNumber();

    finalPath.resize(rr_map->get_netNumber());

    viasMap.resize(rr_map->get_netNumber());

    std::cout << "pin initialize start! \n";

    // layer assignment start !

    pinInitialize(0);

    std::cout << "start ! \n";

    // completeSegAssignment(0);

    // updateDemandPredMap(0);

    // generateTileCongestionMap("beforeAssignment");

    //   readTileCongestionMap("afterAssignment.output");

    for (int i = 1; i < layerNumber; i++)

    {

        std::cout << "layer " << i << " start !\n";

        viaAssignment(i - 1);

        std::cout << "via assignment complete ! \n";

        pinInitialize(i);

        std::cout << "pin initialize complete ! \n";

        completeSegAssignment(i);

        std::cout << "complete Seg Assignment complete ! \n";

        updateDemandPredMap(i);

        std::cout << "updateDemandPredMap complete ! \n";
    }

    nextlayerSegSet.insert(nextlayerSegSet.end(), invalidSegSet.begin(), invalidSegSet.end());

    invalidSegSet.clear();

    addUnseenSeg();

    Coala::CoalaReroute reroute((*this));

    reroute.removeRedundantPS(finalPath, viasMap, pointToPin);

    reroute.aStar3D(finalPath, viasMap);
    std::chrono::steady_clock::time_point startIO = std::chrono::steady_clock::now();

    generateTileCongestionMap("capacityMap");

    // addPinPatch();

    //  generateEdgeCongestionMap("afterAssignmentEdge");

    generateOutput();
    std::chrono::steady_clock::time_point endIO = std::chrono::steady_clock::now();
    std::cout << " generate Output with ... " << std::chrono::duration_cast<std::chrono::seconds>(endIO - startIO).count() << " s ! \n";
}
