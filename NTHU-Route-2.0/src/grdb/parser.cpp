// File: grdb/parser.cpp
// Brief: Parser for parsing test case of ISPD'07 and ISPD'98
// Author: Yen-Jung Chang
// $Date: 2007-11-26 17:18:13 +0800 (Mon, 26 Nov 2007) $
// $Revision: 13 $

#include "parser.h"
#include "misc/debug.h"
#include <string>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <limits>
#include <tuple>
#include <iostream>
#include "gr_db/GrNet.h"
#include <fstream>
#include "../router/parameter.h"

#define MAX_STRING_BUFER_LENGTH 512
#define MAX_PIN 1000

using std::cerr;
using std::endl;

extern std::string TESTCASE_NAME;
extern RoutingParameters *routing_parameter;
 
struct hash_tuple
{ // hash binary tuple
    template <class T>
    size_t operator()(const std::tuple<T, T> &tup) const
    {
        auto hash1 = std::hash<T>{}(std::get<0>(tup));
        auto hash2 = std::hash<T>{}(std::get<1>(tup));
        return hash1 ^ hash2;
    }
};
//====== GRParser =========
// virtual
GRParser::~GRParser() {}

//====== Parser07 =========
//{{{
// virtual
Parser07::~Parser07() {}

//====== Parser18 =========
Parser18::~Parser18() {}

//====== Parser24 =========
Parser24::~Parser24() {}

void Parser24::setNetList()
{
    int numNets = this->nets.size();
    int numPins;
    int x, y, z;

    builder_->setNetNumber(numNets);
    unsigned int i = 0;
    for (auto &[netName, net] : this->nets) {
        numPins = net.numPins;
        builder_->beginAddANet(netName.c_str(), i, numPins, 0);
        for (auto &Pin : net.accessPoints) {
            for (auto &AP : Pin) {
                z = AP[0];
                x = AP[1];
                y = AP[2];
                builder_->addPin(x, y, z);
                break;
            }
        }
        builder_->endAddANet();
        ++i;
    }
}

void Parser24::parseNetFile()
{
    std::ifstream netFile(this->net_file);

    std::string line;
    std::string name;
    int numPins = 0;
    // std::vector<Net> nets;
    // std::unordered_map<std::string, Net> nets;
    // int netId = -1;
    std::vector<std::vector<std::vector<int>>> accessPoints;
    while (std::getline(netFile, line)) {
        // if (line.find("net") != std::string::npos || line.find("pin") != std::string::npos) {
        if (line.find("(") == std::string::npos && line.find(")") == std::string::npos && line.length()>1) {
            // name = line.substr(0, line.size() - 1);
            name = line;
            size_t found = name.find('\n');
            if (found != std::string::npos) {
                name.erase(found, 1);
            }
            // netId = std::stoi(line.substr(3));
            numPins = 0;
            // std::cout << name << " " << netId << std::endl;
        } else if (line.find('[') != std::string::npos) {
            std::vector<std::vector<int>> access;
            std::string text = line.substr(1, line.size() - 2); // Remove brackets and trailing comma
            std::string charsToRemove = "(),";
            text.erase(std::remove_if(text.begin(), text.end(), [&charsToRemove](char c) {
                return charsToRemove.find(c) != std::string::npos;
            }), text.end());
            // std::cout << "current line is: " << text << std::endl;
            std::istringstream ss(text);
            int x, y, z;
            while (ss >> x >> y >> z) {
                std::vector<int> point;
                point.push_back(x);
                point.push_back(y);
                point.push_back(z);
                access.push_back(point);

            }
            accessPoints.push_back(access);
            numPins++;
        } else if (line.find(')') != std::string::npos) {
            // if (netId == 3153 || netId == 3151) {
            //     std::cout << name << " " << netId << " " << numPins << std::endl;
            // }
            // nets.push_back(Net(name, numPins, accessPoints));
            NetISPD24 net(name, numPins, accessPoints);
            this->nets[name] = net;
            accessPoints.clear();
        }
    }
    // std::cout << "Got net info" << std::endl;

    netFile.close();
}

void Parser24::setEdgeCapacity()
{
    int nLayers = this->GcellCapacity.size();
    int xSize = this->GcellCapacity[0].size();
    int ySize = this->GcellCapacity[0][0].size();

    builder_->setGrid(xSize, ySize, nLayers);
    for (int i = 0; i < nLayers; ++i)
    {
        builder_->setLayerMinimumWidth(i, 0);
    }
    for (int i = 0; i < nLayers; ++i)
    {
        builder_->setLayerMinimumSpacing(i, 0);
    }
    for (int i = 0; i < nLayers; ++i)
    {
        builder_->setViaSpacing(i, 1);
    }

    bool isVertical = false; //TODO don't hardcode it
    int resource, prev;
    for (int layer=0; layer<nLayers; ++layer) {
        if (isVertical) {
            for (int i=0; i<xSize; ++i) {
                for (int j=0; j<ySize-1; ++j) {
                    resource = max(0, static_cast<int>(min(this->GcellCapacity[layer][i][j], this->GcellCapacity[layer][i][j+1])));
                    if (layer == 0)
                        resource = 0;
                    builder_->adjustEdgeCapacity(i, j, layer, i, j+1, layer, resource);
                }
            }
            // for (int i=0; i<xSize-1; ++i) {
            //     for (int j=0; j<ySize; ++j) {
            //         builder_->adjustEdgeCapacity(i, j, layer, i+1, j, layer, 0);
            //     }
            // }
        } else {
            for (int j=0; j<ySize; ++j) {
                for (int i=0; i<xSize-1; ++i) {
                    resource = max(0, static_cast<int>(min(this->GcellCapacity[layer][i][j], this->GcellCapacity[layer][i+1][j])));
                    if (layer == 0)
                        resource = 0;
                    builder_->adjustEdgeCapacity(i, j, layer, i+1, j, layer, resource);
                }
            }
            // for (int i=0; i<xSize; ++i) {
            //     for (int j=0; j<ySize-1; ++j) {
            //         builder_->adjustEdgeCapacity(i, j, layer, i, j+1, layer, 0);
            //     }
            // }
        }
        isVertical = !isVertical;
    }
    for (int l = 0; l < nLayers; ++l) {
        if (l % 2 == 0) {
            for (int i=0; i<xSize; ++i) {
                for (int j=0; j<ySize-1; ++j) {
                    assert(builder_->capacity(l, i, j, i, j+1) == 0);
                }
            }
        } else {
            for (int i=0; i<xSize-1; ++i) {
                for (int j=0; j<ySize; ++j) {
                    assert(builder_->capacity(l, i, j, i+1, j) == 0);
                }
            }
        }
    }
}

void Parser24::parseCapFile()
{
    std::ifstream resourceFile(this->cap_file);

    std::string line;
    std::getline(resourceFile, line);
    std::vector<int> dimensions;
    std::istringstream iss(line);
    for (int value; iss >> value;) {
        dimensions.push_back(value);
    }
    int nLayers = dimensions[0];
    int xSize = dimensions[1];
    int ySize = dimensions[2];

    std::cout << "dimensions: " << nLayers << " " << xSize << " " << ySize << std::endl;

    // Get unit costs
    std::getline(resourceFile, line);
    std::istringstream unitCosts(line);
    double unit_length_wire_cost, unit_via_cost;
    unitCosts >> unit_length_wire_cost >> unit_via_cost;

    std::vector<double> unit_length_short_costs;
    double cost;
    while (unitCosts >> cost) {
        unit_length_short_costs.push_back(cost);
    }
    std::cout << "Got unit costs: " << unit_length_wire_cost << " " << unit_via_cost << " "  << std::endl;

    // Get edge lengths
    std::vector<double> hEdgeLengths;
    std::getline(resourceFile, line);
    std::istringstream hEdgeLine(line);
    while (hEdgeLine >> cost) {
        hEdgeLengths.push_back(cost);
    }

    // for (int x=0; x<xSize-1; ++x) {
    //     std::cout << "hedge " << x << " length " << hEdgeLengths[x] << std::endl;
    // }

    std::vector<double> vEdgeLengths;
    std::getline(resourceFile, line);
    std::istringstream vEdgeLine(line);
    while (vEdgeLine >> cost) {
        vEdgeLengths.push_back(cost);
    }
    // for (int y=0; y<ySize-1; ++y) {
    //     std::cout << "vedge " << y << " length " << vEdgeLengths[y] << std::endl;
    // }

    // std::cout << "Got edge lengths: " << std::endl;

    // Get capacity map
    // std::vector<std::vector<std::vector<double>>> GcellCapacity(nLayers, std::vector<std::vector<double>>(xSize, std::vector<double>(ySize)));
    this->GcellCapacity.assign(nLayers, std::vector<std::vector<double>>(xSize, std::vector<double>(ySize)));
    
    std::vector<int> layerDirections;
    std::vector<double> layerMinLengths;

    for (int l = 0; l < nLayers; ++l) {
        std::getline(resourceFile, line);
        std::istringstream layerInfo(line);
        std::string layer_name;
        layerInfo >> layer_name;
        int direction;
        layerInfo >> direction;
        layerDirections.push_back(direction);
        double min_length;
        layerInfo >> min_length;
        layerMinLengths.push_back(min_length);

        for (int y = 0; y < ySize; ++y) {
            std::getline(resourceFile, line);
            std::istringstream capacityInfo(line);
            for (int x = 0; x < xSize; ++x) {
                capacityInfo >> this->GcellCapacity[l][x][y];
            }
        }
    }
    // std::cout << "Got capacity map" << std::endl;

    for (int l=0; l< nLayers; ++l) {
        std::cout << "Layer " << l << " direction " << layerDirections[l] << " minLength " << layerMinLengths[l] << std::endl;
    }

    resourceFile.close();
}

void Parser24::parse(Builder *builder)
{
    assert(builder);
    builder_ = builder;

    this->parseCapFile();
    this->setEdgeCapacity();

    this->parseNetFile();
    this->setNetList();
}  

void Parser18::parseNetList()
{
    std::string netlist_file = "/users/student/mr109/ykfang20/05_15_paper/TritonRoute-WXL/build/" + TESTCASE_NAME + "/netlist.txt";
    std::ifstream fin(netlist_file);
    assert(fin.is_open());
    std::string netName;
    int numNets;
    int numPins;
    int x, y, z;

    fin >> numNets;
    std::cout << "#Nets = " << numNets << '\n';
    builder_->setNetNumber(numNets);
    for (int i=0; i<numNets; ++i) {
        fin >> netName >> numPins;
        builder_->beginAddANet(netName.c_str(), i, numPins, 0);
        for (int j=0; j<numPins; ++j) {
            fin >> x >> y >> z;
            builder_->addPin(x, y, z);
            // if (netName == "pin1") {
            //     std::cout << "Pin (" << x << ", " << y << ", " << z << ") " << j+1 << "\n";
            // }
        }
        builder_->endAddANet();
        // if (netName == "pin1") {
        //     std::cout << "net pin1: " << numPins << " read.\n";
        // }
    }
    fin.close();
 }

void Parser18::parseCmap()
{
    std::string cmap_file = "/users/student/mr109/ykfang20/05_15_paper/TritonRoute-WXL/build/" + TESTCASE_NAME + "/cmap.txt";
    std::ifstream fin(cmap_file);
    assert(fin.is_open());

    int x, y, layerNumber;
    fin >> x >> y >> layerNumber;
    builder_->setGrid(x, y, layerNumber);
    for (int i = 0; i < layerNumber; ++i)
    {
        builder_->setLayerMinimumWidth(i, 0);
    }
    for (int i = 0; i < layerNumber; ++i)
    {
        builder_->setLayerMinimumSpacing(i, 0);
    }
    for (int i = 0; i < layerNumber; ++i)
    {
        builder_->setViaSpacing(i, 1);
    }

    std::vector<std::vector<std::vector<std::pair<int, int>>>> Gcell(x, std::vector<std::vector<std::pair<int, int>>>(y, std::vector<std::pair<int, int>>(layerNumber, {0, 0})));

    for (int layer=0; layer<layerNumber; ++layer) {
        for (int i=0; i<x*y; ++i) {
            int x_coor, y_coor, z_coor, supply, demand;
            fin >> x_coor >> y_coor >> z_coor >> supply >> demand;
            assert(supply >= 0);
            assert(demand >= 0);
            Gcell[x_coor][y_coor][z_coor].first = supply;
            Gcell[x_coor][y_coor][z_coor].second = demand;
        }
    }
    fin.close();

    bool isVertical = true;
    int resource, prev;
    for (int layer=0; layer<layerNumber; ++layer) {
        if (isVertical) {
            for (int i=0; i<x; ++i) {
                for (int j=0; j<y-1; ++j) {
                    resource = max(0, min(Gcell[i][j][layer].first - Gcell[i][j][layer].second, Gcell[i][j+1][layer].first - Gcell[i][j+1][layer].second));
                    
                    if (Gcell[i][j][layer].second > 0 || Gcell[i][j+1][layer].second > 0)
                        resource *= routing_parameter->get_init_reduct();
                    if (layer == 0) {
                        resource = 0;
                    }
                    builder_->adjustEdgeCapacity(i, j, layer, i, j+1, layer, resource);
                    // std::cout << "Edge(" << i << "," << j << ")->(" << i << "," << j+1 << ") capacity =" << resource
                    // << " = min(Gcell[" << i << "][" << j << "][" << layer << "]'s supply(" << Gcell[i][j][layer].first << ") - demand(" << Gcell[i][j][layer].second << "),"
                    // << "Gcell[" << i << "][" << j+1 << "][" << layer << "]'s supply(" << Gcell[i][j+1][layer].first << ") - demand(" << Gcell[i][j+1][layer].second << "))\n";
                }
            }
        } else {
            for (int j=0; j<y; ++j) {
                for (int i=0; i<x-1; ++i) {
                    resource = max(0, min(Gcell[i][j][layer].first - Gcell[i][j][layer].second, Gcell[i+1][j][layer].first - Gcell[i+1][j][layer].second));
                    
                    if (Gcell[i][j][layer].second > 0 || Gcell[i+1][j][layer].second > 0)
                        resource *= routing_parameter->get_init_reduct();
                    if (layer == 0) {
                        resource = 0;
                    }
                    builder_->adjustEdgeCapacity(i, j, layer, i+1, j, layer, resource);
                    // std::cout << "Edge(" << i << "," << j << ")->(" << i+1 << "," << j << ") capacity = " << resource
                    // << " = min(Gcell[" << i << "][" << j << "][" << layer << "]'s supply(" << Gcell[i][j][layer].first << ")-demand(" << Gcell[i][j][layer].second << "),"
                    // << "Gcell[" << i+1 << "][" << j << "][" << layer << "]'s supply(" << Gcell[i+1][j][layer].first << ")-demand(" << Gcell[i+1][j][layer].second << "))\n";
                }
            }
        }
        isVertical = !isVertical;
    }
}

void Parser18::parse(Builder *builder)
{
    assert(builder);
    builder_ = builder;
    parseCmap();
    parseNetList();
}

void Parser18::parseRoutingRegion()
{
    int x = grDatabase.getNumGrLine(X) - 1;
    int y = grDatabase.getNumGrLine(Y) - 1;
    int layerNumber = database.getLayerNum();
    builder_->setGrid(x, y, layerNumber);
    // std::cout << x << ' ' << y << ' ' << layerNumber << '\n';
    // exit(0);
    for (int i = 0; i < layerNumber; ++i)
    {
        builder_->setLayerMinimumWidth(i, 0);
    }
    for (int i = 0; i < layerNumber; ++i)
    {
        builder_->setLayerMinimumSpacing(i, 0);
    }
    for (int i = 0; i < layerNumber; ++i)
    {
        builder_->setViaSpacing(i, 1);
    }
}

void Parser18::parseNets()
{
    int netNumber = grDatabase.nets.size();
    std::cout << netNumber << '\n';
    builder_->setNetNumber(netNumber);
    for (int i = 0; i < netNumber; ++i)
    {
        parseANet(i);
    }
    exit(0);
}

void calculatePinCenter(gr::GrNet &net, std::vector<std::vector<int>> &pinCenters, Builder *builder_)
{
    float net_ctrx = 0;
    float net_ctry = 0;
    for (auto &pinBoxes : net.pinAccessBoxes)
    {
        // float pin_ctrz = 0;
        float pin_ctrx = 0;
        float pin_ctry = 0;
        for (auto &pinBox : pinBoxes)
        {
            pin_ctrx += pinBox.x;
            pin_ctry += pinBox.y;
        }
        pin_ctrx /= pinBoxes.size();
        pin_ctry /= pinBoxes.size();

        net_ctrx += pin_ctrx;
        net_ctry += pin_ctry;
    }
    net_ctrx /= net.pinAccessBoxes.size();
    net_ctry /= net.pinAccessBoxes.size();

    for (auto &pinBoxes : net.pinAccessBoxes)
    {
        float xCenter = 0;
        float yCenter = 0;
        for (auto &pinBox : pinBoxes)
        {
            xCenter += pinBox.x;
            yCenter += pinBox.y;
        }
        xCenter /= pinBoxes.size();
        yCenter /= pinBoxes.size();
        // 3 kinds of accessibility (0) totally vio-free (1) one side vio-free (2) no side vio-free
        float min_dist[3];
        min_dist[0] = min_dist[1] = min_dist[2] = std::numeric_limits<float>::max();
        int best_box[3] = {-1, -1, -1};
        std::vector<int> goodbox;
        for (int pb = 0; pb < pinBoxes.size(); pb++)
        {
            // check pin box's accessibility
            auto &pb_x = pinBoxes[pb].x;
            auto &pb_y = pinBoxes[pb].y;
            int layer_idx = pinBoxes[pb].layerIdx;
            int is_x = database.getLayerDir(layer_idx) == X ? 1 : 0;
            auto low_edge =
                gr::GrEdge({layer_idx, max(pb_x - (1 - is_x), 0), max(pb_y - is_x, 0)}, {layer_idx, pb_x, pb_y});
            auto high_edge = gr::GrEdge({layer_idx, pb_x, pb_y},
                                        {layer_idx,
                                         min(pb_x + (1 - is_x), grDatabase.getNumGrPoint(X) - 1),
                                         min(pb_y + is_x, grDatabase.getNumGrPoint(Y) - 1)});

            int pb_access = 0;
            pb_access += int(grDatabase.hasVio(low_edge, false));
            pb_access += int(grDatabase.hasVio(high_edge, false));

            float dist = abs(pinBoxes[pb].x - net_ctrx) + abs(pinBoxes[pb].y - net_ctrx);
            if (dist < min_dist[pb_access])
            {
                min_dist[pb_access] = dist;
                best_box[pb_access] = pb;
            }
        }
        for (int ac = 0; ac < 3; ac++)
        {
            if (best_box[ac] != -1)
            {
                int pbl = pinBoxes[best_box[ac]].layerIdx;
                if (pbl > 0)
                {
                    std::vector<std::vector<int>> dir({{0, 0}, {0, 1}, {1, 0}, {-1, 0}, {0, -1}});
                    bool addf = false;
                    for (int i = 1; i < 3; i++)
                    {
                        for (auto &d : dir)
                        {
                            int pbx = pinBoxes[best_box[ac]].x + d[0] * i;
                            int pby = pinBoxes[best_box[ac]].y + d[1] * i;
                            int is_x = database.getLayerDir(pbl) == X ? 1 : 0;
                            if (pbx >= grDatabase.getNumGrPoint(X) - 1 || pby >= grDatabase.getNumGrPoint(Y) - 1 || pbx <= 1 || pby <= 1)
                                continue;
                            auto low_edge =
                                gr::GrEdge({pbl, max(pbx - (1 - is_x), 0), max(pby - is_x, 0)}, {pbl, pbx, pby});
                            auto high_edge = gr::GrEdge({pbl, pbx, pby},
                                                        {pbl,
                                                         min(pbx + (1 - is_x), grDatabase.getNumGrPoint(X) - 1),
                                                         min(pby + is_x, grDatabase.getNumGrPoint(Y) - 1)});

                            int capl = grDatabase.getWireCapacity(low_edge) - grDatabase.getFixedUsage(low_edge) - 1;
                            int caph = grDatabase.getWireCapacity(high_edge) - grDatabase.getFixedUsage(high_edge) - 1;
                            if (capl + caph > 0)
                            {
                                pinCenters.emplace_back(std::vector({pbx, pby, pinBoxes[best_box[ac]].layerIdx}));
                                addf = true;
                                break;
                            }
                        }
                        if (addf)
                        {
                            break;
                        }
                    }
                    if (addf)
                    {
                        break;
                    }
                }
                pinCenters.emplace_back(std::vector({pinBoxes[best_box[ac]].x, pinBoxes[best_box[ac]].y, pinBoxes[best_box[ac]].layerIdx}));

                break;
            }
        }
    }
}

void Parser18::parseANet(int index)
{
    auto &net = grDatabase.nets[index];
    string netName = net.getName();
    int pinNumber = net.numOfPins();
     if (netName == "net340182")
        std::cout << netName << ' ' << index << ' ' << pinNumber << '\n';
    int minWidth = 0;
    builder_->beginAddANet(netName.c_str(), index, pinNumber, minWidth);
    std::vector<std::vector<int>> pinCenters;
    calculatePinCenter(net, pinCenters, builder_);
    int cnt = 0;
    for (auto &pin : pinCenters)
    {
        cnt++;
        if (netName == "net340182")
            std::cout << "\tPin " << pin[0] << ' ' << pin[1] << " " << pin[2] << '\n';
        builder_->addPin(pin[0], pin[1], pin[2]); // Gcell x, Gcell y, Gcell z (0-indexed)
    }
    assert(cnt == pinNumber);
    builder_->endAddANet();
}
void Parser18::setCapacity()
{
    int xNumGrPoint = grDatabase.getNumGrPoint(X);
    int yNumGrPoint = grDatabase.getNumGrPoint(Y);
    int xNumGrEdge = yNumGrPoint - 1;
    int yNumGrEdge = xNumGrPoint - 1;
    int numLayers = database.getLayerNum();
    for (int l = 0; l < numLayers; l++)
    {
        auto dir = database.getLayerDir(l);
        if (dir == X)
        {
            for (int gridline = 0; gridline < xNumGrPoint; gridline++)
            {
                for (int cp = 0; cp < xNumGrEdge; cp++)
                {
                    gr::GrEdge tempEdge(l, gridline, cp);
                    int cap = grDatabase.getWireCapacity(tempEdge) - grDatabase.getFixedUsage(tempEdge);
                    bool gt_zero = cap > 0;

                    if (grDatabase.getFixedUsage(tempEdge) > 0 && l != 3)
                        cap *= 0.8;
                    if (gt_zero)
                        cap = max(cap, 1);
                    
                    /*if (grDatabase.getFixedUsage(tempEdge) > 0)
                    {
                        if (cap > 0)
                            cap--;
                    }*/
                    if (cap < 0)
                        cap++;
                    if (l == 0)
                    {

                        (*layerOneCap)[tempEdge.u.x][tempEdge.u.y] += cap;
                        (*layerOneCap)[tempEdge.v.x][tempEdge.v.y] += cap;
                        cap = 0;
                    }
                    
                    else if (l == 3)
                    {
                        if (grDatabase.getFixedUsage(tempEdge) > 0) cap *= 0.5;
                        if (gt_zero)
                            cap = max(cap, 1);
                    }
                    
                    builder_->adjustEdgeCapacity(tempEdge.u.x, tempEdge.u.y, tempEdge.getLayerIdx(),
                                                 tempEdge.v.x, tempEdge.v.y, tempEdge.getLayerIdx(),
                                                 cap);
                }
            }
        }
        else
        {

            for (int gridline = 0; gridline < yNumGrPoint; gridline++)
            {
                for (int cp = 0; cp < yNumGrEdge; cp++)
                {
                    gr::GrEdge tempEdge(l, gridline, cp);
                    int cap = grDatabase.getWireCapacity(tempEdge) - grDatabase.getFixedUsage(tempEdge);
                    bool gt_zero = cap > 0;
                    
                    /* if (grDatabase.getFixedUsage(tempEdge) > 0)
                     {
                         if (cap > 0)
                             cap--;
                     }*/

                    if (grDatabase.getFixedUsage(tempEdge) > 0 && l != 3)
                        cap *= 0.8;
                    if (gt_zero)
                        cap = max(cap, 1);
                    
                    if (cap < 0)
                        cap++;
                    if (l == 0)
                    {
                        (*layerOneCap)[tempEdge.u.x][tempEdge.u.y] += cap;
                        (*layerOneCap)[tempEdge.v.x][tempEdge.v.y] += cap;
                        cap = 0;
                    }
                    else if (l == 3)
                    {
                        if (grDatabase.getFixedUsage(tempEdge) > 0) cap *= 0.5;
                        if (gt_zero)
                            cap = max(cap, 1);
                    }
                    
                    /*
                    else if (l >= 6)
                    {
                        cap *= 0.9;
                    }*/
                    
                    builder_->adjustEdgeCapacity(tempEdge.u.x, tempEdge.u.y, tempEdge.getLayerIdx(),
                                                 tempEdge.v.x, tempEdge.v.y, tempEdge.getLayerIdx(),
                                                 cap);
                }
            }
        }
    }
}
// virtual
void Parser07::parse(Builder *builder)
{
    assert(builder);
    builder_ = builder;

    // Begin to parse file
    if (fh_.open(FileHandler::ReadAccessMode))
    {
        parseRoutingRegion();
        parseNets();
        adjustCapacity();
    }
    else
    {
        cerr << "Error opening test case." << endl;
        abort();
    }

    builder_->endBuild();
}

// virtual
void Parser07::parseRoutingRegion()
{
    // buffer for strtok()
    char *stringBuffer = new char[MAX_STRING_BUFER_LENGTH];

    // Get grid size and layer number
    fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
    // get grid size
    atoi(strtok(stringBuffer, delims_.c_str())); // "grid" string
    int x = atoi(strtok(NULL, delims_.c_str()));
    int y = atoi(strtok(NULL, delims_.c_str()));
    int layerNumber = atoi(strtok(NULL, delims_.c_str()));
    builder_->setGrid(x, y, layerNumber);

    // Set vertical capacity
    fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
    strtok(stringBuffer, delims_.c_str()); // "vertical"
    strtok(NULL, delims_.c_str());         // "capacity"
    // for each layer, set the vertical capacity
    for (int i = 0; i < layerNumber; ++i)
    {
        int capacity = atoi(strtok(NULL, delims_.c_str()));
        builder_->setVerticalCapacity(i, capacity);
    }

    // Set horizontal capacity
    fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
    strtok(stringBuffer, delims_.c_str()); // "horizontal"
    strtok(NULL, delims_.c_str());         // "capacity"
    // for each layer, set the horizontal capacity
    for (int i = 0; i < layerNumber; ++i)
    {
        int capacity = atoi(strtok(NULL, delims_.c_str()));
        builder_->setHorizontalCapacity(i, capacity);
    }

    // Set minimum width
    fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
    strtok(stringBuffer, delims_.c_str()); // "minimum"
    strtok(NULL, delims_.c_str());         // "width"
    // for each layer, set the minimum width
    for (int i = 0; i < layerNumber; ++i)
    {
        int width = atoi(strtok(NULL, delims_.c_str()));
        builder_->setLayerMinimumWidth(i, width);
    }

    // Set minimum spacing
    fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
    strtok(stringBuffer, delims_.c_str()); // "minimum"
    strtok(NULL, delims_.c_str());         // "spacing"
    // for each layer, set the minimum spacing
    for (int i = 0; i < layerNumber; ++i)
    {
        int spacing = atoi(strtok(NULL, delims_.c_str()));
        builder_->setLayerMinimumSpacing(i, spacing);
    }

    // Set via spacing
    fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
    strtok(stringBuffer, delims_.c_str()); // "via"
    strtok(NULL, delims_.c_str());         // "spacing"
    // for each layer, set the via spacing
    for (int i = 0; i < layerNumber; ++i)
    {
        int spacing = atoi(strtok(NULL, delims_.c_str()));
        builder_->setViaSpacing(i, spacing);
    }

    // Set tile transformation information
    fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
    int llx = atoi(strtok(stringBuffer, delims_.c_str()));
    int lly = atoi(strtok(NULL, delims_.c_str()));
    int tileWidth = atoi(strtok(NULL, delims_.c_str()));
    int tileHeight = atoi(strtok(NULL, delims_.c_str()));
    builder_->setTileTransformInformation(llx, lly, tileWidth, tileHeight);
    delete[] stringBuffer;
}

void Parser07::parseNets()
{
    // buffer for strtok()
    char *stringBuffer = new char[MAX_STRING_BUFER_LENGTH];

    // get rid of empty line
    do
    {
        memset(stringBuffer, '\0', MAX_STRING_BUFER_LENGTH);
        fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
    } while (!isalnum(stringBuffer[0]));
    // Get total net number
    strtok(stringBuffer, delims_.c_str());               // "num"
    strtok(NULL, delims_.c_str());                       // "net"
    int netNumber = atoi(strtok(NULL, delims_.c_str())); // read total net number
    builder_->setNetNumber(netNumber);

    for (int i = 0; i < netNumber; ++i)
    {
        parseANet();
    }

    delete[] stringBuffer;
}

void Parser07::parseANet()
{
    // buffer for strtok()
    char *stringBuffer = new char[MAX_STRING_BUFER_LENGTH];

    // Get net information
    fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
    string netName = strtok(stringBuffer, delims_.c_str());
    int netSerial = atoi(strtok(NULL, delims_.c_str())); // read net serial number
    int pinNumber = atoi(strtok(NULL, delims_.c_str())); // read pin number
    int minWidth = atoi(strtok(NULL, delims_.c_str()));  // read net minmum width

    if (pinNumber <= MAX_PIN)
    {                                                                            // pin# > 1000 is a special net, we can skip it
        builder_->beginAddANet(netName.c_str(), netSerial, pinNumber, minWidth); // Add a net to DB
        // reading pin information of a net
        for (int j = 0; j < pinNumber; ++j)
        {
            fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
            int x = atoi(strtok(stringBuffer, delims_.c_str())); // x of pin
            int y = atoi(strtok(NULL, delims_.c_str()));         // y of pin
            int layer = atoi(strtok(NULL, delims_.c_str()));     // layer of pin
            builder_->addPin(x, y, layer - 1);                   // Add pin to a net
        }
        builder_->endAddANet(); // end of reading a net
    }
    else
    {
        // skip reading net information with >1000 pins
        for (int j = 0; j < pinNumber; ++j)
        {
            fh_.skipline();
        }
    }

    delete[] stringBuffer;
}

void Parser07::adjustCapacity()
{
    // buffer for strtok()
    char *stringBuffer = new char[MAX_STRING_BUFER_LENGTH];

    // get rid of empty line
    do
    {
        memset(stringBuffer, '\0', MAX_STRING_BUFER_LENGTH);
        fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
    } while (!isalnum(stringBuffer[0]));

    // Get adjustment number
    // get the total edge adjusting number
    int adjustNumber = atoi(strtok(stringBuffer, delims_.c_str()));

    for (int i = 0; i < adjustNumber; ++i)
    {
        fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
        // reading source gCell
        int x1 = atoi(strtok(stringBuffer, delims_.c_str()));
        int y1 = atoi(strtok(NULL, delims_.c_str()));
        int z1 = atoi(strtok(NULL, delims_.c_str()));
        // reading sink gCell
        int x2 = atoi(strtok(NULL, delims_.c_str()));
        int y2 = atoi(strtok(NULL, delims_.c_str()));
        int z2 = atoi(strtok(NULL, delims_.c_str()));
        // reading the new capacity
        int capacity = atoi(strtok(NULL, delims_.c_str()));
        builder_->adjustEdgeCapacity(x1, y1, z1 - 1,
                                     x2, y2, z2 - 1,
                                     capacity);
    }
    delete[] stringBuffer;
}
//}}}

//====== Parser98 =======
//{{{
Parser98::~Parser98()
{
}

void Parser98::parse(Builder *builder)
{
    assert(builder);
    builder_ = builder;

    // Begin to parse file
    if (fh_.open(FileHandler::ReadAccessMode))
    {
        parseRoutingRegion();
        parseNets();
    }
    else
    {
        cerr << "Error opening test case." << endl;
        abort();
    }

    builder_->endBuild();
}

void Parser98::parseRoutingRegion()
{
    char *stringBuffer = new char[MAX_STRING_BUFER_LENGTH]; // buffer for strtok()

    // Get grid size and layer number
    fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
    // get grid size
    strtok(stringBuffer, delims_.c_str()); // "grid" string
    int x = atoi(strtok(NULL, delims_.c_str()));
    int y = atoi(strtok(NULL, delims_.c_str()));
    builder_->setGrid(x, y, 1); // All test cases in ISPD'98 are single layer

    // Set vertical capacity
    fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
    strtok(stringBuffer, delims_.c_str()); // "vertical"
    strtok(NULL, delims_.c_str());         // "capacity"
    // set the vertical capacity
    int capacity = atoi(strtok(NULL, delims_.c_str()));
    builder_->setVerticalCapacity(0, capacity);

    // Set horizontal capacity
    fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
    strtok(stringBuffer, delims_.c_str()); // "horizontal"
    strtok(NULL, delims_.c_str());         // "capacity"
    // set the horizontal capacity
    capacity = atoi(strtok(NULL, delims_.c_str()));
    builder_->setHorizontalCapacity(0, capacity);

    // Set minimum width
    int width = 1;
    builder_->setLayerMinimumWidth(0, width);

    // Set minimum spacing
    int spacing = 0;
    builder_->setLayerMinimumSpacing(0, spacing);

    // Set via spacing
    int viaSpacing = 0;
    builder_->setViaSpacing(0, viaSpacing);

    // Set tile transformation information
    int llx = 0;
    int lly = 0;
    int tileWidth = 1;
    int tileHeight = 1;
    builder_->setTileTransformInformation(llx, lly, tileWidth, tileHeight);
    delete[] stringBuffer;
}

void Parser98::parseNets()
{
    // buffer for strtok()
    char *stringBuffer = new char[MAX_STRING_BUFER_LENGTH];

    // get rid of empty line
    do
    {
        memset(stringBuffer, '\0', MAX_STRING_BUFER_LENGTH);
        fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
    } while (!isalnum(stringBuffer[0]));
    // Get total net number
    strtok(stringBuffer, delims_.c_str()); // "num"
    strtok(NULL, delims_.c_str());         // "net"
    int netNumber = atoi(strtok(NULL, delims_.c_str()));
    builder_->setNetNumber(netNumber);

    for (int i = 0; i < netNumber; ++i)
    {
        parseANet();
    }
    delete[] stringBuffer;
}

void Parser98::parseANet()
{
    // buffer for strtok()
    char *stringBuffer = new char[MAX_STRING_BUFER_LENGTH];
    // Get net information
    fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
    string netName = strtok(stringBuffer, delims_.c_str());
    int netSerialNumber = atoi(strtok(NULL, delims_.c_str()));
    int pinNumber = atoi(strtok(NULL, delims_.c_str()));
    int minWidth = 1;

    if (pinNumber < MAX_PIN)
    { // pin# > 1000 is a special net, we don't need to route it
        builder_->beginAddANet(netName.c_str(), netSerialNumber, pinNumber, minWidth);
        for (int j = 0; j < pinNumber; ++j)
        {
            fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
            int x = atoi(strtok(stringBuffer, delims_.c_str()));
            int y = atoi(strtok(NULL, delims_.c_str()));
            builder_->addPin(x, y, 0);
        }
        builder_->endAddANet(); // end of reading a net
    }
    else
    {
        for (int j = 0; j < pinNumber; ++j)
        {
            fh_.skipline();
        }
    }
    delete[] stringBuffer;
}

//}}}
