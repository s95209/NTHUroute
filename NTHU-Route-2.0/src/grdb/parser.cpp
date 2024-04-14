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

std::vector<int> layerDirections;
 
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
        builder_->beginAddANet(netName.c_str(), numPins, 0);
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
    std::vector<std::vector<std::vector<int>>> accessPoints;
    while (std::getline(netFile, line)) {
        // 每個net的第一行
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
        //每個net的每個pin
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
        //pin中的 access point
            while (ss >> x >> y >> z) {
                std::vector<int> point;
                point.push_back(x);
                point.push_back(y);
                point.push_back(z);
                access.push_back(point);

            }
            accessPoints.push_back(access);
            numPins++;
        //每個net的結尾            
        } else if (line.find(')') != std::string::npos) {
            NetISPD24 net(name, numPins, accessPoints);
            this->nets[name] = net;
            accessPoints.clear();
        }
    }
    netFile.close();
}

void Parser24::setEdgeCapacity()
{
    int nLayers = this->GcellCapacity.size();
    int xSize = this->GcellCapacity[0].size();
    int ySize = this->GcellCapacity[0][0].size();
    std::cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" <<std::endl;
    builder_->setGrid(xSize, ySize, nLayers);
    std::cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" <<std::endl;
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
    for (int layer = 0; layer < nLayers; ++layer) {
        if (layerDirections[layer]) {
            for (int i = 0; i < xSize; ++i) {
                for (int j = 0; j < ySize - 1; ++j) {
                    resource = this->GcellCapacity[layer][i][j];
                    if (layer == 0)
                        resource = 0;
                    builder_->adjustEdgeCapacity(i, j, layer, i, j + 1, layer, resource);
                }
            }
        } else {
            for (int j = 0; j < ySize; ++j) {
                for (int i = 0; i < xSize-1; ++i) {
                    resource = this->GcellCapacity[layer][i][j];
                    if (layer == 0)
                        resource = 0;
                    builder_->adjustEdgeCapacity(i, j, layer, i+1, j, layer, resource);
                }
            }
        }
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
    std::cout << "parse" << endl;
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
    assert(builder != NULL);
    builder_ = builder;

    this->parseCapFile();

    std::cout << "///////////////////////////////" << std::endl;
    this->setEdgeCapacity();
    std::cout << "///////////////////////////////" << std::endl;
    this->parseNetFile();
    this->setNetList();
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