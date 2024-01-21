#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>
#include <climits>
#include <algorithm>
#include "GrDbTrans.h"
#include "global.h"

void GrDbTransformer::transOutput() const
{
    std::ifstream input_file(this->resFile, std::ifstream::in);
    std::string read_line;

    std::ofstream ofs;
    ofs.open(this->guideFile);

    auto pointToGcell = [&](int x, int y, int xStep, int yStep)
    {
        std::vector<utils::PointT<int>> coord(2);
        coord[0].x = x * xStep;
        coord[0].y = y * yStep;
        coord[1].x = coord[0].x + xStep;
        coord[1].y = coord[0].y + yStep;
        return coord;
    };

    auto rectToGuide = [&](std::string x1, std::string y1, std::string x2, std::string y2, std::string layer)
    {
        std::string str;
        auto coord1 = pointToGcell(std::stoi(x1), std::stoi(y1), cugr_db.getXStep(), cugr_db.getYStep());
        auto coord2 = pointToGcell(std::stoi(x2), std::stoi(y2), cugr_db.getXStep(), cugr_db.getYStep());
        int xmax = INT_MIN, xmin = INT_MAX;
        int ymax = INT_MIN, ymin = INT_MAX;
        coord1.insert(coord1.end(), coord2.begin(), coord2.end());
        for (auto &point : coord1)
        {
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
        str = str + "Metal" + layer + "\n";
        return str;
    };
    auto viaToGuide = [&](std::string x, std::string y, std::string layer1, std::string layer2)
    {
        std::string str = "";
        int layer_1 = std::min(std::stoi(layer1), std::stoi(layer2));
        int layer_2 = std::max(std::stoi(layer1), std::stoi(layer2));
        auto coord1 = pointToGcell(std::stoi(x), std::stoi(y), cugr_db.getXStep(), cugr_db.getYStep());
        int xmax = std::min(int(database.dieRegion.x.high), coord1[1].x);
        int ymax = std::min(int(database.dieRegion.y.high), coord1[1].y);
        for (int i = layer_1; i < layer_2 + 1; i++)
        {
            str = str + std::to_string(coord1[0].x) + " " + std::to_string(coord1[0].y) + " " + std::to_string(coord1[1].x) +
                  " " + std::to_string(coord1[1].y) + " Metal" + std::to_string(i) + "\n";
        }
        return str;
    };

    while (getline(input_file, read_line))
    {
        bool hasPinVia = false;
        std::vector<std::string> splited_string;
        std::string netName = "";
        boost::split(splited_string, read_line, boost::is_any_of(" "), boost::token_compress_on);
        if (splited_string.size() == 1)
        {
            ofs << splited_string[0] << "\n";
        }
        else if (splited_string.size() > 1)
        {
            if (splited_string[2] == splited_string[5])
            {
                ofs << rectToGuide(
                    splited_string[0], splited_string[1], splited_string[3], splited_string[4], splited_string[2]);
            }
            else
            {
                ofs << viaToGuide(splited_string[0], splited_string[1], splited_string[2], splited_string[5]);
            }
        }
    }
}
