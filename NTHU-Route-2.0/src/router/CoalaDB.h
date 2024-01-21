#pragma once

#include "Construct_2d_tree.h"

#include "misc/geometry.h"

#include "gr_db/GrDatabase.h"

#include "db/Database.h"

#include <vector>

#include <utility>

#include <memory>

#include <unordered_map>

#include <algorithm>



namespace CoalaDB

{

    class CoalaPoint // this class implies relationship between points and segments

    {

    public:

        CoalaPoint() {}

        int maxLayer = 0;

        std::vector<std::shared_ptr<Jm::Segment_2d>> segments;

    };

    class CoalaDataBase

    {

    public:

        CoalaDataBase() {}

        std::vector<CoalaPoint> coalamap;

        std::vector<std::vector<std::shared_ptr<Jm::Segment_2d>>> netSegs;

        std::vector<int> netSegsNum;

        std::vector<std::vector<std::pair<int, const Pin *>>> layerPinList; // This data structure is a copycat

        std::vector<int> viaUsage;

        std::vector<std::unordered_map<int, int>> pointPlaced;

        std::vector<std::vector<int>> netPinBound;

        bool coordinate3DValid(const Jm::Coordinate_3d &p1,int factor);

        bool coordinate2DValid(const Jm::Coordinate_2d &p1);

        bool segmentValid(const Jm::Segment_2d &s1);

        void setDataBase();

        void getNetPath();

        void setNewSeg(std::shared_ptr<Jm::Segment_2d>);

        void initialLayerPinList(int netNumber);

        bool isInPointPlaced(const Jm::Coordinate_2d &p, int netIdx)

        {

            int pIndex = Coala2Dto1D(p.x, p.y);

            if (pointPlaced[netIdx].find(pIndex) == pointPlaced[netIdx].end())

                return false;

            return true;

        }

        bool isInPointPlaced(const Jm::Coordinate_3d &p, int netIdx)

        {

            int pIndex = Coala2Dto1D(p.x, p.y);

            if (pointPlaced[netIdx].find(pIndex) == pointPlaced[netIdx].end())

                return false;

            return true;

        }

        void addIntoPointPlaced(const Jm::Coordinate_2d &p, int netIdx)

        {

            int pIndex = Coala2Dto1D(p.x, p.y);

            if (pointPlaced[netIdx].find(pIndex) == pointPlaced[netIdx].end())

                pointPlaced[netIdx][pIndex] = 1;

            else

                pointPlaced[netIdx][pIndex]++;

        }

        int Coala2Dto1D(int x, int y)

        {

            return x + y * rr_map->get_gridx();

        }

        std::vector<int> Coala1Dto2D(int coord)

        {

            return std::vector<int>{coord % rr_map->get_gridx(), coord / rr_map->get_gridx()};

        }

        int Coala3Dto1D(int x, int y, int z)

        {

            return (z * rr_map->get_gridx() * rr_map->get_gridy()) + (y * rr_map->get_gridx()) + x;

        }

        int Coala3Dto1DC(int x, int y , int z, int factor){

            int nx = rr_map->get_gridx() / factor + 1;

            int ny = rr_map->get_gridy() / factor + 1;

            return (z * nx * ny) + (y * nx) + x;

        }

        std::vector<int> Coala1Dto3D(int coord)

        {

            int z = coord / (rr_map->get_gridx() * rr_map->get_gridy());

            coord -= (z * rr_map->get_gridx() * rr_map->get_gridy());

            int y = coord / rr_map->get_gridx();

            int x = coord % rr_map->get_gridx();

            return std::vector<int>{x, y, z};

        };

        std::vector<int> Coala1Dto3DC(int coord,int factor)

        {

            int gridx = rr_map->get_gridx() / factor + 1;

            int gridy = rr_map->get_gridy() / factor + 1;

            int z = coord / (gridx * gridy);

            coord -= (z * gridx * gridy);

            int y = coord / gridx;

            int x = coord % gridx;

            return std::vector<int>{x, y, z};

        };

    };

}