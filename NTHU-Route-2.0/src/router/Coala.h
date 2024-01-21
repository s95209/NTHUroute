#pragma once

#include "CoalaDB.h"

#include <queue>

#include <algorithm>

#include <unordered_map>



namespace Coala

{

    class CompareCandidateSeg

    {

    public:

        bool operator()(std::shared_ptr<Jm::Segment_2d> &a, std::shared_ptr<Jm::Segment_2d> &b)

        {

            if (a->completeness != b->completeness)

                return a->completeness > b->completeness;

            if (a->completenessRate != b->completenessRate)

                return a->completenessRate < b->completenessRate;

            if (a->p1.L1Distance(a->p2) != b->p1.L1Distance(b->p2))

                return a->p1.L1Distance(a->p2) > b->p1.L1Distance(b->p2);

        }

    };

    class CompareRemainSeg

    {

    public:

        bool operator()(std::shared_ptr<Jm::Segment_2d> &a, std::shared_ptr<Jm::Segment_2d> &b)

        {

            return (a->demandTileNum * 20 + a->availableTileNum * 1) < (b->demandTileNum * 20 + b->availableTileNum * 1);

        }

    };

    class Layerassignment

    {

    public:

        Layerassignment(std::shared_ptr<std::vector<std::vector<int>>> &cap) : layerOneCap(cap) {}

        CoalaDB::CoalaDataBase coalaDB;

        std::shared_ptr<std::vector<std::vector<int>>> layerOneCap;

        std::unordered_map<int, std::vector<int>> endPointMap;

        std::priority_queue<std::shared_ptr<Jm::Segment_2d>, std::vector<std::shared_ptr<Jm::Segment_2d>>, CompareCandidateSeg> candidateSegSet;

        std::priority_queue<std::shared_ptr<Jm::Segment_2d>, std::vector<std::shared_ptr<Jm::Segment_2d>>, CompareRemainSeg> remainSegSet;

        std::vector<std::shared_ptr<Jm::Segment_2d>> nextlayerSegSet;

        std::vector<std::shared_ptr<Jm::Segment_2d>> invalidSegSet;

        std::vector<std::vector<std::shared_ptr<Jm::Segment_2d>>> finalPath;

        std::vector<std::unordered_map<int, std::vector<bool>>> viasMap;

        std::vector<std::pair<Jm::Coordinate_2d, std::pair<int, int>>> pointToPin; // (point) (to pin distance , netIdx)

        void run();

        void pinInitialize(int);

        void viaAssignment(int);

        void completeSegAssignment(int);

        void remainSegAssignment(int);

        void addToEndPointMap(Jm::Coordinate_2d &p, int netIdx, int layer);

        void deleteFromEndPointMap(Jm::Coordinate_2d p, int netIdx);

        bool isInEndPointMap(Jm::Coordinate_2d &p, int netIdx);

        void updateDemandPredMap(int);

        void generateOutput();

        bool deleteRedundantEP(Jm::Coordinate_2d &p, int netIdx);

        void addRelativeSeg(Jm::Coordinate_2d &p, int netIdx);

        bool isInViaMap(int index, int);

        void generateTileCongestionMap(string outputFileName);

        void generateEdgeCongestionMap(string outputFileName);

        void readTileCongestionMap(string inputFileName);

        void addPinPatch();

        void setInvalidVia(int x, int y);

        void addUnseenSeg();

    };

}