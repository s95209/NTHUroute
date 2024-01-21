#pragma once

#include "Coala.h"

#include "CoalaDB.h"

#include <iostream>

#include <queue>



namespace Coala

{

    class TwoPinNet

    {

    public:

        Jm::Coordinate_3d p1;

        Jm::Coordinate_3d p2;

        int remainCapacity;

        int wirelength;

        int netIdx;

        int routeIter = 0;

        TwoPinNet(Jm::Coordinate_3d &p1_, Jm::Coordinate_3d &p2_, int rc, int netId) : p1(p1_), p2(p2_), remainCapacity(rc), netIdx(netId)

        {

            wirelength = p1.l1Distance(p2);

        }

    };

    class CompareTwoPinNet

    {

    public:

        bool operator()(const TwoPinNet &n1, const TwoPinNet &n2)

        {

            //return (n1.remainCapacity * 0.5 + n1.wirelength * 0.5) > (n2.remainCapacity * 0.5 + n2.wirelength * 0.5);

            return (n1.wirelength * 0.5) > (n2.wirelength * 0.5);

        }

    };

    class AstarNode

    {

    public:

        Jm::Coordinate_3d p;

        double cost;

        double h_cost = 0.0;

        bool valid = true;

        std::shared_ptr<AstarNode> parent;

        int wirelength = 0;

        int dir = -1;

        AstarNode(Jm::Coordinate_3d p_, double c) : p(p_), cost(c){};

    };

    class CoarseningNode{

        public:

        double cap = 0.0;

        double max_cap = 0.0;

        CoarseningNode(){};

    };

    class CompareAstarNode

    {

    public:

        bool operator()(std::shared_ptr<AstarNode> n1, std::shared_ptr<AstarNode> n2)

        {

            return n1->cost > n2->cost;

        }

    };

    class CoalaReroute

    {

    public:

        CoalaDB::CoalaDataBase coalaDB;

        std::vector<std::shared_ptr<Jm::Segment_2d>> nextlayerSegSet;

        std::vector<std::vector<Jm::Coordinate_3d>> reroutePoints;

        std::priority_queue<TwoPinNet, std::vector<TwoPinNet>, CompareTwoPinNet> twoPinNets;

        std::vector<std::vector<std::vector<CoarseningNode>>> coarseningGraph;

        std::vector<std::unordered_map<int,bool>> coarseningGuide;

        CoalaReroute(const Coala::Layerassignment &ls) : coalaDB(ls.coalaDB), nextlayerSegSet(ls.nextlayerSegSet)

        {

            reroutePoints.resize(rr_map->get_netNumber());

        }

        void setCoarseningGraph();

        void aStarCoarsening(std::vector<std::vector<std::shared_ptr<Jm::Segment_2d>>> &fp);

        void capacityRecovery(std::shared_ptr<Jm::Segment_2d> target);

        void deleteSegFromFinalPath(std::shared_ptr<Jm::Segment_2d> target, std::vector<std::vector<std::shared_ptr<Jm::Segment_2d>>> &fp);

        void viaRecovery(Jm::Coordinate_2d &p, int netIdx, std::vector<std::unordered_map<int,std::vector<bool>>> &viasMap, int layer , bool t);

        void removeRedundantVia();

        void removeRedundantPS(std::vector<std::vector<std::shared_ptr<Jm::Segment_2d>>> &fp, std::vector<std::unordered_map<int,std::vector<bool>>> &viasMap, std::vector<std::pair<Jm::Coordinate_2d, std::pair<int, int>>> &pointToPin);

        std::vector<std::vector<int>> initAdjancentMatrix(int netIdx, int netSize);

        void pinDecomposition(int);

        int calculate3DCon(const Jm::Coordinate_3d &p1, const Jm::Coordinate_3d &p2);

        void aStar3D(std::vector<std::vector<std::shared_ptr<Jm::Segment_2d>>> &fp, std::vector<std::unordered_map<int, std::vector<bool>>> &viasMap);

        void addIntoReroutePoints(const Jm::Coordinate_3d &p1, int netIdx);

    };

}