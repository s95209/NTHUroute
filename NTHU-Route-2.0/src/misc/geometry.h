// File: misc/geometry.h
// Brief: Data containor for storing information of coordinate, location, directions.
// Author: Yen-Jung Chang
// $Date: 2007-11-26 12:48:59 +0800 (Mon, 26 Nov 2007) $
// $Revision: 12 $

#pragma once

#include <utility>
#include <cstdlib>
#include <cassert>
#include <iostream>
#include <memory>

namespace Jm
{

    /**
     * @brief Defined the flags about position, i.e. Direction, Corner
     */
    // class Position {
    // public:
    enum RerouteState
    {
        NO = 0,
        YES
    };
    enum DirectionType
    {
        DIR_NORTH = 0,
        DIR_SOUTH,
        DIR_EAST,
        DIR_WEST,
        DIR_UP,
        DIR_DOWN
    };
    enum CornerType
    {
        CORNER_LL = 0, // Left lower
        CORNER_LU,     // Left upper
        CORNER_RL,     // Right lower
        CORNER_RU      // Right upper
    };
    enum SegmentDir
    {
        None = 0,
        Horizontal,
        Vertical
    };
    enum SegmentStatus
    {
        Unplaced = 0,
        Placed
    };
    enum SegmentCompleteness
    {
        Fragmented = 0,
        Complete
    };
    enum CandidateSetStatus
    {
        New = 0,
        Added
    };
    enum SegmentValid
    {
        Valid = 0,
        Invalid
    };
    enum SegmentSeen
    {
        No = 0,
        Yes
    };
    enum SegAddStop
    {
        Go = 0,
        Stop
    };
    //};

    /**
     * @brief 3D Coordinate. If you only need to use 2D coordinate, just set z = 0.
     */
    class Coordinate
    {
    public:
        Coordinate(int x, int y, int z);
        ~Coordinate();
        void x(int i);
        void y(int i);
        void z(int i);
        int x() const;
        int y() const;
        int z() const;

        /// Check if two coordinate are the same
        bool equal(const Coordinate &co) const;

        ///@brief Check if current coordinate smaller than the other
        ///@details The size is the sum of x, y, and z. If the size of
        /// two coordinate are the same, then the function will compare
        /// by the follow order: x -> y -> z. so even if y and z are
        /// very huge, but x is smaller than the other, then the current
        /// coordinate is smaller.
        ///@return true if current coordinate is smaller
        bool smallerThan(const Coordinate &co) const;

        ///@brief Check if current coordinate bigger than the other
        ///@details The size is the sum of x, y, and z. If the size of
        /// two coordinate are the same, then the function will compare
        /// by the follow order: x -> y -> z. so even if y and z are
        /// very small, but x is bigger than the other, then the current
        /// coordinate is bigger.
        ///@return true if current coordinate is bigger
        bool biggerThan(const Coordinate &co) const;

        /// Check if two coordinate is within distance 1
        bool isNeighbor(const Coordinate &co) const;

    protected:
        uint16_t x_;
        uint16_t y_;
        uint8_t z_;
    };

    typedef std::pair<Coordinate, Coordinate> CoordinatePair;
    typedef std::pair<Coordinate, DirectionType> CoordinatePosition;

    ///@brief Callback function for sorting coordinate by their Manhattan
    /// distance in ascending order
    class ltCoordinate
    {
    public:
        bool operator()(const Coordinate *c1, const Coordinate *c2) const
        {
            return c1->smallerThan(*c2);
        }
    };

    //====== Coordinate ======
    inline Coordinate::Coordinate(int x = 0, int y = 0, int z = 0)
        : x_(x),
          y_(y),
          z_(z)
    {
    }

    inline Coordinate::~Coordinate()
    {
    }

    inline void Coordinate::x(int i)
    {
        x_ = i;
    }

    inline void Coordinate::y(int i)
    {
        y_ = i;
    }

    inline void Coordinate::z(int i)
    {
        z_ = i;
    }

    inline int Coordinate::x() const
    {
        return x_;
    }

    inline int Coordinate::y() const
    {
        return y_;
    }

    inline int Coordinate::z() const
    {
        return z_;
    }

    inline bool Coordinate::equal(const Coordinate &co) const
    {
        if (x_ != co.x())
            return false;
        if (y_ != co.y())
            return false;
        if (z_ != co.z())
            return false;
        return true;
    }

    inline bool Coordinate::isNeighbor(const Coordinate &co) const
    {
        if (abs((x_ + y_ + z_) - (co.x() + co.y() + co.z())) != 1)
            return false;
        return true;
    }

    inline bool Coordinate::smallerThan(const Coordinate &co) const
    {
        if ((x_ + y_ + z_) < (co.x() + co.y() + co.z()))
            return true;
        if ((x_ + y_ + z_) > (co.x() + co.y() + co.z()))
            return false;
        // below comparsion will only excute when (x_ + y_ + z_) == (co.x() + co.y() + co.z())
        if (x_ > co.x())
            return false;
        if (x_ < co.x())
            return true;
        if (y_ > co.y())
            return false;
        if (y_ < co.y())
            return true;
        if (z_ > co.z())
            return false;
        if (z_ < co.z())
            return true;
        return false;
    }

    inline bool Coordinate::biggerThan(const Coordinate &co) const
    {
        if ((x_ + y_ + z_) > (co.x() + co.y() + co.z()))
            return true;
        if ((x_ + y_ + z_) < (co.x() + co.y() + co.z()))
            return false;
        // below comparsion will only excute when (x_ + y_ + z_) == (co.x() + co.y() + co.z())
        if (z_ > co.z())
            return true;
        if (z_ < co.z())
            return false;
        if (y_ > co.y())
            return true;
        if (y_ < co.y())
            return false;
        if (x_ > co.x())
            return true;
        if (x_ < co.x())
            return false;
        return false;
    }

    class Coordinate_2d
    {
    public:
        uint16_t x;
        uint16_t y;
    public:
        Coordinate_2d(int x = 0, int y = 0)
            : x(x), y(y) {}
        Coordinate_2d(const Coordinate_2d &p)
        {
            x = p.x;
            y = p.y;
        }
        bool operator==(const Coordinate_2d &other) const
        {
            return (x == other.x && y == other.y);
        }
        bool operator!=(const Coordinate_2d &other) const
        {
            return (x != other.x || y != other.y);
        }
        int L1Distance(const Coordinate_2d &rhs) const
        {
            return abs(x - rhs.x) + abs(y - rhs.y);
        }
        bool isNeighbor(const Coordinate_2d &rhs) const
        {
            if (this->L1Distance(rhs) <= 1)
                return true;
            return false;
        }
        SegmentDir getDir(const Coordinate_2d &rhs) const
        {
            assert(this->isNeighbor(rhs));
            if (x == rhs.x)
                return Vertical;
            else
                return Horizontal;
        }
    };

    class Coordinate_3d
    {
    public:
        uint16_t x;
        uint16_t y;
        uint8_t z;

    public:
        Coordinate_3d(int x = 0, int y = 0, int z = 0)
            : x(x), y(y), z(z) {}
        bool operator==(const Coordinate_3d &other) const
        {
            return (x == other.x && y == other.y && z == other.z);
        }
        bool operator!=(const Coordinate_3d &other) const
        {
            return (x != other.x || y != other.y || z != other.z);
        }
        Coordinate_3d operator/(const int rhs) const{
            return Coordinate_3d(this->x/rhs,this->y/rhs,this->z);
        }
        int l1Distance(const Coordinate_3d &rhs)
        {
            return abs(rhs.x - this->x) + abs(rhs.y - this->y) + abs(rhs.z - this->z);
        }
        void printpoint()
        {
            std::cout << x << "," << y << "," << z << "\n";
        }
    };

    class Segment_2d
    {
    public:
        int netIdx;
        int layer = -1;
        double completenessRate = 0.0;
        Coordinate_2d p1;
        Coordinate_2d p2;
        SegmentDir dir;
        RerouteState rstat = NO;
        CandidateSetStatus cstat = New;
        SegmentCompleteness completeness = Complete;
        std::shared_ptr<Segment_2d> brotherSeg;
        SegmentStatus stat = Unplaced;
        SegmentValid vstat = Valid;
        SegmentSeen seen = SegmentSeen::No;
        SegAddStop stop = SegAddStop::Go;
        bool merged = false;
        int demandTileNum = 0;
        int availableTileNum = 0;
        Segment_2d(int netid, Coordinate_2d p1_, Coordinate_2d p2_, SegmentDir d) : netIdx(netid), p1(p1_), p2(p2_), dir(d) {}

        int length(){
            return p1.L1Distance(p2);
        }
        void printseg()
        {
            std::cout << this->p1.x << " " << this->p1.y << " "
                      << "1 " << this->p2.x << " " << this->p2.y << " 1 \n";
            /* << "net : " << this->netIdx << " status : " << this->stat << " layer : " << this->layer << " cstat : " << this->cstat << "is complete" << completeness << " \n";
   std::cout<<" need reroute ? "<<this->rstat<<"\n";*/
            return;
        }
        bool pointIsNeighbor(const Coordinate_2d &rhs)
        {
            if (p1.isNeighbor(rhs) || p2.isNeighbor(rhs))
                return true;
            return false;
        }
        bool pointIsSameDir(const Coordinate_2d &s, const Coordinate_2d &rhs) const
        {
            if (p1 == s && p1.getDir(rhs) == dir)
                return true;
            if (p2 == s && p2.getDir(rhs) == dir)
                return true;
            if (p1 == rhs && p1.getDir(s) == dir)
            return true;
            if (p2 == rhs && p1.getDir(s) == dir)
            return true;
            return false;
        }
        bool addPoint(Coordinate_2d s, Coordinate_2d p)
        {
            if (s != p1 && s != p2)
                return false;
            if (dir == None)
            {
                dir = this->p1.getDir(p);
                p2.x = p.x;
                p2.y = p.y;
                return true;
            }
            else
            {
                if (pointIsSameDir(s, p))
                {
                    if (p1 == s)
                    {
                        p1.x = p.x;
                        p1.y = p.y;
                    }
                    else
                    {
                        p2.x = p.x;
                        p2.y = p.y;
                    }
                    return true;
                }
                return false;
            }
        }
        void sortPoint()
        {
            if (this->p1.x == this->p2.x)
            {
                if (this->p1.y > this->p2.y)
                {
                    std::swap(this->p1, this->p2);
                }
            }
            else if (this->p1.y == this->p2.y)
            {
                if (this->p1.x > this->p2.x)
                {
                    std::swap(this->p1, this->p2);
                }
            }
        }
        bool isIntersect(const Segment_2d &rhs)
        {
            assert(this->dir != rhs.dir);
            if (rhs.dir == Vertical)
            {
                if (rhs.p1.x < this->p2.x && rhs.p1.x > this->p1.x)
                {
                    if (this->p1.y >= rhs.p1.y && this->p1.y <= rhs.p2.y)
                    {
                        return true;
                    }
                }
            }
            else
            {
                if (rhs.p1.y < this->p2.y && rhs.p1.y > this->p1.y)
                {
                    if (this->p1.x >= rhs.p1.x && this->p1.x <= rhs.p2.x)
                    {
                        return true;
                    }
                }
            }
            return false;
        }
        Coordinate_2d IntersectPoint(const Segment_2d &rhs)
        {
            assert(this->dir != rhs.dir);
            if (rhs.dir == Vertical)
            {
                return Coordinate_2d(rhs.p1.x, this->p1.y);
            }
            else
            {
                return Coordinate_2d(this->p1.x, rhs.p1.y);
            }
            return false;
        }
        void addSeg(const Segment_2d &rhs)
        {
            assert(this->dir == rhs.dir);
            if (rhs.dir == Vertical)
            {
                if (rhs.p1.y < this->p1.y)
                {
                    this->p1.y = rhs.p1.y;
                }
                if (rhs.p2.y > this->p2.y)
                {
                    this->p2.y = rhs.p2.y;
                }
            }
            else
            {
                if (rhs.p1.x < this->p1.x)
                {
                    this->p1.x = rhs.p1.x;
                }
                if (rhs.p2.x > this->p2.x)
                {
                    this->p2.x = rhs.p2.x;
                }
            }
        }
    };
    class CompareSegX
    {
    public:
        bool operator()(std::shared_ptr<Jm::Segment_2d> &a, std::shared_ptr<Jm::Segment_2d> &b)
        {
            return (a->p1.x) < (b->p1.x);
        }
    };
    class CompareSegY
    {
    public:
        bool operator()(std::shared_ptr<Jm::Segment_2d> &a, std::shared_ptr<Jm::Segment_2d> &b)
        {
            return (a->p1.y) < (b->p1.y);
        }
    };
} // namespace Jalamorm
