#ifndef INC_ROUTINGCOMPONENT_H
#define INC_ROUTINGCOMPONENT_H

#include "plane.h"
#include "misc/geometry.h"
#include <iostream>
#include <utility>
#include <string>
#include <vector>

class Pin : public Jm::Coordinate {
	public:
		//Constructor
		Pin(int x, int y, int z);
		int get_tileX() const;
		int get_tileY() const;
		int get_layerId() const;		//get the layer id that contain this pin
};

typedef std::vector<const Pin*> PinptrList;

class Net{
    public:
		int id;             //the net ordered in test case
	public:
		//Constructor
		Net(const char* name ="", int id = 0);

		void set_name(const char*);	//set net name
		void add_pin(const Pin* pin_ptr); //add pin pointer that point to pin
        const PinptrList& get_pinList();//get the pin pointer list of this net
		const char* get_name() const;	//get net name
        std::string &getName();
		int get_pinNumber() const;		//get pin number in this net
        int get_bboxSize() const ;      //get the bounding box size
        int get_boundaryN() const;
        int get_boundaryE() const;
        int get_boundaryW() const;
        int get_boundaryS() const;

	private:
        std::string name;
        uint16_t minPinX;
        uint16_t maxPinX;
        uint16_t minPinY;
        uint16_t maxPinY;
        PinptrList pin_list;
};

class RoutingSpace {
    private:
        class RoutingEdge {
            public:
                RoutingEdge (float capacity = 0): capacity(capacity) {}
                float capacity;
        };

        class Tile {
            public:
                Tile(int x = 0, int y = 0, int z = 0): coordinate(x, y, z) {}
                const Pin& getAnchor();
                Pin coordinate;         //Coordinate of the tile
        };

    public:
        int tileWidth;
        int tileHeight;
        int originX;
        int originY;
        std::vector<int> wireWidth;		//minimum wire width
        std::vector<int> wireSpacing;	//minimum wire spacing
        std::vector<int> viaSpacing;	//minimum via spacing

    public:
                        RoutingSpace (int x = 0, int y = 0, int z = 0);
                        ~RoutingSpace ();

    void                resize (int x, int y, int z);

    ///Get the size of routing spacing in x-axis
    int                 getXSize () const;

    ///Get the size of routing spacing in y-axis
    int                 getYSize () const;

    ///Get the size of routing spacing in z-axis
    int                 getZSize () const;

    ///@brief Get the specified tile
    Tile&               tile(int x, int y, int z);

    ///@brief Get the specified edge
    RoutingEdge&        edge(int x, int y, int z, Jm::DirectionType);

	private:
        std::vector< Plane<Tile, RoutingEdge> >* routingSpace_;

    private:
        void  assignTileCoordinate ();
};



typedef std::vector<Net> NetList;

//Inline Functions
/*************
 * RoutingSpace
 *************/
inline
void RoutingSpace::resize(int x, int y, int z)
{
    //routingSpace_.resize(x, y , z);
    routingSpace_->resize(z, Plane<Tile, RoutingEdge>(x, y, Tile(), RoutingEdge()));
    wireWidth.resize(z);
    wireSpacing.resize(z);
    viaSpacing.resize(z);
    assignTileCoordinate();
}

inline
RoutingSpace::Tile& RoutingSpace::tile(int x, int y, int z)
{
    return (*routingSpace_)[z].vertex(x, y);
}

inline
RoutingSpace::RoutingEdge& RoutingSpace::edge(int x, int y, int z, Jm::DirectionType dir)
{
    return (*routingSpace_)[z].edge(x, y, dir);
}

inline
int RoutingSpace::getXSize() const 
{
    assert(routingSpace_->size() > 0);
    return (*routingSpace_)[0].getXSize();
}

inline
int RoutingSpace::getYSize() const
{
    assert(routingSpace_->size() > 0);
    return (*routingSpace_)[0].getYSize();
}

inline
int RoutingSpace::getZSize() const
{
    //return routingSpace_.getZSize();
    return routingSpace_->size();
}

inline
const Pin& RoutingSpace::Tile::getAnchor()
{
    return coordinate;
}
#endif /*INC_ROUTINGCOMPONENT_H*/
