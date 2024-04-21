#include <iostream>
#include <cassert>
#include "RoutingRegion.h"
#include "misc/geometry.h"

using namespace std;
using namespace Jm;

/***************
  RoutingRegion
  **************/
RoutingRegion::RoutingRegion()
:netList_(NULL),
 routingSpace_(new RoutingSpace),
 pinTable_(NULL)
{}

RoutingRegion::~RoutingRegion()
{
    if (netList_ != NULL) delete netList_;
    if (routingSpace_ != NULL) delete routingSpace_;
    if (pinTable_ != NULL) delete pinTable_;
}

// Edges' capacity information
//get edge's max capacity
int RoutingRegion::capacity(int layer_id, int x1, int y1, int x2, int y2){
	int x = min(x1, x2);
	int y = min(y1, y2);

	//get vertical capacity
	if( (x1 == x2) && (y1 != y2) ){
        return routingSpace_->edge(x, y, layer_id, DIR_NORTH).capacity;
	}

	//get horizontal capacity
	if( (x1 != x2) && (y1 == y2) ){
        return routingSpace_->edge(x, y, layer_id, DIR_EAST).capacity; 
	}

	return 0;
}


inline
void RoutingRegion::setGrid (unsigned int x,
                             unsigned int y,
                             unsigned int layerNumber)
{
    routingSpace_->resize(x, y, layerNumber);
}

void RoutingRegion::setVerticalCapacity (unsigned int layerId,
                                         unsigned int capacity)
{
    for(int x = 0; x < get_gridx(); ++x){
		//the upmost tiles don't have north neighbor
		//so only set to the (grid_y - 1)th row
        int y = 0;
        for(; y < (get_gridy() - 1); ++y){
            routingSpace_->edge(x, y, layerId, DIR_NORTH).capacity = capacity;
		}
        routingSpace_->edge(x, y, layerId, DIR_NORTH).capacity = 0;
	}
}

void RoutingRegion::setHorizontalCapacity (unsigned int layerId,
                                           unsigned int capacity)
{
	//the rightmost tiles don't have east neighbor
	//so only set to the (grid_x - 1)th column
	for(int y = 0; y < get_gridy(); ++y){
        int x = 0;
		for(; x < (get_gridx()-1); ++x){
            routingSpace_->edge(x, y, layerId, DIR_EAST).capacity = capacity;
		}
        routingSpace_->edge(x, y, layerId, DIR_EAST).capacity = 0;
	}
}

void RoutingRegion::setNetNumber (unsigned int netNumber)
{
    if(netList_ == NULL) {
        netList_ = new NetList();
        netList_->reserve(netNumber);
    }
}

void RoutingRegion::adjustEdgeCapacity (unsigned int x1,
                                        unsigned int y1,
                                        unsigned int z1,
                                        unsigned int x2,
                                        unsigned int y2,
                                        unsigned int,//z2
                                        unsigned int capacity)
{
	int x = min(x1, x2);
	int y = min(y1, y2);

	//get vertical capacity
	if( (x1 == x2) && (y1 != y2) ){
        routingSpace_->edge(x, y, z1, DIR_NORTH).capacity = capacity;
	}

	//get horizontal capacity
	if( (x1 != x2) && (y1 == y2) ){
        // std::cout << "x: " << x << " y: " << y << std::endl;
        routingSpace_->edge(x, y, z1, DIR_EAST).capacity = capacity;
	}
}

void RoutingRegion::setTileTransformInformation (unsigned int llx,
                                                 unsigned int lly, 
                                                 unsigned int tWidth, 
                                                 unsigned int tHeight)
{
    routingSpace_->originX = llx;
    routingSpace_->originY = lly;
    routingSpace_->tileWidth = tWidth;
    routingSpace_->tileHeight = tHeight;
}

void RoutingRegion::beginAddANet(const char* netName, unsigned int, unsigned int minWidth)
{
    int netId = netList_->size();
	netList_->push_back( Net(netName, netId) );
}

void RoutingRegion::addPin(unsigned int x, unsigned int y, unsigned int layer)
{
    if (pinTable_ == NULL) pinTable_ = new PinTable();
	//transfer pin's coordinate to tile position
	int tileX = ((x - get_llx()) / get_tileWidth());
	int tileY = ((y - get_lly()) / get_tileHeight());
    // cout << "llx() " << get_llx() << ' ' << get_tileWidth() << '\n'; // 0 1
    // cout << "lly() " << get_lly() << ' ' << get_tileHeight() << '\n'; // 0 1
    assert(tileX == x);
    assert(tileY == y);

    if (!pinTable_->count({tileX, tileY, layer})) {
        pinTable_->insert({tileX, tileY, layer});
        netList_->back().add_pin(&(routingSpace_->tile(tileX, tileY, layer).getAnchor()));
    } else {
        // cout << tileX << ' ' << tileY << ' ' << layer << " multiple pins in the same gcell\n";
    }
}

void RoutingRegion::endAddANet ()
{
    if (netList_->back().get_pinNumber() <= 1) {
        deletedNetList_->emplace_back(netList_->back());
        netList_->pop_back();
    }
    // justCheck.clear();
    //release memory resource for pin lookup table
    if (pinTable_ != NULL) {
        delete pinTable_;
        pinTable_ = NULL;
    }
}

void RoutingRegion::endBuild ()
{
    cout<<"\033[33mTotal nets to route="<<netList_->size()<<"\033[m"<<endl;
}
