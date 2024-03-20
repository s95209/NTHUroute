// File: grdb/parser.h
// Brief: Parser for parsing test case of ISPD'07 and ISPD'98
// Author: Yen-Jung Chang
// $Date: 2007-11-26 12:48:59 +0800 (Mon, 26 Nov 2007) $
// $Revision: 12 $

#pragma once

#include "builder.h"
#include "misc/filehandler.h"
#include "gr_db/GrDatabase.h"
#include "db/Database.h"

using Jm::FileHandler;

/**
 * @brief Glboal Router Parser insterface
 */
class GRParser
{
public:
  virtual ~GRParser();

  /// @brief Parse file with Builder
  /// @param[in] builder Callback functions for parser
  virtual void parse(Builder *builder) = 0;
};



// class Parser18 : public GRParser
// {
//   Builder *builder_;
//   std::shared_ptr<std::vector<std::vector<int>>> layerOneCap;

// public:
//   Parser18(std::shared_ptr<std::vector<std::vector<int>>> &cap) : layerOneCap(cap)
//   {
//   }
//   virtual ~Parser18();
//   virtual void parse(Builder *builder);

//   // void setLayerOneCap(int x, int y) { this->layerOneCap.resize(x, vector<int>(y, 0)); }
//   //  std::vector<std::vector<int>> getLayerOneCap() { return layerOneCap; };

// private:
//   void parseRoutingRegion();

//   /// Parse information of nets
//   void parseNets();

//   /// Parse information of one net
//   void parseANet(int index);

//   /// Parse information of adjustmenting edge capacity
//   void setCapacity();


//   void parseCmap();
//   void parseNetList();
// };

class NetISPD24 {
    public:
        std::string name;
        int numPins;
        std::vector<std::vector<std::vector<int>>> accessPoints;

        NetISPD24(std::string n, int num, std::vector<std::vector<std::vector<int>>>& access) : name(n), numPins(num), accessPoints(access) {};
        NetISPD24() {};
    };

class Parser24 : public GRParser
{
  Builder *builder_;

  std::string cap_file;
  std::string net_file;

  std::vector<std::vector<std::vector<double>>> GcellCapacity;
  std::unordered_map<std::string, NetISPD24> nets;

  public:
  Parser24(const string& _cap_file, const string& _net_file): cap_file{_cap_file}, net_file{_net_file} { }
  void parseCapFile();
  void parseNetFile();
  void setEdgeCapacity();
  void setNetList(); 
  virtual ~Parser24();
  virtual void parse(Builder *builder);
};