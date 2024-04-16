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

// /**
//  * @brief Global Router Parser for parsing ISPD'07 test case
//  */
// class Parser07 : public GRParser
// {
//   string fname_; ///< File name
//   string delims_;
//   FileHandler fh_;   ///< File Handler
//   Builder *builder_; ///< Data stucture builder
// public:
//   Parser07(const char *fname, FileHandler::FileType ftype);
//   virtual ~Parser07();
//   virtual void parse(Builder *builder);

// private:
//   /// Parse information of routing layers, tiles
//   void parseRoutingRegion();

//   /// Parse information of nets
//   void parseNets();

//   /// Parse information of one net
//   void parseANet();

//   /// Parse information of adjustmenting edge capacity
//   void adjustCapacity();
// };

// /**
//  * @brief Global Router Parser for parsing ISPD'98 test case
//  */
// class Parser98 : public GRParser
// {
//   string fname_; ///< File name
//   string delims_;
//   FileHandler fh_;   ///< File Handler
//   Builder *builder_; ///< Data stucture builder
// public:
//   Parser98(const char *fname, FileHandler::FileType ftype);
//   virtual ~Parser98();

//   virtual void parse(Builder *builder);

// private:
//   /// Parse the information of routing layers, tiles
//   void parseRoutingRegion();

//   /// Parse the information of nets
//   void parseNets();

//   /// Parse information of one net
//   void parseANet();
// };

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



//======= Inline Functions =======

// inline Parser07::Parser07(const char *fname, FileHandler::FileType ftype)
//     : fname_(fname),
//       delims_(" \t\n"),
//       fh_(fname, ftype)
// {
// }

// inline Parser98::Parser98(const char *fname, FileHandler::FileType ftype)
//     : fname_(fname),
//       delims_(" \t\n"),
//       fh_(fname, ftype)
// {
// }


class NetISPD24 {
    public:
        std::string name;
        int numPins;
        std::vector<std::vector<std::array<uint16_t, 3>>> accessPoints;

        NetISPD24(std::string n, int num, std::vector<std::vector<std::array<uint16_t, 3>>>& access) : name(n), numPins(num), accessPoints(access) {};
        NetISPD24() {};
    };

class Parser24 : public GRParser
{
  Builder *builder_;

  std::string cap_file;
  std::string net_file;

  std::vector<std::vector<std::vector<float>>> GcellCapacity;
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