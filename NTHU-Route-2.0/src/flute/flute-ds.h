#ifndef INC_FLUTE_DS_H
#define INC_FLUTE_DS_H

#include "string"

extern std::string route_dict;

#define POWVFILE (route_dict + "/POWV9.dat").c_str()    // LUT for POWV (Wirelength Vector)
// #define POWVFILE "POWV9.dat"    // LUT for POWV (Wirelength Vector)
#define POSTFILE (route_dict + "/POST9.dat").c_str()    // LUT for POST (Steiner Tree)
// #define POSTFILE "POST9.dat"    // LUT for POST (Steiner Tree)

#ifndef DTYPE   // Data type for distance
#define DTYPE double
#endif

typedef struct
{
    DTYPE x, y;   // starting point of the branch
    int n;        // index of neighbor
} Branch;

typedef struct
{
    int deg;          // degree
    DTYPE length;     // total wirelength
    Branch *branch;   // array of tree branches

    int number;
} Tree;

#endif //INC_FLUTE_DS_H
