#pragma once
#include "GrDatabase.h"
#include <string>
class GrDbTransformer {
public:
    gr::GrDatabase cugr_db;  //這邊會複製 不好
    const std::string resFile;
    const std::string guideFile;
    GrDbTransformer(gr::GrDatabase &gr_db, const std::string &in_file, const std::string &out_file)
        : cugr_db(gr_db), resFile(in_file), guideFile(out_file) {}
    void transOutput() const;
};
