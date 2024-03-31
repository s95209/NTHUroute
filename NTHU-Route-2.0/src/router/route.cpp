#include <cstdio>

#include <cstdlib>

#include <ctime>

#include <cassert>

#include <chrono>

#include <vector>

#include <filesystem>  // Include the <filesystem> header

#include <omp.h>

#include "parameter.h"

#include "Construct_2d_tree.h"

#include "grdb/RoutingRegion.h"

#include "grdb/parser.h"

#include "misc/filehandler.h"

#include "util/verifier.h"

#include "db/Database.h"

#include "global.h"

#include "gr_db/GrDatabase.h"

#include "gr_db/GrDbTrans.h"

double LARGE_NUM = 100000000;

string route_dict;

void dataPreparetion(ParameterAnalyzer &ap, Builder *builder, std::shared_ptr<std::vector<std::vector<int>>> &layerOneCap);

void dataPreparetionISPD2024(ParameterAnalyzer &ap, Builder *builder);

extern void construct_2d_tree(RoutingRegion *);

extern void Post_processing();

extern void Layer_assignment(const std::string&);

extern double GAMER_TIME;
extern double PATH_SEARCH_TIME;
extern double MAZE_TIME;
extern double DP_TIME;

#define MAX_THREAD_NUM 16

std::string TESTCASE_NAME;

std::chrono::high_resolution_clock::time_point prog_start;
std::chrono::high_resolution_clock::time_point prog_end;
std::chrono::high_resolution_clock::time_point pattern_start;
std::chrono::high_resolution_clock::time_point pattern_end;
std::chrono::high_resolution_clock::time_point main_start;
std::chrono::high_resolution_clock::time_point main_end;
std::chrono::high_resolution_clock::time_point post_start;
std::chrono::high_resolution_clock::time_point post_end;
std::chrono::high_resolution_clock::time_point la_start;
std::chrono::high_resolution_clock::time_point la_end;
std::chrono::high_resolution_clock::time_point io_start;
std::chrono::high_resolution_clock::time_point io_end;


void transOutput(const std::string &in_file, const std::string &out_file)
{
    std::ifstream input_file(in_file, std::ifstream::in);
    std::string read_line;

    std::ofstream ofs;
    ofs.open(out_file);

    auto pointToGcell = [&](int x, int y, int xStep, int yStep)
    {
        std::vector<utils::PointT<int>> coord(2);
        coord[0].x = max(0, x * xStep - 1);
        coord[0].y = max(0, y * yStep - 1);
        coord[1].x = max(0, (x+1) * xStep - 1);
        coord[1].y = max(0, (y+1) * yStep - 1);
        return coord;
    };

    auto rectToGuide = [&](std::string x1, std::string y1, std::string x2, std::string y2, std::string layer)
    {
        std::string str;
        auto coord1 = pointToGcell(std::stoi(x1), std::stoi(y1), grDatabase.getXStep(), grDatabase.getYStep());
        auto coord2 = pointToGcell(std::stoi(x2), std::stoi(y2), grDatabase.getXStep(), grDatabase.getYStep());
        int xmax = INT_MIN, xmin = INT_MAX;
        int ymax = INT_MIN, ymin = INT_MAX;
        coord1.insert(coord1.end(), coord2.begin(), coord2.end());
        for (auto &point : coord1)
        {
            if (point.x > xmax)
                xmax = point.x;
            if (point.x < xmin)
                xmin = point.x;
            if (point.y > ymax)
                ymax = point.y;
            if (point.y < ymin)
                ymin = point.y;
        }

        str = str + std::to_string(xmin) + " ";
        str = str + std::to_string(ymin) + " ";
        if (xmax > database.dieRegion.x.high)
            str = str + std::to_string(database.dieRegion.x.high) + " ";
        else
            str = str + std::to_string(xmax) + " ";
        if (ymax > database.dieRegion.y.high)
            str = str + std::to_string(database.dieRegion.y.high) + " ";
        else
            str = str + std::to_string(ymax) + " ";
        str = str + "Metal" + layer + "\n";
        return str;
    };
    auto viaToGuide = [&](std::string x, std::string y, std::string layer1, std::string layer2)
    {
        std::string str = "";
        int layer_1 = std::min(std::stoi(layer1), std::stoi(layer2));
        int layer_2 = std::max(std::stoi(layer1), std::stoi(layer2));
        auto coord1 = pointToGcell(std::stoi(x), std::stoi(y), grDatabase.getXStep(), grDatabase.getYStep());
        int xmax = std::min(int(database.dieRegion.x.high), coord1[1].x);
        int ymax = std::min(int(database.dieRegion.y.high), coord1[1].y);
        for (int i = layer_1; i < layer_2 + 1; i++)
        {
            str = str + std::to_string(coord1[0].x) + " " + std::to_string(coord1[0].y) + " " + std::to_string(coord1[1].x) +
                  " " + std::to_string(coord1[1].y) + " Metal" + std::to_string(i) + "\n";
        }
        return str;
    };

    while (getline(input_file, read_line))
    {
        bool hasPinVia = false;
        std::vector<std::string> splited_string;
        std::string netName = "";
        boost::split(splited_string, read_line, boost::is_any_of(" "), boost::token_compress_on);
        if (splited_string.size() == 1)
        {
            ofs << splited_string[0] << "\n";
        }
        else if (splited_string.size() > 1)
        {
            if (splited_string[2] == splited_string[5])
            {
                ofs << rectToGuide(
                    splited_string[0], splited_string[1], splited_string[3], splited_string[4], splited_string[2]);
            }
            else
            {
                ofs << viaToGuide(splited_string[0], splited_string[1], splited_string[2], splited_string[5]);
            }
        }
    }
}

int main(int argc, char *argv[])
{

    cout << "=======================================================" << endl

         << "= NTHU-Route                                          =" << endl

         << "= Version 2.11 is deveploped by                       =" << endl

         << "= Yen-Jung Chang, Yu-ting Lee, Tsung-Hsien Lee        =" << endl

         << "= Jhih-Rong Gao, Pei-Ci Wu, Chao-Yuan Huang           =" << endl

         << "= Adviser: Ting-Chi Wang (tcwang@cs.nthu.edu.tw)      =" << endl

         << "= http://www.cs.nthu.edu.tw/~tcwang/nthuroute/        =" << endl

         << "=======================================================" << endl

         << endl

         << "=======================================================" << endl

         << "= Running FLUTE for initial steiner tree              =" << endl

         << "= FLUTE is developed by Dr. Chris C. N. Chu           =" << endl

         << "=                       Iowa State University         =" << endl

         << "= http://home.eng.iastate.edu/~cnchu/                 =" << endl

         << "=======================================================" << endl

         << "=======================================================" << endl

         << "= Jinkela Shengdiyage                                 =" << endl

         << "= Fix undefined behavior and use c++11                =" << endl

         << "=                       Chao-Yuan Huang               =" << endl

         << "= https://github.com/jacky860226/nthu-route           =" << endl

         << "=======================================================" << endl;

    std::chrono::steady_clock::time_point startRoute = std::chrono::steady_clock::now();

    clock_t t0 = clock();

    // to get the absolute path of the binary route for flute operation
    char* routeAbsolutePath = realpath(argv[0], nullptr);
    std::filesystem::path routePath(routeAbsolutePath);
    
    //in my case route_dict = /users/student/mr111/lywu22/ISPD2024/ISPD2024-NTHU_Route_2.0/NTHU-Route-2.0
    route_dict = routePath.parent_path().string(); 

    // to set the maximun thread constraints for the openmp
    omp_set_num_threads(MAX_THREAD_NUM);
    std::cout << "This is " << MAX_THREAD_NUM << " threads version!" << endl;

    ParameterAnalyzer ap(argc, argv);

    cout << "000000000000000000000000000000000000000000000000" << endl;

    RoutingRegion* routingData = new RoutingRegion();
    cout << "111111111111111111111111111111111111111111111111" << endl;
    parameter_set = ap.parameter(); // Global variable: routing parameters //還沒assign值
    cout << "222222222222222222222222222222222222222222222222" << endl;
    routing_parameter = ap.routing_param();
    
    pre_evaluate_congestion_cost_fp = pre_evaluate_congestion_cost_all; //指向這個function
    
    dataPreparetionISPD2024(ap, routingData);

    prog_start = std::chrono::high_resolution_clock::now();

    construct_2d_tree(routingData);

    post_start = std::chrono::high_resolution_clock::now();
    Post_processing();
    post_end = std::chrono::high_resolution_clock::now();

    if (ap.caseType() == 0)
    {
    }
    else
    {
	    la_start = std::chrono::high_resolution_clock::now();
        Layer_assignment(ap.get_outPR_file());
	    la_end = std::chrono::high_resolution_clock::now();
    }

    prog_end = std::chrono::high_resolution_clock::now();

    std::ofstream fout("./runtime.txt");
    assert(fout.is_open());

    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(prog_end - prog_start);
    std::cerr << std::endl <<"Runtime taken: " << time_span.count() << std::endl;
    fout << "Runtime taken: " << time_span.count() << std::endl;


    time_span = std::chrono::duration_cast<std::chrono::duration<double>>(main_end - main_start);
    std::cerr << std::endl <<"Main Route taken: " << time_span.count() << std::endl;
    fout << std::endl <<"Main Route taken: " << time_span.count() << std::endl;


    time_span = std::chrono::duration_cast<std::chrono::duration<double>>(post_end - post_start);
    std::cerr << std::endl <<"Post Route taken: " << time_span.count() << std::endl;
    fout << std::endl <<"Post Route taken: " << time_span.count() << std::endl;


    std::cerr << std::endl <<"Maze time taken: " << MAZE_TIME << std::endl;
    fout << std::endl <<"Maze time taken: " << MAZE_TIME << std::endl;


    time_span = std::chrono::duration_cast<std::chrono::duration<double>>(io_end - io_start);
    double IO_time = time_span.count();


    time_span = std::chrono::duration_cast<std::chrono::duration<double>>(la_end - la_start);
    std::cerr << std::endl <<"Layer Assignment taken: " << (time_span.count()-IO_time) << std::endl;
    fout << std::endl <<"Layer Assignment taken: " << (time_span.count()-IO_time) << std::endl;


    std::cerr << std::endl <<"I/O taken: " << IO_time << std::endl;
    fout << std::endl <<"I/O taken: " << IO_time << std::endl;


    std::cerr << std::endl <<"Gamer time taken: " << GAMER_TIME << std::endl;
    fout << std::endl <<"Gamer time taken: " << GAMER_TIME << std::endl;
    

    std::cerr << std::endl <<"Path search time taken: " << PATH_SEARCH_TIME << std::endl;
    fout << std::endl <<"Path search time taken: " << PATH_SEARCH_TIME << std::endl;


    std::cerr << std::endl <<"DP time taken: " << DP_TIME << std::endl;
    fout << std::endl <<"DP time taken: " << DP_TIME << std::endl;


    fout.close();

    free(routeAbsolutePath);
    return 0;
}


void dataPreparetionISPD2024(ParameterAnalyzer &ap, Builder *builder)
{
    assert(builder != NULL);
    std::cout << ap.get_cap_file() << ' ' << ap.get_net_file() << '\n';
    GRParser *parser = new Parser24(ap.get_cap_file(), ap.get_net_file());
    parser->parse(builder);
}
