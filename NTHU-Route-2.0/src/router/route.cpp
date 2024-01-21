#include <cstdio>

#include <cstdlib>

#include <ctime>

#include <cassert>

#include <chrono>

#include <vector>

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

#include "Coala.h"



double LARGE_NUM = 100000000;

void dataPreparetion(ParameterAnalyzer &ap, Builder *builder, std::shared_ptr<std::vector<std::vector<int>>> &layerOneCap);

void dataPreparetionISPD2024(ParameterAnalyzer &ap, Builder *builder);

extern void construct_2d_tree(RoutingRegion *);

extern void Post_processing();

extern void Layer_assignment(const std::string&);

extern void OutputResult(const char *);

extern double GAMER_TIME;
extern double PATH_SEARCH_TIME;
extern double MAZE_TIME;
extern double DP_TIME;

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

void readParams(const string &fileName) {
  fstream fin(fileName.c_str());
  string line;
  if (fin.is_open()) {
    while (fin.good()) {
      getline(fin, line);
      if (line[0] != '#') {
        char delimiter=':';
        int pos = line.find(delimiter);
        string field = line.substr(0, pos);
        string value = line.substr(pos + 1);
        stringstream ss(value);
        if (field == "rcong_min") {
            routing_parameter->set_rcong_min(std::stod(value));
            std::cout << "rcong_min:" << routing_parameter->get_rcong_min() << '\n';
        }
        else if (field == "rcong_diff") {
            routing_parameter->set_rcong_diff(std::stod(value));
            std::cout << "rcong_diff:" << routing_parameter->get_rcong_diff() << '\n';
        }
        else if (field == "rcongV_min") {
            routing_parameter->set_rcongV_min(std::stod(value));
            std::cout << "rcongV_min:" << routing_parameter->get_rcongV_min() << '\n';
        }
        else if (field == "rcongV_diff") {
            routing_parameter->set_rcongV_diff(std::stod(value));
            std::cout << "rcongV_diff:" << routing_parameter->get_rcongV_diff() << '\n';
        }
        else if (field == "rcongH_min") {
            routing_parameter->set_rcongH_min(std::stod(value));
            std::cout << "rcongH_min:" << routing_parameter->get_rcongH_min() << '\n';
        }
        else if (field == "rcongH_diff") {
            routing_parameter->set_rcongH_diff(std::stod(value));
            std::cout << "rcongH_diff:" << routing_parameter->get_rcongH_diff() << '\n';
        }
        else if (field == "rcongNonZeroV_min") {
            routing_parameter->set_NonZeroV_min(std::stod(value));
            std::cout << "rcongNonZeroV_min:" << routing_parameter->get_NonZeroV_min() << '\n';
        }
        else if (field == "rcongNonZeroV_diff") {
            routing_parameter->set_NonZeroV_diff(std::stod(value));
            std::cout << "rcongNonZeroV_diff:" << routing_parameter->get_NonZeroV_diff() << '\n';
        }
        else if (field == "rcongNonZeroH_min") {
            routing_parameter->set_NonZeroH_min(std::stod(value));
            std::cout << "rcongNonZeroH_min:" << routing_parameter->get_NonZeroH_min() << '\n';
        }
        else if (field == "rcongNonZeroH_diff") {
            routing_parameter->set_NonZeroH_diff(std::stod(value));
            std::cout << "rcongNonZeroH_diff:" << routing_parameter->get_NonZeroH_diff() << '\n';
        }
        else if (field == "init_reduct") {
            routing_parameter->set_init_reduct(std::stod(value));
            std::cout << "init_reduct:" << routing_parameter->get_init_reduct() << '\n';
        }
      }
    }
    fin.close();
  }
}

void runISPD2018Flow(std::string lef_file, std::string def_file, std::string guide_file)

{

    Rsyn::Session session;

    std::string lefFile = lef_file;

    std::string defFile = def_file;

    std::string outputFile = guide_file;

    Rsyn::ISPD2018Reader reader;

    const Rsyn::Json params = {

        {"lefFile", lefFile},

        {"defFile", defFile},

    };

    reader.load(params);

    database.init();

    grDatabase.init();

}

std::string getTestCaseName(const std::string &lef_file) {
    std::string testcase_name;
    std::string delimiter = "/";
    size_t pos = lef_file.find_last_of(delimiter);
    std::string tmp = lef_file.substr(pos+1);
    pos = tmp.find(".");
    testcase_name = tmp.substr(0, pos);
    return testcase_name;
}

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


// extern void sum_std(const std::vector<int> &input);
// extern void sum_pthread(const std::vector<int> &input);
// extern void sum_omp(const std::vector<int> &input);
// extern void parallel_prefix_sum(std::vector<long long>& input, std::vector<long long>& output);
extern void parallel_prefix_sum(long long *input, long long *output, long long n);
extern int parallel_sum(const std::vector<long long>& input);
int main(int argc, char *argv[])
{
    // omp_set_num_threads(8);
/*
    using namespace std::chrono;
    std::cout << "Maximum number of threads: " << omp_get_max_threads() << std::endl;
    // std::vector<int> ints(testSize);
    // std::vector<int> outs(testSize);
    // for (auto &i : ints) {
    //     i = 1;
    // }
    // auto seq_start = std::chrono::high_resolution_clock::now();
    // std::inclusive_scan(ints.cbegin(), ints.cend(), outs.begin());
    // auto seq_end = std::chrono::high_resolution_clock::now();
    // duration<double> time_span = duration_cast<duration<double>>(seq_end - seq_start);
    // std::cerr << std::endl <<"Overall time taken: " << time_span.count() << std::endl;

    // auto par_start = std::chrono::high_resolution_clock::now();
    // std::inclusive_scan(std::execution:par, ints.cbegin(), ints.cend(), outs.begin());
    // auto par_end = std::chrono::high_resolution_clock::now();
    // time_span = duration_cast<duration<double>>(par_end - par_start);
    // std::cerr << std::endl <<"Overall time taken: " << time_span.count() << std::endl;

    // omp_set_num_threads(128);
    // int n = 9;
    // std::vector<int> input(n);
    // for (int i=0; i<n; ++i)
    //     input[i] = i+1;
    long long n = 1e6; // size of the array
    // std::vector<long long> array(n);
    long long *array = new long long[n];
    long long *prefix_sum = new long long[n];

    // Initialize the array with values from 1 to n
    for (long long i = 0; i < n; i++) {
        array[i] = i + 1;
    }
    std::cout << "Parallel prefix sum start\n";
    auto start = std::chrono::high_resolution_clock::now();
    parallel_prefix_sum(array, prefix_sum, n);
    // prefix_sum[0] = array[0];
    // for (long long i=1; i<n; ++i) {
    //     prefix_sum[i] = prefix_sum[i-1] + array[i];
    // }
    // long long sum = prefix_sum[n-1];
    auto end = std::chrono::high_resolution_clock::now();
    // std::cout << "Sum: " << sum << std::endl;
    for (long long i=1; i<n; ++i) {
        assert(prefix_sum[i] == prefix_sum[i-1] + array[i]);
    }
    duration<double> time_span = duration_cast<duration<double>>(end - start);
    std::cerr << std::endl <<"Overall time taken: " << time_span.count() << std::endl;
    // int s1 = sum_std(input);
    // int s2 = sum_pthread(input);
    // int s3 = sum_omp(input);
    // assert(s1 == n*(n+1)/2);
    // assert(s2 == n*(n+1)/2);
    // assert(s3 == n*(n+1)/2);
    exit(0);
*/
    // omp_set_num_threads(32);
    // Rsyn::Session::init();

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

    ParameterAnalyzer ap(argc, argv);

    // std::cout << "lef : " << ap.get_lef_def()[0] << " def : " << ap.get_lef_def()[1] << " gudie : " << ap.get_guide() << "\n";
    // std::cout << "Parameter file: " << ap.get_param() << '\n';

    // TESTCASE_NAME = getTestCaseName(ap.get_lef_def()[0]);
    // std::cout << "Testcase name: " << TESTCASE_NAME << '\n';

    // runISPD2018Flow(ap.get_lef_def()[0], ap.get_lef_def()[1], ap.get_guide());

    RoutingRegion *routingData = new RoutingRegion();

    // auto layerOneCap = std::make_shared<std::vector<std::vector<int>>>(std::vector<std::vector<int>>(grDatabase.getNumGrPoint(X), std::vector<int>(grDatabase.getNumGrPoint(Y), 0)));

    parameter_set = ap.parameter(); // Global variable: routing parameters

    routing_parameter = ap.routing_param();

    pre_evaluate_congestion_cost_fp = pre_evaluate_congestion_cost_all;

    auto get_dir_path = [](std::string guide_path) { 
        std::string delimiter = "/";
        size_t pos = guide_path.find_last_of(delimiter);
        std::string dir_path = guide_path.substr(0, pos+delimiter.length()); 
        return dir_path;
    };
    auto write_params = [&](const std::string dir_path) {
        std::string param_path = dir_path + "param.txt";
        std::cout << "Writing parameter file to " << param_path << '\n';
        ofstream fout(param_path);
        assert(fout.is_open());

        fout << "rcong_min:" << routing_parameter->get_rcong_min() << '\n';
        fout << "rcong_diff:" << routing_parameter->get_rcong_diff() << '\n';
        fout << "rcongV_min:" << routing_parameter->get_rcongV_min() << '\n';
        fout << "rcongV_diff:" << routing_parameter->get_rcongV_diff() << '\n';
        fout << "rcongH_min:" << routing_parameter->get_rcongH_min() << '\n';
        fout << "rcongH_diff:" << routing_parameter->get_rcongH_diff() << '\n';
        fout << "rcongNonZeroV_min:" << routing_parameter->get_NonZeroV_min() << '\n';
        fout << "rcongNonZeroV_diff:" << routing_parameter->get_NonZeroV_diff() << '\n';
        fout << "rcongNonZeroH_min:" << routing_parameter->get_NonZeroH_min() << '\n';
        fout << "rcongNonZeroH_diff:" << routing_parameter->get_NonZeroH_diff() << '\n';
        fout << "init_reduct:" << routing_parameter->get_init_reduct() << '\n';

        fout.close();
    };

    // std::string dir_path = get_dir_path(ap.get_guide());
    // std::cout << "Dir path: " << dir_path << '\n';
    // readParams(ap.get_param());
    // write_params(dir_path);
    // dataPreparetion(ap, routingData, layerOneCap);
    dataPreparetionISPD2024(ap, routingData);
    // exit(0);

    using namespace std::chrono;
    prog_start = high_resolution_clock::now();

    // clock_t t1 = clock();
    
    construct_2d_tree(routingData);

    // clock_t t2 = clock();

    post_start = std::chrono::high_resolution_clock::now();
    Post_processing();
    post_end = std::chrono::high_resolution_clock::now();

    // clock_t t3 = clock();

    // printf("\033[33mtime:\033[m %.2f %.2f %.2f\n", (double)(t2 - t1) / CLOCKS_PER_SEC, (double)(t3 - t2) / CLOCKS_PER_SEC, (double)(t3 - t1) / CLOCKS_PER_SEC);



    if (ap.caseType() == 0)
    {
    }
    else
    {
	    la_start = std::chrono::high_resolution_clock::now();
        Layer_assignment(ap.get_outPR_file());
	    la_end = std::chrono::high_resolution_clock::now();
    }

    prog_end = high_resolution_clock::now();

    std::ofstream fout("./runtime.txt");
    assert(fout.is_open());

    duration<double> time_span = duration_cast<duration<double>>(prog_end - prog_start);
    std::cerr << std::endl <<"Runtime taken: " << time_span.count() << std::endl;
    fout << "Runtime taken: " << time_span.count() << std::endl;

    // time_span = duration_cast<duration<double>>(pattern_end - pattern_start);
    // std::cerr << std::endl <<"Pattern Route taken: " << time_span.count() << std::endl;
    // fout << std::endl <<"Pattern Route taken: " << time_span.count() << std::endl;

    time_span = duration_cast<duration<double>>(main_end - main_start);
    std::cerr << std::endl <<"Main Route taken: " << time_span.count() << std::endl;
    fout << std::endl <<"Main Route taken: " << time_span.count() << std::endl;


    time_span = duration_cast<duration<double>>(post_end - post_start);
    std::cerr << std::endl <<"Post Route taken: " << time_span.count() << std::endl;
    fout << std::endl <<"Post Route taken: " << time_span.count() << std::endl;

    std::cerr << std::endl <<"Maze time taken: " << MAZE_TIME << std::endl;
    fout << std::endl <<"Maze time taken: " << MAZE_TIME << std::endl;

    time_span = duration_cast<duration<double>>(io_end - io_start);
    // std::cerr << std::endl <<"I/O taken: " << time_span.count() << std::endl;
    // fout << std::endl <<"I/O taken: " << time_span.count() << std::endl;

    double IO_time = time_span.count();

    time_span = duration_cast<duration<double>>(la_end - la_start);
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

    return 0;
}


void dataPreparetionISPD2024(ParameterAnalyzer &ap, Builder *builder)
{
    assert(builder != NULL);
    std::cout << ap.get_cap_file() << ' ' << ap.get_net_file() << '\n';
    GRParser *parser = new Parser24(ap.get_cap_file(), ap.get_net_file());
    parser->parse(builder);
}

void dataPreparetion(ParameterAnalyzer &ap, Builder *builder, std::shared_ptr<std::vector<std::vector<int>>> &layerOneCap)

{

    assert(builder != NULL);



    GRParser *parser;

    if (ap.caseType() == 0)

    {

        parser = new Parser98(ap.input(), FileHandler::AutoFileType);

    }

    else if (ap.caseType() == 1)

    {

        parser = new Parser07(ap.input(), FileHandler::AutoFileType);

    }

    else

    {

        std::cout<<"address 1 = "<<layerOneCap<<"\n";

        parser = new Parser18(layerOneCap);

        std::cout << "using LEF/DEF testcases ! \n";

    }

    parser->parse(builder);

}

