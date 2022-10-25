
#include "head.h"

int main() {

    Graph g; // call class Graph as g

    g.road_network_construction();
    g.map_construction();

/*
    g.depar_time_adjustment(3);
*/

/*    boost::thread_group threadf;

    for (int i=1;i<32;i++)
    {
        threadf.add_thread(new boost::thread(&Graph::depar_time_adjustment, i));

    }
    threadf.join_all();*/

/*

    // Step 1: DATA PREPARE
    // -----------------------------------------------------------------------------
    cout << "\n" << "Step 1: DATA PREPARE" << endl;
    cout << "-------------------------------------" << endl;

    */

/*
    g.data_preparation();
*/

    g.road_network_construction();
    g.map_construction();

    // read query and route raw data
    pair<vector<vector<int>>, vector<vector<int>>> query_route;

//    string query_path = g.Base + "trajectory_data/export_query/3/query_result_main";
//    string route_path = g.Base + "route_combine/3_combine";

//    string query_path = g.Base + "generated_data_3/generated_data_3_query";
//    string route_path = g.Base + "generated_data_3/generated_data_3_route";

//    string query_path = "/Volumes/RDMS/generated_data_3/generated_data_3_query";
//    string route_path = "/Volumes/RDMS/generated_data_3/generated_data_3_route";

    string query_path = g.Base + "data_31d_in_4h/query_31d_in_4h";
    string route_path = g.Base + "data_31d_in_4h/route_31d_in_4h";

    int read_num = 1000000;

    query_route = g.read_query_route(query_path, route_path, read_num);

    if (read_num > query_route.first.size())
    {
        cout << "Error. Selected data size is bigger than we have." << endl;
    }
/*
    vector<vector<int>> query_raw = query_route.first; vector<vector<int>> route_raw = query_route.second;
*/

    // 新代码读取数据的时候已经清理了不连续的routes，这里不需要再清理了如果用data_31d_in_4h的数据
    vector<vector<int>> query_data = query_route.first; vector<vector<int>> route_data = query_route.second;

/*    // clean query and route raw data
    pair<vector<vector<int>>,vector<vector<int>>> route_query_dataPair;

    // route_query_dataPair =  g.query_route_raw_clean(g.RoadNetwork, route_raw, query_raw);

    route_query_dataPair =  g.query_route_raw_clean(g.Neighbor, route_raw, query_raw);
    vector<vector<int>> route_data = route_query_dataPair.first;
    g.Pi = route_data;
    g.route_data_size = route_data.size();
    vector<vector<int>> query_data = route_query_dataPair.second;*/

    g.Pi = route_data;

    // find min departure time from queries.
    g.min_depar_time(query_data);

    //  convert route from node ID pair to road segment ID.
    g.route_N2R(route_data);



/*    // generate new od data
    cout << "data generation start" << endl;
    int time_range_left = 0;
    int time_range_right = 60*60*4;
    string query_out_file = g.Base + "generated_data_3/generated_data_3_query";
    string route_out_file = g.Base + "generated_data_3/generated_data_3_route";
    g.od_generation(query_data, g.RoadNetwork, 20, time_range_left, time_range_right, query_out_file,route_out_file);
    cout << "data generation done" << endl;*/


    // Step 2: ALGORITHM I SIMULATION
    // -----------------------------------------------------------------------------
    cout << "\n" << "Step 2: ALGORITHM I SIMULATION" << endl;
    cout << "-------------------------------------" << endl;


    // algorithm I simulation
    vector<vector<pair<int,int>>> Test = g.alg1Records(query_data, route_data);

/*
     int globalTime = g.global_evaluation(Test);
*/

/*    // Step 3: ROUTE DATA UPDATE OPERATIONS
    // -----------------------------------------------------------------------------
    cout << "\n" << "Step 3: ROUTE DATA UPDATE OPERATIONS" << endl;
    cout << "-------------------------------------" << endl;

    // convert time records from node ID pairs to road ID
    g.Route_timeFlowChange(g.timeFlowChange);
    // split time records into time slices
    g.RouteSlice_timeFlowChange(g.route_timeFlowChange);
    // check the correctness of time records
    g.TimeRecordCheck(g.TimeRecordInSlice);


    // 随机生成50000个new route和departure time
    string route_file = g.Base + "test_data/test_route";
    string depar_file = g.Base + "test_data/test_depar";
    string Pi_file = g.Base + "test_data/test_Pi";*/

/*    // 生成并储存到路径
    g.data_generation(route_data, query_data, route_file, depar_file, Pi_file, 100);
    cout << "test route and departure time generation done." << endl;*/

/*    // 正确性检验代码
    g.one_route_update_check_debug(route_file, depar_file, Pi_file);*/


/*    // check the correctness of one_route_update by sequence.
    g.one_route_update_check(route_data, query_data, 500);*/

/*    // check the correctness of one_route_update by threads
    vector<pair<pair<int, vector<int>>, int>> multi_test;
    multi_test = g.multi_task_initial(route_data, query_data, 500);
    g.one_route_parallel_update_check(multi_test);*/

    // Step 4: ... ...
    // -----------------------------------------------------------------------------
    cout << "\n" << "Step 4: ... ..." << endl;
    cout << "-------------------------------------" << endl;





    /* application of count flow num at specific time
    int flow = g.FlowNuminTime(g.NodesF, 5247193, 296703, 268606); cout << "traffic flow is: " << flow << endl;
    */

    /*
    // Parallel
    vector<vector<pair<int,int>>> baseline = g.Algorithm1(RoadNetwork, QCopy, PiCopy);
    vector<vector<pair<int,int>>> baseline_parallel = g.Parallel(RoadNetwork, QCopy, PiCopy, 3, BJ_NodeIDLonLat);

    // check if route and query data are clean
    int count_INF = 0;
    for (int i=0;i<baseline.size();i++){
        for (int j=0;j<baseline[i].size();j++){
            if (baseline[i][j].second == INF){
                count_INF += 1;
            }
        }
        cout << endl;
    }
    cout << "count of INF is: " << count_INF << endl;

    // check if results of algorithm1 and Parallel are difference
    int count = 0;
    int equal_count = 0;
    for (int i=0;i<baseline.size();i++){
        for (int j=0;j<baseline[i].size();j++){
            int basevalue = baseline[i][j].second;
            int basevalue_parrallel = baseline_parallel[i][j].second;
            if (basevalue == INF){
                break;
            }
            if (basevalue != basevalue_parrallel){
                count += 1;
                cout << i << ": " <<  j << " with " << basevalue << " "<< basevalue_parrallel << endl;
            }
            else{
                equal_count +=1;
            }
        }
    }
    cout << "the number of same is: " << equal_count << endl;
    cout << "the number of difference is: " << count << endl;
    */

    /*
    // Situation 2: multiple query and route
    // READ DATA
    // read query list
    string q_path_part1 = Base + "trajectory_data/export_query/";
    string q_path_part2 = "/query_result_main";
    g.Q_list = g.ReadQueryList(q_path_part1, q_path_part2, 31);
    cout << "Q_list size is: " << g.Q_list.size() << endl;
    cout << "--------------------------------------------------------------------" << endl;

    // read route list
    string route_path_part1 = Base + "route_combine/";
    string route_path_part2 = "_combine";
    g.Pi_list = g.ReadTrajectoryList(route_path_part1, route_path_part2, 31, g.Q_list);
    cout << "Pi_list size is: " << g.Pi_list.size() << endl;
    cout << "--------------------------------------------------------------------" << endl;

    vector<vector<vector<int>>> PiCopy_list(g.Pi_list.size());
    vector<vector<vector<int>>> QCopy_list(g.Q_list.size());
    for (int i=0;i<g.Pi_list.size();i++){
        vector<vector<vector<int>>> CleanedResult_temp =  g.RouteCLean(g.RoadNetwork, g.Pi_list[i], g.Q_list[i]);
        PiCopy_list[i] = CleanedResult_temp[0];
        QCopy_list[i] = CleanedResult_temp[1];
    }
    cout << "clean done" << endl;
    cout << "--------------------------------------------------------------------" << endl;

    vector<int> new_path = {282574, 285278, 285277, 285272, 286749, 285283, 274610, 294582, 11833, 11831, 11832,
                            268890, 266465, 289542, 262515, 262514, 246882, 246881, 262388, 11823, 268891, 246100, 289546, 262534, 14428, 286751};
    vector<int> new_query = {282574, 286751, 5274612};

    vector<vector<int>> PiTemp;
    vector<vector<int>> QTemp;
    for (int i=0;i<PiCopy_list.size();i++) {
        for (int j = 0; j < PiCopy_list[i].size(); j++) {
            PiTemp.push_back(PiCopy_list[i][j]);
            QTemp.push_back(QCopy_list[i][j]);
        }
        cout << "round " << i << " with size: " << PiTemp.size() << " start: " << endl;

//        vector<vector<pair<int,int>>> baseline = g.Algorithm1(g.RoadNetwork, QTemp, PiTemp);
//        vector<vector<pair<int,int>>> baseline_parallel = g.Parallel(g.RoadNetwork, QTemp, PiTemp, 3, BJ_NodeIDLonLat);


        vector<vector<pair<int,int>>> ETA_result = g.alg2_opt(new_path, PiTemp, new_query, QTemp);
        cout << "--------------------------------------------------------------------" << endl;
    }
    */

    /*
    // Parallel
    vector<vector<pair<int,int>>> baseline = g.Algorithm1(RoadNetwork, QCopy, PiCopy);
    vector<vector<pair<int,int>>> baseline_parallel = g.Parallel(RoadNetwork, QCopy, PiCopy, 3, BJ_NodeIDLonLat);

    // check if route and query data are clean
    int count_INF = 0;
    for (int i=0;i<baseline.size();i++){
        for (int j=0;j<baseline[i].size();j++){
            if (baseline[i][j].second == INF){
                count_INF += 1;
            }
        }
        cout << endl;
    }
    cout << "count of INF is: " << count_INF << endl;

    // check if results of algorithm1 and Parallel are difference
    int count = 0;
    int equal_count = 0;
    for (int i=0;i<baseline.size();i++){
        for (int j=0;j<baseline[i].size();j++){
            int basevalue = baseline[i][j].second;
            int basevalue_parrallel = baseline_parallel[i][j].second;
            if (basevalue == INF){
                break;
            }
            if (basevalue != basevalue_parrallel){
                count += 1;
                cout << i << ": " <<  j << " with " << basevalue << " "<< basevalue_parrallel << endl;
            }
            else{
                equal_count +=1;
            }
        }
    }
    cout << "the number of same is: " << equal_count << endl;
    cout << "the number of difference is: " << count << endl;
    */





    /*
    std::chrono::high_resolution_clock::time_point t0_1, t0_2;
    std::chrono::duration<double> time_span;
    t0_1=std::chrono::high_resolution_clock::now();

//    vector<vector<pair<int,int>>> result = g.Algorithm1(RoadNetwork, g.Q_list[0], g.Pi_list[0]);
    vector<vector<pair<int,int>>> result2 = g.Algorithm2(RoadNetwork, g.Q_list, g.Pi_list);

    t0_2=std::chrono::high_resolution_clock::now();
    time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t0_2-t0_1);
    cout << time_span.count() <<endl;
    */

//    for (int i=0;i<result2.size();i++){
//        cout << "route " << i << " is: ";
//        for (int j=0;j<result2[i].size();j++){
//            cout<< result2[i][j].second << " ";
//        }
//        cout << endl;
//    }

    /*
    std::chrono::high_resolution_clock::time_point t3, t4;
    std::chrono::duration<double> time_span;
    t3=std::chrono::high_resolution_clock::now();

    // Algorithm I
    vector<vector<pair<int,int>>> baseline = g.Algorithm1(RoadNetwork, Q, Pi);

    t4=std::chrono::high_resolution_clock::now();
    time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t4-t3);
    cout << time_span.count() <<endl;
    */


    return 0;
}

