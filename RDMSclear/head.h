//
// Created by 徐子卓 on 5/7/22.
//

#ifndef ETA_HEAD_H
#define ETA_HEAD_H


#include <stdio.h>
#include <string.h>
#include <vector>
#include <map>
#include <set>

#include<iostream>
#include<fstream>
#include<math.h>
#include <chrono>
#include <unordered_map>
#include <unordered_set>
#include <ctime>

#include <functional>
#include <utility>
#include <string>

#include <algorithm>
#include <map>
#include <thread>
#include <future>
#include <boost/thread/thread.hpp>
#include <semaphore.h>
#include "boost/thread.hpp"
#include <time.h>
#include <cstdlib>


#define INF 999999999

using namespace std;

class Semaphore {
public:
    explicit Semaphore(int count = 0) : count_(count) {
    }

    void Signal() {
        std::unique_lock<std::mutex> lock(mutex_);
        ++count_;
        cv_.notify_one();
    }

    void Wait() {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [=] { return count_ > 0; });
        --count_;
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    int count_;
};

typedef struct ROAD
{
    int roadID;
    int ID1, ID2;
    int length;
    int travelTime;
}Road;

class Graph{
public:

    // functional functions
    // ------------------------------------------------------------------------------

    // find duplicate values
    template<typename T>
    std::set<T> findDuplicates(std::vector<T> vec)
    {
        std::set<int> duplicates;
        std::sort(vec.begin(), vec.end());
        std::set<int> distinct(vec.begin(), vec.end());
        std::set_difference(vec.begin(), vec.end(), distinct.begin(), distinct.end(),
                            std::inserter(duplicates, duplicates.end()));
        return duplicates;
    }

    // data path
    string Base = "/export/project/zizhuo/data/modified_data2/";
    string BJ = Base + "BJ_data/BJ";
    string beijingNodeNew = Base + "BJ_data/beijingNodeNew";
    string BJ_nodeRoadID = Base + "BJ_data/BJ_nodeRoadID";
    string BJ_node_new_RoadID = Base + "BJ_data/BJ_node_new_RoadID";
    string beijingRoadSpeedLimit = Base + "BJ_data/beijingRoadSpeedLimit";
    string BJ_minTravleTime = Base + "BJ_data/BJ_minTravleTime";
    string BJ_NodeIDLonLat = Base + "BJ_data/BJ_NodeIDLonLat";
    string BJ_NodeWeight = Base + "BJ_data/BJ_NodeWeight";
    string BJ_RoadIDWeight = Base + "BJ_data/BJ_RoadIDWeight";
    string temp1 = Base + "BJ_data/temp1";
    string path1 = Base + "BeijingTrajectoryMatched/Beijing_trajectory/25/result_";
    string path2 = Base + "trajectory_data/trajectory_data/1/result_";
    string path3 = Base + "trajectory_data/route/1/result_";
    string export_route = Base + "trajectory_data/export_route/2/result_";
    string export_query = Base + "trajectory_data/export_query/1/query_result_main";
    string beijingRoadMap = Base + "BeijingTrajectoryMatched/Beijing_trajectory/beijingRoadMap";

    void data_preparation();
    void map_construction();
    void road_network_construction();
    pair<vector<vector<int>>, vector<vector<int>>> read_query_route(string query_path, string route_path, int read_num);
    vector<vector<int>> ReadQuery_w_num(string filename, int num);

    pair<vector<vector<int>>,vector<vector<int>>> query_route_raw_clean(
            vector<vector<pair<int,int>>> RoadNetwork, vector<vector<int>> route_raw, vector<vector<int>> query_raw);

    vector<vector<int>> only_route_raw_clean(vector<vector<pair<int,int>>> RoadNetwork, vector<vector<int>> route_raw);

        int minDeparture, minHour; void min_depar_time(vector<vector<int>> &Q);
    void route_N2R(vector<vector<int>> &Pi); vector<vector<int>> PiRoadSegmentID;

    int nodenum; int edgenum;

    vector<vector<pair<int,int>>> RoadNetwork;

    vector<vector<vector<int>>> Pi_list;
    vector<vector<vector<int>>> Q_list;

    vector<Semaphore*> vLock;


    int eNum;//(a,b) is a edge with a<b
    vector<set<int>> AdjacentNodes;

    //bidirectional graph
    vector<vector<pair<int,int>>> Neighbor;
    vector<vector<pair<int, int>>> NeighborRoad;
    vector<pair<double,double>> GraphLocation;//location used in A* algorithm
    vector<Road> vRoad;
    // 转换roadID
    map<int,int> mapRoadID;
    map<int,pair<int,int>> mapRoadIDNodeID;

    // 读取结构复合(ID1,ID2,weight)的路网
    void ReadGraph(std::string filename);

    // Dijkstra’s算法
    int Dij(int ID1, int ID2);

    // 提取以时间为权重的路网信息
    // 提取路段信息的函数
    void ReadRoadID(std::string filename, string export_filename);
    // 把道路限速转换成vector的形式
    vector<int> speedLim;
    void speedLim2vector(string filename);
    // 根据限速和道路长度获得以时间为权重的路网
    void ExportminTime(string filename, string export_filename);
    // 提取node以及其经纬度
    void ReadNodeIDLonLat(string filename, string export_filename);
    // 提取两个相连node1和node2之间的weight
    void ReadWeight(string filename, string export_filename);
    // 提取两个相连node1和node2之间的weight以及roadID
    void ReadRoadIDWeight(string filename1, string filename2, string export_filename);


    int FindIndex(int element, vector<int> &Vector){

        /*
         * Description: This function find element's index in a vector.
         *
         * Parameters:
         * element -> target element.
         * Vectorr -> target vector contianed element.
         *
         * Return:
         * index -> index of element in the vector.
         */

        int index;

        for(int i=0;i<Vector.size();i++){
            if(Vector[i] == element){
                index = i; return index;
            }
        }

        cout << "there is no required element in the vector" << endl;
        index = INF; return index;
    }

    int CountLines(string filename){
        ifstream ReadFile;
        int n=0;
        char line[512];
        string temp;
        ReadFile.open(filename,ios::in);//ios::in 表示以只读的方式读取文件
        if(ReadFile.fail())//文件打开失败:返回0
        {
            return 0;
        }
        else//文件存在
        {
            while(getline(ReadFile,temp))
            {
                n++;
            }
            return n;
        }
        ReadFile.close();
    }

    bool check_range(double value, double min, double max){
        if (value < max and value > min){
            return true;
        }
        else{
            return false;
        }
    }

    vector<vector<pair<int,int>>> Algorithm1(vector<vector<pair<int, int>>> &G,
                                             vector<vector<int>> &Q, vector<vector<int>> &Pi);
    vector<vector<pair<int,int>>> Algorithm2(vector<vector<pair<int, int>>> &G,
                                             vector<vector<vector<int>>> &Q, vector<vector<vector<int>>> &Pi);
    bool OverlapDet(vector<int> &Pi1, vector<int> &Pi2);
    int OverlapReturn(vector<int> &Pi1, vector<int> &Pi2);
    vector<vector<pair<int,int>>> Algorithm1_V1(vector<vector<pair<int, int>>> &G,
                                                vector<vector<int>> &Q, vector<vector<int>> &Pi);
    vector<vector<pair<int,int>>> Algorithm2_V1(vector<vector<pair<int, int>>> &G,
                                                vector<vector<vector<int>>> &Q, vector<vector<vector<int>>> &Pi);
    vector<vector<vector<int>>> ReadTrajectoryList(string path_part1, string path_part2, int Listnum, vector<vector<vector<int>>> Q_List);
    vector<vector<vector<int>>> ReadQueryList(string path_part1, string path_part2, int filenum);


    // 读取轨迹数据并修改时间
    void modify_trajectory(string path, string export_filename, int time, int fileNum);
    void export_trajectory(string path, string export_filename, int fileNum);

    void CreateMap(string filename);

    void CreateRoadIDMapNodeID(string filename);

    void export_query_trajectory(string path1, string path2, string path3, int fileNum);

    vector<vector<int>> ReadQuery(string path);

    vector<vector<int>> ReadTrajectory(string path, int fileNum);

    vector<vector<pair<int,int>>> ReadRoadNetwork(string filename);

    // algorithm from Shuo Shang's paper
    vector<vector<int>> Toward(string filename, vector<vector<int>> Q);

    // Parallel Computation
    pair<vector<double>, vector<double>> Graph2Grids(double lat1, double lat2, double lon1, double lon2, int num);
    map<int,pair<double,double>> IDtoLocation (string filename);
    map<pair<int,int>, int> GridsIndex (int num);
    vector<vector<pair<int,int>>> Parallel(vector<vector<pair<int, int>>> &G, vector<vector<int>> &Q, vector<vector<int>> &Pi, int num, string filename);

    static void Algorithm1_Parallel(vector<vector<pair<int, int>>> &G,
                                    vector<vector<int>> &Q, vector<vector<int>> &Pi,
                                    vector<vector<pair<int,int>>>& GResult);
    void RouteCombine(vector<vector<int>> Pi, string export_file);
    vector<vector<int>> ReadRoutes(string input_file);
    vector<vector<int>> ReadRoutes_w_num(string input_file, int lines, int read_num);

//    vector<vector<pair<int,int>>> Algorithm1Record(vector<vector<pair<int, int>>> &G, vector<vector<int>> &Q,
//                                                   vector<vector<int>> &Pi, map<pair<int, int>, int> NodeID2MinTravelTime);


//    // 构建RoadID和NodeID相互转换的map
//    map<int, pair<int, int>> RoadID2NodeID(string filename);
//    map<pair<int, int>, int> NodeID2RoadID(string filename);
//    //
//    map<pair<int, int>, int> NodeID2MinTravelTime(string filename);

    pair<vector<vector<pair<int,int>>>, vector<pair<pair<int, int>,vector<int>>>> Alg1forAlg2Opt(vector<vector<int>> &Q, vector<pair<pair<int, int>, vector<int>>> &Pi, vector<vector<pair<int,int>>> exit_path_ETA_result, int node1, int node2);

        vector<vector<pair<int,int>>> Algorithm2_Opt(vector<vector<pair<int, int>>> &G, vector<int> newPi, vector<vector<int>> Pi, map<pair<int, int>, int> NodeID2MinTravelTime,
                                                 map<pair<int, int>, int> NodeID2RoadID, map<int, pair<int, int>> RoadID2NodeID,
                                                 vector<vector<int>> Q, vector<int> newQ, vector<vector<pair<int,int>>> GResult);
    bool CheckChange(vector<vector<pair<int, int>>> &G, vector<vector<int>> &Q,
                     vector<pair<pair<int, int>, vector<int>>> &Pi,
                     map<pair<int, int>, int> NodeID2MinTravelTime,
                     vector<vector<pair<int,int>>> Result, int node1, int node2,
                     vector<vector<int>> label);

    // Alg2.cpp
    map<pair<int, int>, int> map_nodeID_2_roadID;

    void construct_map_nodeID_2_roadID(string filename);
    int new_edgenum;
    vector<vector<int>> path_nodeID_2_roadID (vector<vector<int>> Pi);

    map<int, pair<int, int>> map_roadID_2_nodeID;
    void construct_map_roadID_2_nodeID(string filename);

    map<pair<int, int>, int> map_nodeID_2_minTime;
    void construct_map_nodeID_2_minTime(string filename);

    vector<vector<vector<int>>> nodes_label; // -> {pathID, node_index, time}
    vector<vector<pair<int,vector<pair<int, int>>>>> time_w_flow_change;
    vector<vector<pair<int,int>>> alg1_w_records(vector<vector<int>> &Q, vector<vector<int>> &Pi);

    vector<vector<vector<int>>> edge_label;
    void nodes_label_2_edge_label(vector<vector<int>> &Pi);

    map<int,vector<vector<int>>> construct_inverted_table(vector<vector<vector<int>>> paths_w_roadID_time);
    void show_inverted_table(map<int,vector<vector<int>>> inverted_table);
    vector<vector<int>> inverted_table_query(int edgeID, map<int,vector<vector<int>>> inverted_table);

    vector<vector<pair<int,int>>> alg2_opt(vector<int> new_path,vector<vector<int>> Pi, vector<int> new_query, vector<vector<int>> Q);

    int capture_flow_by_time(int time, int node1, int node2);

    void BJ_nodeRoadID_w_new_roadID(string filename, string export_filename);

    // structure.cpp
//    vector<vector<pair<int,map<int, vector<int>>>>> timeFlowChange;
    vector<vector<pair<int,map<int, vector<vector<int>>>>>> timeFlowChange;


    vector<vector<pair<int,int>>> alg1Records(vector<vector<int>> &Q, vector<vector<int>> &Pi);
    vector<map<int, vector<vector<int>>>> route_timeFlowChange;
    void Route_timeFlowChange(vector<vector<pair<int,map<int, vector<vector<int>>>>>> &timeFlowChange);
    vector<vector<map<int, vector<vector<int>>>>> TimeRecordInSlice; // RoadSegmentID: TimeSlice: TimeRecord
    void RouteSlice_timeFlowChange(vector<map<int, vector<vector<int>>>> &route_timeFlowChange);
    int Time2Hour(int intTime);

    int Hour2Index(int hour);
    int IndexFlow(vector<vector<map<int, vector<vector<int>>>>> &TimeRecordInSlice, int timeValue, int RoadSegmentID, vector<int> features);
    vector<vector<map<int, vector<vector<int>>>>> timeRecordsChecked;
    void TimeRecordCheck(vector<vector<map<int, vector<vector<int>>>>> &TimeRecordInSlice);
    int timeRecords_correct_check(map<int, vector<vector<int>>> timeRecords);

    float sigma = 0.15; float varphi = 20; float beta = 2;
    pair<int, vector<pair<int, pair<int,int>>>> RouteUpdateInitial(pair<int, vector<int>> &Route, int DriveInTime);
    // map<int, vector<int>> RouteUpdate(pairRoadNetwork<int, vector<int>> RouteNew, int DriveInTime, vector<vector<int>> &Pi);

    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> updateOperation1st(pair<int,int> RoadSegmentID, int inTime, pair<int, vector<int>> newRoutePair);
    vector<int> route_node2roadSegment(vector<int> route_node);

    int findNextRoadSegmentID(vector<int> route, int roadSegmentID);

    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> updateOperationFurther(
            pair<int,int> RoadSegmentID, int inTime, pair<int, vector<int>> newRoutePair, map<int, vector<vector<int>>> InsertPre, vector<int> DeletionPre);

    int global_evaluation(vector<vector<pair<int,int>>> &ETA_result);
    int flowEvaluation(vector<vector<map<int, vector<int>>>> &timeRecordsChecked, int searchTime, int RoadSegmentID);
    vector<vector<int>> Pi; int route_data_size;

    void multi_route_update(vector<pair<pair<int, vector<int>>, int>> &multi_test);

    vector<int> one_route_parallel_update(pair<int, vector<int>> newRoute, int inTime, vector<Semaphore*>& vLock);
    vector<int> affected_parallel_roadID;
    void one_route_parallel_update_check(vector<pair<pair<int, vector<int>>, int>> multi_test);

    vector<int> one_route_update(pair<int, vector<int>> newRoute, int inTime);
    void one_route_update_check(vector<vector<int>> &route_data,  vector<vector<int>> &query_data, int ad_num);

    vector<pair<pair<int, vector<int>>, int>> multi_task_initial(vector<vector<int>> &route_data,  vector<vector<int>> &query_data, int iNum);

    void data_generation(vector<vector<int>> &route_data,  vector<vector<int>> &query_data,
                                string route_file, string depar_file, string Pi_file, int adj_num);

    void one_route_update_check_debug(string route_file, string depar_file, string Pi_file);

        // od_generation.cpp
    int find_neighbor_random(int vertex, int num_step, vector<vector<pair<int,int>>> &RoadNetwork);

    void od_generation(vector<vector<int>> query_data, vector<vector<pair<int,int>>> &RoadNetwork, int itr_times,
                       int time_range_left, int time_range_right, string query_out_file, string route_out_file);

    vector<int> Dij_vetex(int ID1, int ID2);

    void depar_time_adjustment(int file_index);
    };

namespace benchmark {

#define NULLINDEX 0xFFFFFFFF

    template<int log_k, typename k_t, typename id_t>
    class heap {

    public:

        // Expose types.
        typedef k_t key_t;
        typedef id_t node_t;

        // Some constants regarding the elements.
        //static const node_t NULLINDEX = 0xFFFFFFFF;
        static const node_t k = 1 << log_k;

        // A struct defining a heap element.
        struct element_t {
            key_t key;
            node_t element;

            element_t() : key(0), element(0) {}

            element_t(const key_t k, const node_t e) : key(k), element(e) {}
        };


    public:

        // Constructor of the heap.
        heap(node_t n) : n(0), max_n(n), elements(n), position(n, NULLINDEX) {
        }

        heap() {

        }

        // Size of the heap.
        inline node_t size() const {
            return n;
        }

        // Heap empty?
        inline bool empty() const {
            return size() == 0;
        }

        // Extract min element.
        inline void extract_min(node_t &element, key_t &key) {
            assert(!empty());

            element_t &front = elements[0];

            // Assign element and key.
            element = front.element;
            key = front.key;

            // Replace elements[0] by last element.
            position[element] = NULLINDEX;
            --n;
            if (!empty()) {
                front = elements[n];
                position[front.element] = 0;
                sift_down(0);
            }
        }

        inline key_t top() {
            assert(!empty());

            element_t &front = elements[0];

            return front.key;

        }

        inline node_t top_value() {

            assert(!empty());

            element_t &front = elements[0];

            return front.element;
        }

        // Update an element of the heap.
        inline void update(const node_t element, const key_t key) {
            if (position[element] == NULLINDEX) {
                element_t &back = elements[n];
                back.key = key;
                back.element = element;
                position[element] = n;
                sift_up(n++);
            } else {
                node_t el_pos = position[element];
                element_t &el = elements[el_pos];
                if (key > el.key) {
                    el.key = key;
                    sift_down(el_pos);
                } else {
                    el.key = key;
                    sift_up(el_pos);
                }
            }
        }


        // Clear the heap.
        inline void clear() {
            for (node_t i = 0; i < n; ++i) {
                position[elements[i].element] = NULLINDEX;
            }
            n = 0;
        }

        // Cheaper clear.
        inline void clear(node_t v) {
            position[v] = NULLINDEX;
        }

        inline void clear_n() {
            n = 0;
        }


        // Test whether an element is contained in the heap.
        inline bool contains(const node_t element) const {
            return position[element] != NULLINDEX;
        }


    protected:

        // Sift up an element.
        inline void sift_up(node_t i) {
            assert(i < n);
            node_t cur_i = i;
            while (cur_i > 0) {
                node_t parent_i = (cur_i - 1) >> log_k;
                if (elements[parent_i].key > elements[cur_i].key)
                    swap(cur_i, parent_i);
                else
                    break;
                cur_i = parent_i;
            }
        }

        // Sift down an element.
        inline void sift_down(node_t i) {
            assert(i < n);

            while (true) {
                node_t min_ind = i;
                key_t min_key = elements[i].key;

                node_t child_ind_l = (i << log_k) + 1;
                node_t child_ind_u = std::min(child_ind_l + k, n);

                for (node_t j = child_ind_l; j < child_ind_u; ++j) {
                    if (elements[j].key < min_key) {
                        min_ind = j;
                        min_key = elements[j].key;
                    }
                }

                // Exchange?
                if (min_ind != i) {
                    swap(i, min_ind);
                    i = min_ind;
                } else {
                    break;
                }
            }
        }

        // Swap two elements in the heap.
        inline void swap(const node_t i, const node_t j) {
            element_t &el_i = elements[i];
            element_t &el_j = elements[j];

            // Exchange positions
            position[el_i.element] = j;
            position[el_j.element] = i;

            // Exchange elements
            element_t temp = el_i;
            el_i = el_j;
            el_j = temp;
        }


    private:

        // Number of elements in the heap.
        node_t n;

        // Number of maximal elements.
        node_t max_n;

        // Array of length heap_elements.
        vector<element_t> elements;

        // An array of positions for all elements.
        vector<node_t> position;
    };
}

#endif //ETA_HEAD_H
