//
// Created by 徐子卓 on 9/20/22.
//

#include "head.h"

int Graph::find_neighbor_random(int vertex, int num_step, vector<vector<pair<int,int>>> &RoadNetwork){

    int vertex_ini = vertex;

    int nei_size, nei_position;

    // srand((unsigned)time(NULL));

    for (int i=0;i<num_step;i++)
    {
        nei_size = RoadNetwork[vertex].size();

        if (nei_size == 0){

            break;
        }

        nei_position = (rand() % (nei_size - 0)) + 0;

        vertex = RoadNetwork[vertex][nei_position].first;
    }

    return vertex;
}

vector<int> Graph::Dij_vetex(int ID1, int ID2){

    /*
     * Description: Dijkstra’s algorithm to find the shortest path between two nodes.
     *
     * Parameters:
     * int ID1 -> source node.
     * int ID2 -> destination node.
     *
     * Return:
     * int d -> the shortest path between ID1 and ID2.
     */

    vector<int> vPath; vPath.clear();

    if(ID1==ID2) return vPath;
    //if(NodeOrder[ID1]==-1 || NodeOrder[ID2]==-1) return INF;
    benchmark::heap<2, int, int> pqueue(nodenum);
    pqueue.update(ID1,0);

    vector<bool> closed(nodenum, false);
    vector<int> distance(nodenum, INF);
    vector<int> vPrevious(nodenum, -1);
    vector<int> vPreviousEdge(nodenum, -1);

    distance[ID1]=0;
    int topNodeID, topNodeDis;
    int NNodeID,NWeigh;

    int d=INF;//initialize d to infinite for the unreachable case

    while(!pqueue.empty()){
        pqueue.extract_min(topNodeID, topNodeDis);
        if(topNodeID==ID2){
            d=distance[ID2];
            break;
        }
        closed[topNodeID]=true;

        for(auto it=Neighbor[topNodeID].begin();it!=Neighbor[topNodeID].end();it++){
            NNodeID=(*it).first;
            NWeigh=(*it).second+topNodeDis;
            if(!closed[NNodeID]){
                if(distance[NNodeID]>NWeigh){
                    distance[NNodeID]=NWeigh;
                    pqueue.update(NNodeID, NWeigh);
                    auto itr = map_nodeID_2_roadID.find(make_pair(topNodeID, NNodeID));
                    if(itr == map_nodeID_2_roadID.end())
                        cout << "No Road from " << topNodeID << " to " << NNodeID <<endl;
                    else
                    {
                        vPreviousEdge[NNodeID] = (*itr).second;
                    }
                    vPrevious[NNodeID] = topNodeID;
                }
            }
        }
    }

    vPath.push_back(ID2);
    int p = vPrevious[ID2];
    while(p != -1)
    {
        vPath.push_back(p);
        p = vPrevious[p];
    }

    reverse(vPath.begin(), vPath.end());

    return vPath;
}

void Graph::od_generation(
        vector<vector<int>> query_data, vector<vector<pair<int,int>>> &RoadNetwork, int itr_times,
        int time_range_left, int time_range_right, string query_out_file, string route_out_file){

    int source_vertex, dest_vertex;
    int nei_source_vertex, nei_dest_vertex;
    int num_step;
    int new_depart_time, gen_depart_time;
    int time_range, depar_time_left, depar_time_right;
    int Dij_size;
    vector<int> new_route;


/*    ofstream outfile_query;
    outfile_query.open(query_out_file);

    ofstream outfile_route;
    outfile_route.open(route_out_file);*/


    srand((unsigned)time(NULL));

    for (int i = 0; i < query_data.size(); i++)
    {
        // find od
        source_vertex = query_data[i][0]; dest_vertex = query_data[i][1];

        // randomly generate departure time
        gen_depart_time = (rand() % (time_range_right - time_range_left + 1)) + time_range_left;

        time_range = 2 * 60;

        depar_time_left = gen_depart_time - time_range;
        depar_time_right = gen_depart_time + time_range;

        if (depar_time_left < 0)
        {
            depar_time_left = 0;
        }

        for (int j = 0; j < itr_times; j++)
        {
            // Step 1: generate new od
            // ------------------------------------------------------------------------------

            // randomly generate nei step num
            num_step = (rand() % (5 - 1 + 1)) + 1;

            // generate new od vertex
            nei_source_vertex = find_neighbor_random(source_vertex, num_step, RoadNetwork);
            nei_dest_vertex = find_neighbor_random(dest_vertex, num_step, RoadNetwork);

            // Step 2: generate new departure time for new od
            // ------------------------------------------------------------------------------

            // generate new departure time
            new_depart_time = (rand() % (depar_time_right - depar_time_left + 1)) + depar_time_left;

            // Step 3: generate shortest path based on new od
            // ------------------------------------------------------------------------------

            new_route = Dij_vetex(nei_source_vertex, nei_dest_vertex); Dij_size = new_route.size();

            // Step 4: write generated route and new od into file
            // ------------------------------------------------------------------------------


/*
            outfile_query << nei_source_vertex << " " << nei_dest_vertex << " " << new_depart_time << endl;

            outfile_route << Dij_size <<  " ";

            for (int i = 0; i < new_route.size(); i++)
            {
                outfile_route << new_route[i] << " ";
            }

            outfile_route << endl;
*/

        }

    }


/*
    outfile_query.close(); outfile_route.close();
*/

}

void Graph::depar_time_adjustment(int file_index){

    /*调整to_string的参数选择读取的文件*/
    string q_path_part1 = Base + "trajectory_data/export_query/";
    string q_path_part2 = "/query_result_main";
    string q_index = to_string(file_index);  // here
    string q_path = q_path_part1 + q_index + q_path_part2;
    vector<vector<int>> query = ReadQuery(q_path);

    string route_path_part1 = Base + "route_combine/";
    string route_path_part2 = "_combine";
    string route_index = to_string(file_index); // here
    string route_path = route_path_part1 + route_index + route_path_part2;
    vector<vector<int>> route = ReadRoutes(route_path);


    /*清理不符合要求的route和related query data*/
    pair<vector<vector<int>>,vector<vector<int>>> route_query_dataPair;

    route_query_dataPair =  query_route_raw_clean(Neighbor, route, query);
    vector<vector<int>> route_data = route_query_dataPair.first;
    vector<vector<int>> query_data = route_query_dataPair.second;


    /*开两个文件写出来，写到对应的地址去*/
    string out_query = Base + "data_31d_in_4h/" + q_index + "/query" + q_index;
    ofstream outfile_query;
    outfile_query.open(out_query);

    int time_range_right = 60*60*4; // 随机从4个小时中选择出发时间

    for (int i = 0; i < query_data.size(); i++)
    {
        int depar_node = query_data[i][0];
        int desti_node = query_data[i][2];
        int depar_time = (rand() % (time_range_right - 0 + 1)) + 0;
        outfile_query << depar_node << " " << desti_node << " " << depar_time << " " << endl;
    }

    string out_route = Base + "data_31d_in_4h/" + route_index + "/route" + route_index;
    ofstream outfile_route;
    outfile_route.open(out_route);

    for (int i = 0; i < route_data.size(); i++)
    {
        outfile_route << route_data[i].size() << " ";
        for (int j = 0; j < route_data[i].size(); j++)
        {
            outfile_route << route_data[i][j] << " ";
        }
        outfile_route << endl;
    }

    outfile_route.close();
    outfile_query.close();
}