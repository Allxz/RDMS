//
// Created by 徐子卓 on 5/7/22.
//

#include "head.h"


int count_lines(string filename){

    /*
     * Description: calculate the number of lines in a text file.
     * Arguments: string filename -> address of text file.
     * Return: int n -> the number of lines in this text file.
     */

    // Step 1: 打开文件
    // -------------------------------------------------------
    ifstream ReadFile;
    int n=0; char line[512]; string temp;
    ReadFile.open(filename,ios::in);//ios::in 表示以只读的方式读取文件

    // Step 2: 计算文件行数
    // -------------------------------------------------------
    if(ReadFile.fail()){
        return 0;
    }
    else{
        while(getline(ReadFile,temp)){
            n++;
        }
        return n;
    }
    ReadFile.close();
}


void Graph::construct_map_nodeID_2_roadID(string filename){

    /*
     * Description: construct map to transfer nodeID pair into road ID.
     * Arguments: string filename -> address of text file: nodeID, nodeID, roadID.
     */

    // Step 1: clear
    // -------------------------------------------------------
    map_nodeID_2_roadID.clear();

    // Step 2: 打开文件
    // -------------------------------------------------------
    ifstream IF(filename);
    if(!IF){
        cout<<"Cannot open Map "<<filename<<endl;
    }

    // Step 3: 读入数据&构建map
    // -------------------------------------------------------
    int nodenum; IF >> nodenum; // 读取nodenum
    //int lines = count_lines(filename);
    new_edgenum = 0;
    int nodeID1, nodeID2, roadID;
    while(IF >> nodeID1)
    {
        IF  >> nodeID2 >> roadID;
        Road r;
        r.ID1 = nodeID1;
        r.ID2 = nodeID2;
        r.roadID = roadID;
        map_nodeID_2_roadID[make_pair(nodeID1,nodeID2)] = roadID;
    }

    for(int i = 0; i < Neighbor.size(); i++)
    {
        for(int j = 0; j < Neighbor[i].size(); j++)
        {
            map<pair<int, int>, int>::iterator it = map_nodeID_2_roadID.find(make_pair(i, Neighbor[i][j].first));
            if(it == map_nodeID_2_roadID.end())
            {
                if(map_nodeID_2_roadID.find(make_pair(Neighbor[i][j].first,i)) == map_nodeID_2_roadID.end())
                    cout << "ERROR no road between " << i << " and " <<Neighbor[i][j].first << endl;
                map_nodeID_2_roadID.insert(make_pair(make_pair(i,Neighbor[i][j].first), map_nodeID_2_roadID[make_pair(Neighbor[i][j].first,i)]));
            }
        }
    }

    /*
    // Step 4: 打印 map
    // -------------------------------------------------------
    map<pair<int,int>, int>::iterator itr;
    cout << "\nThe map NodeID2RoadID is : \n";
    cout << "\tKEY\tELEMENT\n";
    for (itr = map_nodeID_2_roadID.begin(); itr != map_nodeID_2_roadID.end(); ++itr) {
        cout << '\t' << itr->first.first << '\t' << itr->first.second << '\t'
             << itr->second << '\n';
    }
    cout << endl;
    */
}


vector<vector<int>> Graph::path_nodeID_2_roadID (vector<vector<int>> Pi){

    /*
     * Description: transfer nodeID constructed path into roadID constructed path.
     * Arguments:
     * Pi -> paths constructed by nodeID
     */

    // Step 1: 从nodeID constructed path转换成roadID constructed path
    // -------------------------------------------------------
    vector<vector<int>> path_roadID(Pi.size());
    for (int i=0;i<Pi.size();i++){
        path_roadID[i].resize(Pi[i].size()-1);
    }
    for (int i=0;i<Pi.size();i++){
        for (int j=0;j<Pi[i].size()-1;j++){
            path_roadID[i][j] = map_nodeID_2_roadID[make_pair(Pi[i][j],Pi[i][j+1])];
        }
    }

    /*
    // Step 2: 打印roadID constructed path
    // -------------------------------------------------------
    for (int i=0;i<PiRoadID.size();i++){
        cout << "path " << i << " : ";
        for (int j=0;j<PiRoadID[i].size();j++){
            cout << PiRoadID[i][j] << " ";
        }
        cout << endl;
    }
    */

    // Step 3: return path with roadID
    // -------------------------------------------------------
    return path_roadID;
}


void Graph::construct_map_roadID_2_nodeID(string filename){

    /*
     * Description: construct map to transfer road ID into nodeID pairs.
     * Arguments: string filename -> address of text file: nodeID, nodeID, roadID.
     */

    // Step 1: clear map_roadID_2_nodeID
    // -------------------------------------------------------
    map_roadID_2_nodeID.clear();

    // Step 2: 打开文件
    // -------------------------------------------------------
    ifstream IF(filename);
    if(!IF){
        cout<<"Cannot open Map "<<filename<<endl;
    }

    // Step 3: 读入数据&构建map
    // -------------------------------------------------------
    int nodenum; IF >> nodenum; // 读取nodenum
    int lines = count_lines(filename);

    int nodeID1, nodeID2, roadID;
    for (int i=0;i<lines;i++){
        IF  >> nodeID1 >> nodeID2 >> roadID;
        map_roadID_2_nodeID[roadID] = make_pair(nodeID1,nodeID2);
    }

    /*
    // Step 4: 打印 map
    // -------------------------------------------------------
    map<int, pair<int,int>>::iterator itr;
    cout << "\nThe map RoadID2NodeID is : \n";
    cout << "\tKEY\tELEMENT\n";
    for (itr = map_roadID_2_nodeID.begin(); itr != map_roadID_2_nodeID.end(); ++itr) {
        cout << '\t' << itr->first << '\t' << itr->second.first << '\t'
        << itr->second.second << '\n';
    }
    cout << endl;
    */
}


void Graph::construct_map_nodeID_2_minTime(string filename){

    /*
     * Description: construct map to transfer road ID into min travel time.
     * Arguments: string filename -> address of text file: nodeID, nodeID, minTime.
     */

    // Step 1: clear map_nodeID_2_minTime
    // -------------------------------------------------------
    map_nodeID_2_minTime.clear();

    // Step 2: 打开文件
    // -------------------------------------------------------
    ifstream IF(filename);
    if(!IF){
        cout<<"Cannot open Map "<<filename<<endl;
    }

    // Step 3: 读入数据&构建map
    // -------------------------------------------------------
    int nodenum; IF >> nodenum; // 读取nodenum
   // int lines = count_lines(filename);

    int nodeID1, nodeID2, MinTravelTime;
    while(IF >> nodeID1)
    {
        IF  >> nodeID2 >> MinTravelTime;
        map_nodeID_2_minTime[make_pair(nodeID1,nodeID2)] = MinTravelTime;
        int roadID = map_nodeID_2_roadID[make_pair(nodeID1, nodeID2)];
        vRoad[roadID].travelTime = MinTravelTime;
    }

    for(int i = 0; i < vRoad.size(); i++)
    {
        if(vRoad[i].travelTime == -1)
        {
            int roadID2 = map_nodeID_2_roadID[make_pair(vRoad[i].ID2, vRoad[i].ID1)];
            vRoad[i].travelTime = vRoad[roadID2].travelTime;
            map_nodeID_2_minTime[make_pair(vRoad[i].ID1,vRoad[i].ID2)] = vRoad[roadID2].travelTime;
        }
    }
    /*
    // Step 4: 打印 map
    // -------------------------------------------------------
    map<pair<int,int>, int>::iterator itr;
    cout << "\nThe map NodeID2MinTravelTime is : \n";
    cout << "\tKEY\tELEMENT\n";
    for (itr = map_nodeID_2_minTime.begin(); itr != map_nodeID_2_minTime.end(); ++itr) {
        cout << '\t' << itr->first.first << '\t' << itr->first.second << '\t'
        << itr->second << '\n';
    }
    cout << endl;
    */
}


vector<vector<pair<int,int>>> Graph::alg1_w_records(vector<vector<int>> &Q, vector<vector<int>> &Pi){

    /*
     * Description: it's an extension of algorithm I, which records time each path passes through each point,
     * and the time at each edge when the traffic flow changes.
     *
     * Parameters:
     * &Q -> queries. contains queries and each query contains source node, destination node, and departure time.
     * &Pi -> paths. vertices passed by each query.
     *
     * Return:
     * GResult -> estimated travel time on each trajectory's road segment
     */

    // Step 1: 初始化变量
    // -------------------------------------------------------
    benchmark::heap<2, int, int> H(Q.size());

    // 初始化nodes_label，记录每个path经过每个点的时间
    nodes_label.clear();
    nodes_label.resize(Q.size());
    for (int i=0;i<nodes_label.size();i++){
        nodes_label[i].resize(Pi[i].size());
        for (int j=0;j<nodes_label[i].size();j++){
            nodes_label[i][j].resize(3);
        }
    }

    // 初始化estimated travel time
    vector<vector<pair<int,int>>> ETA_result(Pi.size());
    for (int i=0;i<Pi.size();i++){
        ETA_result[i].resize(Pi[i].size());
    }
    for (int i=0;i<Pi.size();i++){
        for (int j=0;j<1;j++){
            ETA_result[i][j].first = Pi[i][j];
            ETA_result[i][j].second = Q[i][2];
        }
        for (int k=1;k<Pi[i].size();k++){
            ETA_result[i][k].first = Pi[i][k];
            ETA_result[i][k].second = INF;
        }
    }

    // 初始化每个road segment的traffic flow
    vector<vector<pair<int,int>>> F = RoadNetwork;
    for (int i=0;i<F.size();i++){
        for (int j=0;j<F[i].size();j++){
            F[i][j].second = 0;
        }
    }

    // 初始化记录每个road segment的traffic flow number change时的time
    time_w_flow_change.clear();
    time_w_flow_change.resize(F.size());
    for (int i=0;i<time_w_flow_change.size();i++){
        time_w_flow_change[i].resize(F[i].size()); // num of nei
        for (int j=0;j<F[i].size();j++){
            time_w_flow_change[i][j].first = F[i][j].first;
        }
    }

    // 初始化path到达的node以及到达时间的label，并添加到优先队列H中
    vector<vector<int>> label(Q.size());
    int NodeIndex = 0;
    for (int i=0;i<Q.size();i++){
        label[i] = {i,NodeIndex,Q[i][2]};
        nodes_label[i][0] = label[i];
        H.update(i,label[i][2]); // query index, departure time
    }

    // Step 2: 模拟paths的出行顺序
    // -------------------------------------------------------
    int current_label_index, current_time, current_node_index;
    while (!H.empty()){
        H.extract_min(current_label_index, current_time);

        current_node_index = label[current_label_index][1];
        int current_node = Pi[current_label_index][current_node_index];

        // 定义latency function的参数
        float sigma = 0.15; float varphi = 20; float beta = 2;

        // Step 3: 如果current_node是该path的最后一个node
        // -------------------------------------------------------
        if (current_node_index == (Pi[current_label_index].size()-1)){

            int previous_node = Pi[current_label_index][current_node_index-1];

            for (int i=0;i<F[previous_node].size();i++){
                if(F[previous_node][i].first == current_node){
                    F[previous_node][i].second = F[previous_node][i].second - 1;
                    time_w_flow_change[previous_node][i].second.push_back({F[previous_node][i].second,label[current_label_index][2]});
                }
            }
            continue;
        }

        // Step 4: 如果current_node是该path的首个node
        // -------------------------------------------------------

        else{
            int next_node = Pi[current_label_index][current_node_index+1];
            if (current_node_index == 0){
                for (int i=0;i<F[current_node].size();i++){
                    if(F[current_node][i].first == next_node){
                        F[current_node][i].second = F[current_node][i].second + 1;
                        time_w_flow_change[current_node][i].second.push_back({F[current_node][i].second,label[current_label_index][2]});
                        // 根据traffic flow和latency function计算travel time
                        int tm = map_nodeID_2_minTime[make_pair(current_node, next_node)];
                        int Cflow = F[current_node][i].second;
                        int te = tm * (1 + sigma * pow(Cflow/varphi, beta));
                        // 更新label，travel time，nodes_label，以及优先队列
                        label[current_label_index][2] = label[current_label_index][2] + te;
                        label[current_label_index][1] = label[current_label_index][1] + 1;
                        ETA_result[current_label_index][current_node_index+1].second = label[current_label_index][2];
                        H.update(label[current_label_index][0], label[current_label_index][2]);
                        nodes_label[current_label_index][label[current_label_index][1]] = label[current_label_index];
                    }
                }
            }
            // Step 5: 如果current_node是该path的非首位node
            // -------------------------------------------------------
            else{
                int previous_node = Pi[current_label_index][current_node_index-1];

                for (int i=0;i<F[previous_node].size();i++){
                    if(F[previous_node][i].first == current_node){
                        F[previous_node][i].second = F[previous_node][i].second - 1;
                        time_w_flow_change[previous_node][i].second.push_back({F[previous_node][i].second,label[current_label_index][2]});
                    }
                }

                for (int i=0;i<F[current_node].size();i++){
                    if(F[current_node][i].first == next_node){
                        F[current_node][i].second = F[current_node][i].second + 1;
                        time_w_flow_change[current_node][i].second.push_back({F[current_node][i].second,label[current_label_index][2]});
                        // 根据traffic flow和latency function计算travel time
                        int tm = map_nodeID_2_minTime[make_pair(current_node, next_node)];
                        int Cflow = F[current_node][i].second;
                        int te = tm * (1 + sigma * pow(Cflow/varphi, beta));
                        // 更新label，travel time，nodes_label，以及优先队列
                        label[current_label_index][2] = label[current_label_index][2] + te;
                        label[current_label_index][1] = label[current_label_index][1] + 1;
                        ETA_result[current_label_index][current_node_index+1].second = label[current_label_index][2]; // ！！！得用这个
                        H.update(label[current_label_index][0], label[current_label_index][2]);
                        nodes_label[current_label_index][label[current_label_index][1]] = label[current_label_index];
                    }
                }
            }
        }
    }

    /*
    // Step 6: 打印nodes_label
    // -------------------------------------------------------
    for (int i=0;i<nodes_label.size();i++){
        cout << "path " << i << ": ";
        for (int j=0;j<nodes_label[i].size();j++){
            cout << nodes_label[i][j][1] << " " << nodes_label[i][j][2] << " ";
        }
        cout << endl;
    }
     */

    /*
    // Step 7: 打印time_w_flow_change
    // -------------------------------------------------------
    for (int i=0;i<time_w_flow_change.size();i++){
        cout << "node1: " << i << " ";
        for (int j=0;j<F[i].size();j++){
            cout << "node2 " << time_w_flow_change[i][j].first << " ";
            for (int k=0;k<time_w_flow_change[i][j].second.size();k++){
                cout << "flow: " << time_w_flow_change[i][j].second[k].first << " ";
                cout << "time: " << time_w_flow_change[i][j].second[k].second << " ";
            }
        }
        cout << endl;
    }
    */

    /*
    // Step 8: 打印ETA_result
    // -------------------------------------------------------
    for (int i=0;i<ETA_result.size();i++){
        cout << "path " << i << ": ";
        for (int j=0;j<ETA_result[i].size();j++){
            cout << "node " << ETA_result[i][j].first << "with ";
            cout << "time " << ETA_result[i][j].second << " " << endl;
        }
    }
    */

    // Step 8: return ETA_result
    // -------------------------------------------------------
    return ETA_result;
}


void Graph::nodes_label_2_edge_label(vector<vector<int>> &Pi){

    /*
     * Description: convert paths' nodes label into edge label -> {pathID, edgeID, time_in, time_out}
     *
     * Argument: &Pi -> trajectories. vertices passed by each query.
     */

    // Step 1: 设定edge_label的size
    // -------------------------------------------------------
    edge_label.clear();
    edge_label.resize(nodes_label.size());
    for (int i=0;i<nodes_label.size();i++){
        edge_label[i].resize(nodes_label[i].size()-1);// path
        for (int j=0;j<nodes_label[i].size()-1;j++){ //node
            edge_label[i][j].resize(4);
        }
    }

    // Step 2: 更新edge_label的内容
    // -------------------------------------------------------
    int edgeID, time_in, time_out, node_in, node_out;
    for (int i=0;i<nodes_label.size();i++){ // path
        for (int j=0;j<nodes_label[i].size()-1;j++){ //node
            node_in = Pi[i][nodes_label[i][j][1]]; node_out = Pi[i][nodes_label[i][j+1][1]];
            edgeID = map_nodeID_2_roadID[make_pair(node_in, node_out)];
            time_in = nodes_label[i][j][2]; time_out = nodes_label[i][j+1][2];
            edge_label[i][j][0] = nodes_label[i][j][0]; edge_label[i][j][1] = edgeID;
            edge_label[i][j][2] = time_in; edge_label[i][j][3] = time_out;
        }
    }

    /*
    // Step 3: 打印edge_label的内容
    // -------------------------------------------------------
    for (int i=0;i<edge_label.size();i++){
        cout << "path " << i << ": ";
        for (int j=0;j<edge_label[i].size();j++){
            cout << "edge " << edge_label[i][j][1] << " ";
            cout << "time in " << edge_label[i][j][2] << " ";
            cout << "time out " << edge_label[i][j][3] << " ";
        }
        cout << endl;
    }
    */
}


map<int,vector<vector<int>>> Graph::construct_inverted_table(vector<vector<vector<int>>> paths_w_roadID_time){

    /*
     * Description: This function create Inverted Table for paths.
     *
     * Argument: vector<vector<vector<int>>> paths_w_roadID_time ->
     * path: {{pathID, edge, time_in, time_out}, ..., {pathID, edge, time_in, time_out}}
     *
     * Return: map<int,vector<vector<int>>> -> edgeID, {{pathID, time_in, time_out}, ..., {pathID, time_in, time_out}}
     */

    // Step 1: 构建inverted table
    // -------------------------------------------------------
    map<int,vector<vector<int>>> inverted_table; inverted_table.clear();

    for (int i=0;i<paths_w_roadID_time.size();i++) {
        for (int j=0;j<paths_w_roadID_time[i].size();j++){
            int edge = paths_w_roadID_time[i][j][1];
            int time_in = paths_w_roadID_time[i][j][2];
            int time_out = paths_w_roadID_time[i][j][3];

            map<int,vector<vector<int>>>::iterator it;
            it = inverted_table.find(edge);
            // 如果edge已经在全体倒排记录表中
            if (it != inverted_table.end()) {
                it->second.push_back({i,time_in,time_out});//只将path及时间加入倒查表
            }
            else {// edge不在倒查表中
                pair<int, vector<vector<int>>> new_path(edge, {{i,time_in,time_out}});
                inverted_table.insert(new_path);//将edge，path及时间加入倒查表
            }
        }
    }
    cout << "Inverted Table Create Successful" << endl;

    // Step 2: return inverted table
    // -------------------------------------------------------
    return inverted_table;
}


void Graph::show_inverted_table(map<int, vector<vector<int>>> inverted_table){

    /*
     * Description: This function show values in Inverted Table.
     *
     * Parameters:
     * const map<int, set<int>> indexList -> variable which store paths.
     *
     * Return: void.
     */

    // Step 1: print inverted table
    // -------------------------------------------------------
    for (auto edge : inverted_table) {//显示倒排记录表
        cout << "edge " << edge.first << ": ";
        for (auto path_w_time : edge.second) {
            cout << "path " << path_w_time[0] << " ";
            cout << "time_in " << path_w_time[1] << " ";
            cout << "time_out " << path_w_time[2] << " ";
        }
        cout << endl;
    }

}


//void Graph::delete_content_inverted_table(map<int, vector<vector<int>>> inverted_table, int pathID){
//
//    /*
//     * Description: This function show values in Inverted Table.
//     *
//     * Parameters:
//     * const map<int, set<int>> indexList -> variable which store paths.
//     *
//     * Return: void.
//     */
//
//    // Step 1: print inverted table
//    // -------------------------------------------------------
//    for (auto edge : inverted_table) {//显示倒排记录表
//        for (auto path_w_time : edge.second) {
//            cout << "path " << path_w_time[0] << " ";
//            cout << "time_in " << path_w_time[1] << " ";
//            cout << "time_out " << path_w_time[2] << " ";
//            if (path_w_time[0] == pathID){
//                edge.second.erase(path_w_time);
//            }
//        }
//        cout << endl;
//    }
//}


vector<vector<int>> Graph::inverted_table_query(int edgeID, map<int,vector<vector<int>>> inverted_table) {

    /*
     * Description: This function query values in Inverted Table.
     *
     * Parameters:
     * int edge -> edgeID
     * map<int,vector<vector<int>>> inverted_table -> inverted table.
     *
     * Return:
     * vector<vector<int>> -> paths contains the target edgeID with drive in and out time.
     */

    // Step 1: go through inverted table for the target values.
    // -------------------------------------------------------
    // 判断edge是否在倒排表
    map<int, vector<vector<int>>>::iterator it = inverted_table.find(edgeID);
    vector<vector<int>> path_w_time; path_w_time.clear();
    if (it != inverted_table.end()) {//Yes
        /* cout << edgeID << ":"; */
        for (auto i: it->second) {//输出该词项的倒排记录表
            /*
            cout << "path " << i[0] << " ";
            cout << "time_in " << i[1] << " " << "time_out " << i[2] << " ";
            */
            path_w_time.push_back({i[0], i[1], i[2]});
        }
        /*
        cout << endl;
        */
    }
    else {//N0
        cout << edgeID << "edge does not appeared in inverted table" << endl;
    }

    /*
    // Step 2: print out path_w_time
    // -------------------------------------------------------
    for (int i=0;i<path_w_time.size();i++){
        cout << "edge: " << edgeID << " "; cout << "path " << path_w_time[i][0] << " ";
        cout << "time_in " << path_w_time[i][1] << " " << "time_out " << path_w_time[i][2] << " ";
    }
    */

    // Step 3: return path_w_time
    // -------------------------------------------------------
    return path_w_time;
}


int Graph::capture_flow_by_time(int time, int node1, int node2){

    /*
     * Description: capture flow on the road with [nodeID1, nodeID2] at a specific time.
     */

    // Step 1: find road segment in time_w_flow_change
    // -------------------------------------------------------

    int flow = 0; int count = 0;
    for (int i=0;i<time_w_flow_change[node1].size();i++){
        if(time_w_flow_change[node1][i].first == node2){
            count += 1;
            vector<pair<int, int>> time_w_flow = time_w_flow_change[node1][i].second;

            // Step 2: 判断time是否在记录时间范围内
            // -------------------------------------------------------

            for(int j=0;j<time_w_flow.size()-1;j++){
                if(time >= time_w_flow[j].second and time <= time_w_flow[j+1].second){
                    flow = time_w_flow[j].first; break;
                }
            }
        }
    }
    if (count == 0){
        cout << "Error, there is no this road segment. Please check the values of input nodes." << endl;
    }

    // Step 3: return flow
    // -------------------------------------------------------
    return flow;
}


pair<vector<vector<pair<int,int>>>, vector<pair<pair<int, int>,vector<int>>>> Graph::Alg1forAlg2Opt(vector<vector<int>> &Q,
                                                                                                    vector<pair<pair<int, int>, vector<int>>> &Pi,
                                                                                                    vector<vector<pair<int,int>>> exit_path_ETA_result,
                                                                                                    int node1, int node2){

    /*
     * Description:
     *
     * Arguments:
     *
     * Return:
     */

    // Step 1: 初始化
    // -------------------------------------------------------
    benchmark::heap<2, int, int> H(Q.size());

    // 初始化estimated travel time. Pi: <<pathID, position of shared edge>, nodes>
    vector<vector<pair<int,int>>> ETA_result(Pi.size());
    for (int i=0;i<Pi.size();i++){
        ETA_result[i].resize(Pi[i].second.size());
    }
    for (int i=0;i<Pi.size();i++){
        for (int j=0;j<1;j++){
            ETA_result[i][j].first = Pi[i].second[j];
            ETA_result[i][j].second = Q[i][2];
        }
        for (int k=1;k<Pi[i].second.size();k++){
            ETA_result[i][k].first = Pi[i].second[k];
            ETA_result[i][k].second = INF;
        }
    }

    // 初始化每个road segment的traffic flow
    vector<vector<pair<int,int>>> F = RoadNetwork;
    for (int i=0;i<F.size();i++){
        for (int j=0;j<F[i].size();j++){
            F[i][j].second = 0;
        }
    }

    // 初始化labels
    vector<vector<int>> label(Q.size());
    int NodeIndex = 0;
    for (int i=0;i<Pi.size();i++){
        label[i] = {i,NodeIndex,Q[i][2]};
        H.update(i,label[i][2]);
    }

    // 初始化迭代的label的ETA是否在公共边变化，1变化，0不变
    vector<int> label_bool(label.size());
    for (int i=0;i<label_bool.size();i++){
        label_bool[i] = -1;
    }

    // 定义需要返还的结果，前一部分是ETA，后面是会受影响的paths
    pair<vector<vector<pair<int,int>>>, vector<pair<pair<int, int>,vector<int>>>> ReturnValue;
    ReturnValue.first.clear(); ReturnValue.second.clear();

    // Step 2: 模拟paths的出行顺序
    // -------------------------------------------------------
    int current_label_index, current_time, current_node_index;
    int check_stop = 0; int stop_time = 0;
    while (!H.empty()){
        H.extract_min(current_label_index, current_time);

        current_node_index = label[current_label_index][1];
        int current_node = Pi[current_label_index].second[current_node_index];
        int next_node = Pi[current_label_index].second[current_node_index+1];

        // 定义latency fucntion的参数
        float sigma = 0.15; float varphi = 20; float beta = 2;

        // Step 3: 如果current_node是该path的最后一个node
        // -------------------------------------------------------
        if (current_node_index == (Pi[current_label_index].second.size()-1)){

            int previous_node = Pi[current_label_index].second[current_node_index-1];

            for (int i=0;i<F[previous_node].size();i++){
                if(F[previous_node][i].first == current_node){
                    F[previous_node][i].second = F[previous_node][i].second - 1;
                }
            }
            continue;
        }

            // Step 4: 如果current_node是该path的首个node
            // -------------------------------------------------------

        else{
            if (current_node_index == 0){
                for (int i=0;i<F[current_node].size();i++){
                    if(F[current_node][i].first == next_node){
                        F[current_node][i].second = F[current_node][i].second + 1;
                        // 根据traffic flow和latency function计算travel time
                        int tm = map_nodeID_2_minTime[make_pair(current_node, next_node)];
                        int Cflow = F[current_node][i].second;
                        int te = tm * (1 + sigma * pow(Cflow/varphi, beta));
                        // 更新label，travel time，nodes_label，以及优先队列
                        label[current_label_index][2] = label[current_label_index][2] + te;
                        label[current_label_index][1] = label[current_label_index][1] + 1;
                        ETA_result[current_label_index][current_node_index+1].second = label[current_label_index][2];
                        H.update(label[current_label_index][0], label[current_label_index][2]);
                    }
                }
            }

                // Step 5: 如果current_node是该path的非首位node
                // -------------------------------------------------------

            else{
                int previous_node = Pi[current_label_index].second[current_node_index-1];

                for (int i=0;i<F[previous_node].size();i++){
                    if(F[previous_node][i].first == current_node){
                        F[previous_node][i].second = F[previous_node][i].second - 1;
                    }
                }

                for (int i=0;i<F[current_node].size();i++){
                    if(F[current_node][i].first == next_node){
                        F[current_node][i].second = F[current_node][i].second + 1;
                        // 根据traffic flow和latency function计算travel time
                        int tm = map_nodeID_2_minTime[make_pair(current_node, next_node)];
                        int Cflow = F[current_node][i].second;
                        int te = tm * (1 + sigma * pow(Cflow/varphi, beta));
                        // 更新label，travel time，nodes_label，以及优先队列
                        label[current_label_index][2] = label[current_label_index][2] + te;
                        label[current_label_index][1] = label[current_label_index][1] + 1;
                        ETA_result[current_label_index][current_node_index+1].second = label[current_label_index][2]; // ！！！得用这个
                        H.update(label[current_label_index][0], label[current_label_index][2]);
                    }
                }
            }
        }

        // Step 6: 判断是否触发停止条件
        // -------------------------------------------------------

        // 如果停止条件未触发，判断迭代是否发生在公共边
        if (check_stop == 0){// 判断迭代是否发生在公共边
            if (current_node == node1 and next_node == node2){
                // 定义相关变量用于之后的比较
                int path_index = Pi[label[current_label_index][0]].first.first;
                int next_node_index = Pi[label[current_label_index][0]].first.second + 1;
                int time_on_next_node = exit_path_ETA_result[path_index][next_node_index].second;
                // 判断这个边的驶出时间是否变化
                int ETA_time_on_next_node = label[current_label_index][2];
                // 判断travel time是否改变，如果未改变，更改触发停止条件参数
                if (ETA_time_on_next_node != time_on_next_node){
                    label_bool[current_label_index] = 1;
                    ReturnValue.second.push_back(Pi[label[current_label_index][0]]);
                }
                else {
                    check_stop += 1;
                    stop_time = ETA_time_on_next_node;
                    label_bool[current_label_index] = 0;
                }
            }
        }
            // 如果停止条件触发
        else{
            if (current_time < stop_time){ // 如果在停止条件时间内
                if (current_node == node1 and next_node == node2){ // 并且label在公共边
                    int path_index = Pi[label[current_label_index][0]].first.first;
                    int next_node_index = Pi[label[current_label_index][0]].first.second + 1;
                    int time_on_next_node = exit_path_ETA_result[path_index][next_node_index].second;
                    int ETA_time_on_next_node = label[current_label_index][2];
                    // 判断travel time是否改变，如果改变，停止失败，继续迭代
                    if (ETA_time_on_next_node != time_on_next_node){
                        check_stop = 0; stop_time = 0; label_bool[current_label_index] = 1;
                        // 如果触发停止条件，时间范围内，驶入公共边的path仍改变，之前判断label=0的还会改变
                        for (int i=0;i<label_bool.size();i++){
                            if (label_bool[i] == 0){
                                label_bool[i] = 1;
                                ReturnValue.second.push_back(Pi[label[current_label_index][0]]);
                            }
                        }
                    }
                    else{
                        label_bool[current_label_index] = 0;
                    }
                }
            }

                // Step 7: 迭代受到影响的paths
                // -------------------------------------------------------

            else{ // 超出停止时间
                H.clear();
                for (int i=0;i<label.size();i++){
                    if (label_bool[i] == 1){
                        H.update(i,label[i][2]);
                    }
                }
            }
        }
    }

    // Step 8: 替换结果
    // -------------------------------------------------------

    for (int i=0;i<ETA_result.size();i++){
        int IndexInResult = Pi[i].first.first;
        int IndexInPosition = Pi[i].first.second;
        // 判断是否是受到影响的paths
        for (int j=0;j<ETA_result[i].size();j++){
            if (ETA_result[i][j].second == INF){
                break;
            }
        }
        for (int j=0;j<ETA_result[i].size();j++){
            exit_path_ETA_result[IndexInResult][j+IndexInPosition] = ETA_result[i][j];
        }
    }
    ReturnValue.first = ETA_result;

    // Step 9: 返还结果
    // -------------------------------------------------------
    return ReturnValue;
}


vector<vector<pair<int,int>>> Graph::alg2_opt(vector<int> new_path, vector<vector<int>> Pi,
                                              vector<int> new_query, vector<vector<int>> Q){

    // Step 1: 对老paths的初始处理，以及对其构建倒查表
    // -------------------------------------------------------

    // 构建从nodeID pair转换到roadID的map
    construct_map_nodeID_2_roadID(BJ_nodeRoadID);
    construct_map_roadID_2_nodeID(BJ_nodeRoadID);
    // 把由nodeID构成的path变成由roadID构成的path (path: node1 node2 node3 -> path: road1 road2)
    vector<vector<int>> path_roadID = path_nodeID_2_roadID(Pi);
    // 构建从nodeID pair到min travel time的map
    construct_map_nodeID_2_minTime(BJ_minTravleTime);

    std::chrono::high_resolution_clock::time_point t0_1, t0_2;
    std::chrono::duration<double> time_span;
    t0_1=std::chrono::high_resolution_clock::now();

    // 计算paths的travel time并记录paths经过每个点的时间以及flow 改变的时间
    vector<vector<pair<int,int>>> ETA_result = alg1_w_records(Q, Pi);

    t0_2=std::chrono::high_resolution_clock::now();
    time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t0_2-t0_1);
    cout << "time consumption for alg2 is: " << time_span.count() * 2 <<endl;

    /*
    // 打印ETA_result
    for (int i=0;i<ETA_result.size();i++){
        cout << "path " << i << ": ";
        for (int j=0;j<ETA_result[i].size();j++){
            int result = ETA_result[i][j].second;
            cout << result <<  " ";
        }
        cout << endl;
    }
    */

    // 把记录的paths经过每个node的时间 转换成 paths经过每个road的时间范围
    nodes_label_2_edge_label(Pi);
    // 通过index函数对PiRoadID构建倒查表
    map<int,vector<vector<int>>> inverted_table = construct_inverted_table(edge_label);
    /* g.show_inverted_table(inverted_table); // 打印倒查表 */

    // Step 2: 构建list_of_paths储存，处理new_path和受new_path影响的paths直至empty
    // -------------------------------------------------------

    std::chrono::high_resolution_clock::time_point t1_1, t1_2;
    std::chrono::duration<double> time_span1;
    t1_1=std::chrono::high_resolution_clock::now();

    vector<vector<int>> list_of_paths; list_of_paths.push_back(new_path);
    while (!list_of_paths.empty()){
        // 赋值list_of_paths中的最后一个path给new_path, 并从list_of_paths中移除
        new_path.clear(); new_path = list_of_paths.back(); list_of_paths.pop_back();

        // Step 3: 计算new_path的出行，及结束时间
        // -------------------------------------------------------

        // 把newPi由nodeID构成的path变成由roadID构成的path
        vector<int> new_path_w_roadID(new_path.size()-1);
        for (int i=0;i<new_path.size()-1;i++){
            new_path_w_roadID[i] = map_nodeID_2_roadID[make_pair(new_path[i],new_path[i+1])];
        }

        // 根据FlowNuminTime，以及latency function计算newPi的行驶时间
        int flow_temp, t_m; int new_path_travel_time = new_query[2]; vector<pair<int,int>> new_path_ETA;
        float sigma = 0.15; float varphi = 20; float beta = 2;
        new_path_ETA.resize(new_path.size()); new_path_ETA[0] = make_pair(new_path[0], new_path_travel_time);
        for (int i=0;i<new_path.size()-1;i++){
            flow_temp = capture_flow_by_time(new_path_travel_time, new_path[i], new_path[i+1]);
            t_m = map_nodeID_2_minTime[make_pair(new_path[i], new_path[i+1])];
            new_path_travel_time += t_m * (1 + sigma * pow(flow_temp/varphi, beta));
            new_path_ETA[i+1] = make_pair(new_path[i+1],new_path_travel_time);
        }

        /*
        // 打印new_path的ETA结果
        for (int i=0;i<new_path_ETA.size();i++){
            cout << "nodeID: " << new_path_ETA[i].first << " ETA: " << new_path_ETA[i].second << " ";
        }
        cout << endl;
        */

        // 定义new_path结束和开始行驶的时间
        int new_path_dest_time = new_path_ETA[new_path_ETA.size()-1].second;
        /* cout << "new_path_dest_time is: " << new_path_dest_time << endl; // 打印new_path的destination time */
        int new_path_depar_time = new_query[2];
        /* cout << "new_path_depar_time is: " << new_path_depar_time << endl; // 打印new_path的departure time */

        // Step 3: 找到和new_path一定不存在时间相交的paths
        // -------------------------------------------------------

        int exit_path_depar_time, exit_path_dest_time;
        // 对于new_path的每一个edge
        for (int i=0;i<new_path_w_roadID.size();i++){
            int shared_roadID = new_path_w_roadID[i];
            // 根据倒查表找到newPath每个edge空间直接相交的paths：edgeID -> {pathID, time_in, time_out}
            vector<vector<int>> exit_paths_w_time; exit_paths_w_time.clear();
            exit_paths_w_time = inverted_table_query(shared_roadID, inverted_table);
            // 如果没有空间相交的paths，跳过该edge到下一个
            if (exit_paths_w_time.size() == 0){
                continue;
            }
            int affected_path_size = exit_paths_w_time.size();

            // 提取new_path驶入公共边的时间，以及受影响的paths驶离公共边的时间
            int shared_node1 = map_roadID_2_nodeID[shared_roadID].first;
            int shared_node2 = map_roadID_2_nodeID[shared_roadID].second;
            int new_path_time_in_shared_road = new_path_ETA[shared_node1].second;

            // 对于每一个和new_path共享edge的existing path
            for (int j=0;j<exit_paths_w_time.size();j++){
                int exit_pathID = exit_paths_w_time[j][0];
                int exit_path_time_in = exit_paths_w_time[j][1];
                int exit_path_time_out = exit_paths_w_time[j][2];

                // 提取existing paths的出行时间用于比较
                exit_path_depar_time = Q[exit_pathID][2];
                exit_path_dest_time = ETA_result[exit_pathID].back().second;
                int a = 0;
                // 删除没有受到new_path影响的paths
                // new_path_depar_time > exit_path_dest_time or  new_path_time_in_shared_road > exit_path_time_in or new_path_dest_time < exit_path_depar_time
                if (new_path_time_in_shared_road > exit_path_dest_time or  new_path_time_in_shared_road > exit_path_time_in or new_path_dest_time < exit_path_time_in){
                    vector<vector<int>>::iterator it; it = exit_paths_w_time.begin()+j;
                    exit_paths_w_time.erase(it);
                }
            }

            // 打印可能受影响的paths总数，以及一轮filter下仍可能受影响的paths总数
            /* cout << "the number of affected by edge " << shared_roadID << " is: " << affected_path_size << endl; */
            /* cout << "the number of affected paths after filter is: " << exit_paths_w_time.size() << endl; */

            /*
            // 打印exit_paths_w_time中的pathID
            cout << "shared edge: " << shared_roadID << " ";
            for (int j=0;j<exit_paths_w_time.size();j++){
                cout << exit_paths_w_time[j][0] << " ";
            }
            cout << endl;
            */

            // Step 4: 根据filter后的paths生成需要迭代的filtered_paths和filtered_query
            // -------------------------------------------------------

            // 初始化需要迭代alg1的被filtered paths以及相应的queries: <<pathID, position of shared edge>, nodes>
            vector<pair<pair<int, int>, vector<int>>> filtered_paths(exit_paths_w_time.size()); int node1_posi;
            // 初始化filtered_query
            vector<vector<int>> filtered_query(exit_paths_w_time.size());
            for (int j=0;j<filtered_query.size();j++){
                filtered_query[j].resize(3);
            }

            // 给filtered_path和filtered_query赋值
            for (int j=0;j<exit_paths_w_time.size();j++){
                int exit_pathID = exit_paths_w_time[j][0];
                // 找到node1在当前path中的位置
                auto itr = find(Pi[exit_pathID].begin(), Pi[exit_pathID].end(), shared_node1);
                if (itr != Pi[exit_pathID].end()){
                    node1_posi = distance(Pi[exit_pathID].begin(),itr);
                }
                else{
                    cout << "error! current paths does not contain node1." << endl;
                }
                // 把从公共边开始的paths添加到filtered_paths中
                filtered_paths[j].first.first = exit_pathID;
                filtered_paths[j].first.second = node1_posi;
                filtered_paths[j].second.clear();
                for (int k=node1_posi;k<Pi[exit_pathID].size();k++) {
                    filtered_paths[j].second.push_back(Pi[exit_pathID][k]);
                }

                // 初始化剩余paths迭代Algorithm1的输入InputQ: <source node, destination node, departure time>
                filtered_query[j][0] = Pi[exit_pathID][node1_posi]; // source node
                filtered_query[j][1] = Pi[exit_pathID][Pi[exit_pathID].size()-1]; // destination node
                filtered_query[j][2] = ETA_result[exit_pathID][node1_posi].second; // departure time
            }

            /*
            // 打印从公共边开始的filtered_paths
            cout << "shared edge node1 is: " << shared_node1 << endl;
            for (int j=0;j<filtered_paths.size();j++){
                cout << "pathID is: " << filtered_paths[j].first.first << " ";
                for (int k=0;k<filtered_paths[j].second.size();k++){
                    cout << filtered_paths[j].second[k] << " ";
                }
                cout << endl;
            }
            */

            /*
            // 打印从公共边开始的filtered_query
            for (int j=0;j<filtered_query.size();j++){
                cout << "queryID: " <<  filtered_paths[j].first.first << " ";
                cout << "source " << filtered_query[j][0] << " ";
                cout << "desti " << filtered_query[j][1] << " ";
                cout << "depar " << filtered_query[j][2] << " " << endl;
            }
            */

            // Step 6: 判断剩余潜在受new_path影响的paths哪些受影响哪些不受影响
            // -------------------------------------------------------
            pair<vector<vector<pair<int,int>>>, vector<pair<pair<int, int>, vector<int>>>> ReturnValue;
            ReturnValue = Alg1forAlg2Opt(filtered_query, filtered_paths, ETA_result, shared_node1, shared_node2);
            vector<pair<pair<int, int>, vector<int>>> candidatePathsTemp = ReturnValue.second;
            for (int j=0;j<candidatePathsTemp.size();j++){
                list_of_paths.push_back(candidatePathsTemp[j].second);
            }
            // 从到查表中删除包含这些paths的 ！！！！
            vector<vector<pair<int,int>>> ETA_result = ReturnValue.first;
        }
    }

    t1_2=std::chrono::high_resolution_clock::now();
    time_span1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1_2-t1_1);
    cout << "time consumption for alg2 opt is: " << time_span1.count() <<endl;

    return ETA_result;
}

void Graph::BJ_nodeRoadID_w_new_roadID(string filename, string export_filename){

    /*
     * Description: construct map to transfer nodeID pair into road ID.
     * Arguments: string filename -> address of text file: nodeID, nodeID, roadID.
     */

    ifstream IF(filename);
    if(!IF){cout<<"Cannot open Map "<<filename<<endl;}

    ofstream outfile;
    outfile.open(export_filename);

    int node_num; IF >> node_num;
    int lines = count_lines(filename);

    outfile << node_num << " " << endl;

    int nodeID1, nodeID2, roadID;
    for (int i=0;i<lines-1;i++){
        IF >> nodeID1 >> nodeID2 >> roadID;
        outfile << nodeID1 << " " << nodeID2 << " " << i << " " << endl;
    }

    outfile.close();
}

