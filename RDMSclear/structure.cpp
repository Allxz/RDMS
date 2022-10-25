//
// Created by 徐子卓 on 7/22/22.
//

#include "head.h"

vector<vector<pair<int,int>>> Graph::alg1Records(vector<vector<int>> &Q, vector<vector<int>> &Pi){

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

    std::chrono::high_resolution_clock::time_point t0_1, t0_2;
    std::chrono::duration<double> time_span;
    t0_1=std::chrono::high_resolution_clock::now();

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
    timeFlowChange.clear();
    timeFlowChange.resize(F.size());
    for (int i=0;i<timeFlowChange.size();i++){
        timeFlowChange[i].resize(F[i].size()); // num of nei
        for (int j=0;j<F[i].size();j++){
            timeFlowChange[i][j].first = F[i][j].first;
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
    map<int,vector<int>> timeRecord;
    while (!H.empty()){
        H.extract_min(current_label_index, current_time);
        timeRecord.clear();

        current_node_index = label[current_label_index][1];
        int current_node = Pi[current_label_index][current_node_index];

        // 定义latency fucntion的参数
        // float sigma = 0.15; float varphi = 20; float beta = 2;

        // Step 3: 如果current_node是该path的最后一个node
        // -------------------------------------------------------
        if (current_node_index == (Pi[current_label_index].size()-1)){

            int previous_node = Pi[current_label_index][current_node_index-1];

            for (int i=0;i<F[previous_node].size();i++){
                if(F[previous_node][i].first == current_node){
                    F[previous_node][i].second = F[previous_node][i].second - 1;

//                    timeFlowChange[previous_node][i].second.insert(pair<int, vector<int>>(label[current_label_index][2],
//                                                                                          {current_label_index, 0, F[previous_node][i].second}));

                    // map<vector<vector>>
                    int record_time = label[current_label_index][2];

                    if (timeFlowChange[previous_node][i].second.find(record_time) == timeFlowChange[previous_node][i].second.end())
                    {
                        timeFlowChange[previous_node][i].second.insert(pair<int, vector<vector<int>>>(
                                label[current_label_index][2],{{current_label_index, 0, F[previous_node][i].second}}));
                    }
                    else
                    {
                        timeFlowChange[previous_node][i].second[record_time].push_back({current_label_index, 0, F[previous_node][i].second});
                    }

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


//                        timeFlowChange[current_node][i].second.insert(pair<int,vector<int>>(label[current_label_index][2],
//                                {current_label_index, 1, F[current_node][i].second}));

                        // map<vector<vector>>
                        int record_time = label[current_label_index][2];

                        if (timeFlowChange[current_node][i].second.find(record_time) == timeFlowChange[current_node][i].second.end())
                        {
                            timeFlowChange[current_node][i].second.insert(pair<int, vector<vector<int>>>(
                                    label[current_label_index][2],{{current_label_index, 1, F[current_node][i].second}}));
                        }
                        else
                        {
                            timeFlowChange[current_node][i].second[record_time].push_back({current_label_index, 1, F[current_node][i].second});
                        }

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

//                        timeFlowChange[previous_node][i].second.insert(pair<int, vector<int>>(label[current_label_index][2],
//                                {current_label_index, 0, F[previous_node][i].second}));

                        // map<vector<vector>>
                        int record_time = label[current_label_index][2];

                        if (timeFlowChange[previous_node][i].second.find(record_time) == timeFlowChange[previous_node][i].second.end())
                        {
                            timeFlowChange[previous_node][i].second.insert(pair<int, vector<vector<int>>>(
                                    label[current_label_index][2],{{current_label_index, 0, F[previous_node][i].second}}));
                        }
                        else
                        {
                            timeFlowChange[previous_node][i].second[record_time].push_back({current_label_index, 0, F[previous_node][i].second});
                        }

                    }
                }

                for (int i=0;i<F[current_node].size();i++){
                    if(F[current_node][i].first == next_node){
                        F[current_node][i].second = F[current_node][i].second + 1;

//                        timeFlowChange[current_node][i].second.insert(pair<int, vector<int>>(label[current_label_index][2],
//                                {current_label_index, 1, F[current_node][i].second}));

                        // map<vector<vector>>
                        int record_time = label[current_label_index][2];

                        if (timeFlowChange[current_node][i].second.find(record_time) == timeFlowChange[current_node][i].second.end())
                        {
                            timeFlowChange[current_node][i].second.insert(pair<int, vector<vector<int>>>(
                                    label[current_label_index][2],{{current_label_index, 1, F[current_node][i].second}}));
                        }
                        else
                        {
                            timeFlowChange[current_node][i].second[record_time].push_back({current_label_index, 1, F[current_node][i].second});
                        }

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
    // Step 7: 打印timeFlowChange
    // -------------------------------------------------------
    for (int i=0;i<timeFlowChange.size();i++){
        cout << "node1: " << i << endl;
        for (int j=0;j<F[i].size();j++){

            cout << " node2: " << timeFlowChange[i][j].first;

            cout << " with size: " << timeFlowChange[i][j].second.size() << " ";

//            map<int, vector<int>>::iterator itr;
//            for (itr = timeFlowChange[i][j].second.begin(); itr != timeFlowChange[i][j].second.end(); ++itr) {
//                cout << " time " << itr->first << " routeID " << itr->second[0];
//                cout << " status " << itr->second[1] << " flow " << itr->second[2] << "||";
//            }

            // map<vector<vector>>
            map<int, vector<vector<int>>>::iterator itr;
            for (itr = timeFlowChange[i][j].second.begin(); itr != timeFlowChange[i][j].second.end(); ++itr) {
                for (int i=0;i<itr->second.size();i++)
                {
                    cout << " time " << itr->first << " routeID " << itr->second[i][0];
                    cout << " status " << itr->second[i][1] << " flow " << itr->second[i][2] << "||";
                }
            }

            cout << "\n" << endl;
        }
        cout << "\n" << endl;
    }
    */

    /*
    // Step 8: 打印ETA_result
    // -------------------------------------------------------
    for (int i=0;i<ETA_result.size();i++){
        cout << "path " << i << ": ";
        for (int j=0;j<ETA_result[i].size();j++){
            cout << "node " << ETA_result[i][j].first << " with ";
            cout << "time " << ETA_result[i][j].second << " ";
        }
        cout << endl;
    }
    */

    cout <<  "algorithm I simulation done" << endl;

    t0_2=std::chrono::high_resolution_clock::now();
    time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t0_2-t0_1);
    cout << "Algorithm 1 simulation time consumption is: " << time_span.count() <<endl;

    // Step 8: return ETA_result
    // -------------------------------------------------------
    return ETA_result;
}

int Graph::global_evaluation(vector<vector<pair<int,int>>> &ETA_result){

    std::chrono::high_resolution_clock::time_point t0_1, t0_2;
    std::chrono::duration<double> time_span;
    t0_1=std::chrono::high_resolution_clock::now();

    int globalTravel = 0;
    int beginTime, endTime, pathSize;
    for (int i=0;i<ETA_result.size();i++){
        pathSize = ETA_result[i].size();
        beginTime = ETA_result[i][0].second;
        endTime = ETA_result[i][pathSize-1].second;
        globalTravel += endTime - beginTime;
    }

    /* cout << "Global Travel Time is: " << globalTravel << endl; */

    t0_2=std::chrono::high_resolution_clock::now();
    time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t0_2-t0_1);
    cout << "global evaluation time consumption is: " << time_span.count() <<endl;
}


void Graph::Route_timeFlowChange(vector<vector<pair<int,map<int, vector<vector<int>>>>>> &timeFlowChange){

    /*
     * Description: Change NodeID into RoadSegmentID.
     *
     * Parameters:
     * timeFlowChange -> time records follow two NodeID.
     *
     * Return:
     * route_timeFlowChange (declared in .h file) -> time records follow RoadSegmentID.
     */

    route_timeFlowChange.resize(map_nodeID_2_roadID.size());
    int node01, node02, routeID;
    for (int i=0;i<timeFlowChange.size();i++){
        node01 = i;
        for (int j=0;j<timeFlowChange[i].size();j++){
            node02 = timeFlowChange[i][j].first;
            routeID = map_nodeID_2_roadID[make_pair(node01,node02)];
            route_timeFlowChange[routeID] = timeFlowChange[i][j].second;
        }
    }

    /*
    // Step 2: Print Result Out
    // -------------------------------------------------------
    for (int i=0;i<route_timeFlowChange.size();i++){

        cout << "RoadSegmentID: " << i << " size " << route_timeFlowChange[i].size() << endl;

        map<int, vector<int>>::iterator itr;
        for (itr = route_timeFlowChange[i].begin(); itr != route_timeFlowChange[i].end(); ++itr){
            cout << " time " << itr->first << " routeID " << itr->second[0];
            cout << " status " << itr->second[1] << " flow " << itr->second[2] << "||";
        }
        cout << "\n" << endl;

    }
    */

    cout << "N2R Done." << endl;

}

int Graph::Time2Hour(int intTime){

    /*
     * Description: convert int time into its UTC hour.
     * Parameters: intTime -> int time.
     * Return: hour -> UTC hour.
     */

    time_t time = intTime;
    tm *ltm = localtime(&time);
    int hour = ltm->tm_hour - 8;
    return hour;
}

void Graph::min_depar_time(vector<vector<int>> &Q){

    /*
     * Description: find min departure time from queries.
     * Parameters: Q -> queries.
     * Return: minDeparture (.h) -> min departure time. minHour (.h) -> min departure time's hour.
     */

    vector<int> times;
    for (int i = 0; i < Q.size(); i++) {
        times.push_back(Q[i][2]);
    }

    minDeparture = *min_element(times.begin(), times.end());
    minHour = Time2Hour(minDeparture);

    cout << "min time is: " << minDeparture << " in " << minHour << " hour." << endl;
}

int Graph::Hour2Index(int hour){

    /*
     * Description: find belonged index by time's hour
     * Parameters: hour -> time's hour.
     * Return: index -> hour belonged index.
     */

    int index = hour - minHour;

    if (index >= 0){
        return index;
    }else{
        index += 24;
        return index;
    }
}

void Graph::RouteSlice_timeFlowChange(vector<map<int, vector<vector<int>>>> &route_timeFlowChange){

    /*
     * Description: Put time records into time slices.
     *
     * Parameters: route_timeFlowChange -> time records follow RoadSegmentID.
     *
     * Return: TimeRecordInSlice: map<int, map<int, map<int, vector<int>>>> - RoadSegmentID: TimeSlice: TimeRecord
     */

    // Step 1: Define First Time Record
    // -------------------------------------------------------
    TimeRecordInSlice.resize(route_timeFlowChange.size());
    for (int i =0; i < TimeRecordInSlice.size();i++){
        TimeRecordInSlice[i].resize(24);
    }
    for (int i = 0; i < route_timeFlowChange.size();i++){
        int RoadSegmentID = i;
        map<int, vector<vector<int>>>::iterator itr;
        for (itr = route_timeFlowChange[i].begin(); itr != route_timeFlowChange[i].end(); ++itr){
            int time = itr->first;
            int hour = Time2Hour(time);
            int index = Hour2Index(hour);
            TimeRecordInSlice[RoadSegmentID][index].insert(pair<int, vector<vector<int>>>(itr->first,itr->second));
        }
    }

    /*
    // Step 2: Print Result Out
    // -------------------------------------------------------
    for (int i=0;i<TimeRecordInSlice.size();i++){
        for (int j=0;j<TimeRecordInSlice[i].size();j++){
            if (TimeRecordInSlice[i][j].size() != 0){
                if (i == 222286)
                {
                    cout << "RoadSegmentID: " << i;
                    cout << " TimeSlice " << j << ":";
                    map<int, vector<vector<int>>>::iterator itr;
                    for (itr = TimeRecordInSlice[i][j].begin(); itr != TimeRecordInSlice[i][j].end(); ++itr){
                        for (int i=0;i<itr->second.size();i++)
                        {
                            cout << " time " << itr->first << " routeID " << itr->second[i][0];
                            cout << " status " << itr->second[i][1] << " flow " << itr->second[i][2] << "||";
                        }
                    }
                    cout << "\n" << endl;
                }

            }
        }
    }
    */

    cout << "Time Slice Done." << endl;

}

int Graph::IndexFlow(vector<vector<map<int, vector<vector<int>>>>> &TimeRecordInSlice, int timeValue, int RoadSegmentID, vector<int> features){

    /*
     * Description: Index Traffic Flow with RoadSegmentID and Time.
     *
     * Parameters:
     * TimeRecordInSlice -> time records in time slice.
     * timeValue -> search time.
     * RoadSegmentID -> search RoadSegmentID.
     * features -> {0,0,0} for position without true meaning.
     *
     * Return: TimeRecordInSlice: map<int, map<int, map<int, vector<int>>>> - RoadSegmentID: TimeSlice: TimeRecord
     */


    // Step 1: Find Time Records
    // -------------------------------------------
    int hour = Time2Hour(timeValue);
    int index = Hour2Index(hour);
    map<int, vector<vector<int>>> timeRecords = TimeRecordInSlice[RoadSegmentID][index];


    // Print Time Records
    // -------------------------------------------
    /*
    map<int, vector<vector<int>>>::iterator itPrint;
    cout << "-----------------------------------------------------------------" << endl;
    for (itPrint = timeRecords.begin(); itPrint != timeRecords.end(); ++itPrint){
        for (int i=0;i<itPrint->second.size();i++)
        {
            cout << " time " << itPrint->first << " routeID " << itPrint->second[i][0];
            cout << " status " << itPrint->second[i][1] << " flow " << itPrint->second[i][2] << "||";
        }
    }
    */


    // Step 2: Find Time Value or Insert
    // -------------------------------------------
    int preFlow;

    map<int, vector<vector<int>>>::iterator itFind;
    itFind = timeRecords.find(timeValue);

    if (itFind == timeRecords.end()){ // not find
        timeRecords.insert(pair<int, vector<vector<int>>>(timeValue, {features}));

        // Find Previous Value
        // -------------------------------------------
        map<int, vector<vector<int>>>::iterator itPre;
        itPre = timeRecords.find(timeValue);
        if (itPre != timeRecords.end()){ // find
            if (itPre == timeRecords.begin()){
                preFlow = 0;
                /*
                cout << "time is the first one with flow : " << preFlow << endl;
                */
            } else{
                itPre = --itPre;
                preFlow = itPre->second.back()[2];
                /*
                cout << "previous time: " << itPre->first << " flow: " << preFlow << endl;
                */
            }
        } else{
            cout << "No value found. Please check Insertion." << endl;
        }

        // Erase Inserted Value
        // -------------------------------------------
        map<int, vector<vector<int>>>::iterator itErase;
        itErase = timeRecords.find(timeValue);
        if (itErase != timeRecords.end()){
            timeRecords.erase(itErase);
        }
    } else{ // find
        preFlow = itFind->second.back()[2];
        cout << "time record has already existed." << endl;
        cout << "previous key: " << itFind->first << " value: " << preFlow << endl;
    }

    // Step 3: Return Result
    // -------------------------------------------
    return preFlow;
}

void Graph::TimeRecordCheck(vector<vector<map<int, vector<vector<int>>>>> &TimeRecordInSlice){

    /*
     * Description: Check Correctness of Time Records
     *
     * Parameters:
     * TimeRecordInSlice -> time records in time slice.
     *
     * Return: TimeRecordInSlice: Checked TimeRecord Slices.
     */

    int TimeRecordCountBefore = 0;
    int TimeRecordCountAfter = 0;

    // Count Number of Time Records Before Deletion
    for (int i=0; i<TimeRecordInSlice.size(); i++)
    {
        for (int j = 0; j < TimeRecordInSlice[i].size(); j++)
        {
            map<int, vector<vector<int>>>::iterator itr_count;
            for (itr_count = TimeRecordInSlice[i][j].begin(); itr_count != TimeRecordInSlice[i][j].end(); ++itr_count)
            {
                TimeRecordCountBefore += itr_count->second.size();
            }
        }
    }

    // Step 1: Clear Incorrect Time Records
    // -------------------------------------------
    for (int i=0; i<TimeRecordInSlice.size(); i++) {
        for (int j=0;j<TimeRecordInSlice[i].size();j++){

            if (TimeRecordInSlice[i][j].size() != 0){

                // 如果第一个TimeRecord的flow不是1，清除
                map<int, vector<vector<int>>>::iterator itrBegin;
                itrBegin = TimeRecordInSlice[i][j].begin();
                if (itrBegin->second[0][2] != 1){
                    // clear
                    TimeRecordInSlice[i][j].clear();
                    continue;
                }

                // 如果最后一个TimeRecord的flow不是0，清除
                map<int, vector<vector<int>>>::iterator itrEnd;
                itrEnd = --TimeRecordInSlice[i][j].end();
                if (itrEnd->second.back()[2] != 0){
                    // erase
                    TimeRecordInSlice[i][j].clear();
                    continue;
                }

                // 如果TimeRecords不是递增或者递减，清除
                map<int, vector<vector<int>>>::iterator itrCurrent;
                for (itrCurrent = TimeRecordInSlice[i][j].begin(); itrCurrent != --TimeRecordInSlice[i][j].end(); ++itrCurrent)
                {
                    if (itrCurrent->second.size() == 1)
                    {
                        int CFlow = itrCurrent->second[0][2];
                        ++itrCurrent;
                        int NFlow = itrCurrent->second[0][2];
                        int FlowChange = abs(CFlow - NFlow);
                        --itrCurrent;
                        if (FlowChange != 1){
                            TimeRecordInSlice[i][j].clear();
                            break;
                        }
                    }
                    else
                    {
                        for (int i=0;i<itrCurrent->second.size()-1;i++)
                        {
                            int CFlow = itrCurrent->second[i][2];
                            int NFlow = itrCurrent->second[i+1][2];
                            int FlowChange = abs(CFlow - NFlow);
                            if (FlowChange != 1){
                                TimeRecordInSlice[i][j].clear();
                                break;
                            }
                        }

                        int CFlow = itrCurrent->second.back()[2];
                        ++itrCurrent;
                        int NFlow = itrCurrent->second[0][2];
                        int FlowChange = abs(CFlow - NFlow);
                        --itrCurrent;
                        if (FlowChange != 1){
                            TimeRecordInSlice[i][j].clear();
                            break;
                        }
                    }

                }

            }

        }
    }

    // Count Number of Time Records After Deletion
    for (int i=0; i<TimeRecordInSlice.size(); i++)
    {
        for (int j = 0; j < TimeRecordInSlice[i].size(); j++)
        {
            map<int, vector<vector<int>>>::iterator itr_count;
            for (itr_count = TimeRecordInSlice[i][j].begin(); itr_count != TimeRecordInSlice[i][j].end(); ++itr_count)
            {
                TimeRecordCountAfter += itr_count->second.size();
            }
        }
    }

    /*
    // Step 2: Print Result
    // -------------------------------------------
    for (int i=0;i<TimeRecordInSlice.size();i++){
        for (int j=0;j<TimeRecordInSlice[i].size();j++){
            if (TimeRecordInSlice[i][j].size() != 0){
                cout << "RoadSegmentID: " << i;
                cout << " TimeSlice " << j << ":";
                map<int, vector<vector<int>>>::iterator itr;
                for (itr = TimeRecordInSlice[i][j].begin(); itr != TimeRecordInSlice[i][j].end(); ++itr){
                    for (int k=0;k<itr->second.size();k++)
                    {
                        cout << " time " << itr->first << " routeID " << itr->second[k][0];
                        cout << " status " << itr->second[k][1] << " flow " << itr->second[k][2] << "||";
                    }
                }
                cout << "\n" << endl;
            }
        }
    }
    */


    // Step 3: Check Deleted Number
    // -------------------------------------------
    cout << "TimeRecords before deletion is: " << TimeRecordCountBefore << endl;
    cout << "TimeRecords after deletion is: " << TimeRecordCountAfter << endl;


    timeRecordsChecked = TimeRecordInSlice;

    cout << "Correctness Check Done." << endl;
}

int Graph::timeRecords_correct_check(map<int, vector<vector<int>>> timeRecords){

    int error_check = 0;

    if (timeRecords.size() != 0){

        map<int, vector<vector<int>>>::iterator itrBegin;
        itrBegin = timeRecords.begin();
        if (itrBegin->second[0][2] != 1){
/*
            cout << "Error! Traffic flow of the only one time record is not correct." << endl;
*/
            error_check += 1;
        }

        map<int, vector<vector<int>>>::iterator itrEnd;
        itrEnd = --timeRecords.end();
        if (itrEnd->second.back()[2] != 0){
/*
            cout << "Error! Traffic flow of the last time record is not 0." << endl;
*/
            error_check += 1;
        }

        map<int, vector<vector<int>>>::iterator itrCurrent;
        for (itrCurrent = timeRecords.begin(); itrCurrent != --timeRecords.end(); ++itrCurrent)
        {
            if (itrCurrent->second.size() == 1)
            {
                int CFlow = itrCurrent->second[0][2];
                ++itrCurrent;
                int NFlow = itrCurrent->second[0][2];
                int FlowChange = abs(CFlow - NFlow);
                --itrCurrent;
                if (FlowChange != 1){
/*
                    cout << "Error! Traffic flow of time records is not continuous." << endl;
*/
                    error_check += 1;
                }
            }
            else
            {
                for (int i=0;i<itrCurrent->second.size()-1;i++)
                {
                    int CFlow = itrCurrent->second[i][2];
                    int NFlow = itrCurrent->second[i+1][2];
                    int FlowChange = abs(CFlow - NFlow);
                    if (FlowChange != 1){
/*
                        cout << "Error! Traffic flow of time records is not continuous." << endl;
*/
                        error_check += 1;
                    }
                }

                int CFlow = itrCurrent->second.back()[2];
                ++itrCurrent;
                int NFlow = itrCurrent->second[0][2];
                int FlowChange = abs(CFlow - NFlow);
                --itrCurrent;
                if (FlowChange != 1){
/*
                    cout << "Error! Traffic flow of time records is not continuous." << endl;
*/
                    error_check += 1;
                }
            }

        }

    }

    return error_check;
}

vector<int> Graph::route_node2roadSegment(vector<int> route_node){

    vector<int> route_roadSegment;
    route_roadSegment.resize(route_node.size()-1);
    int node01, node02, roadSegmentID, routeID;

    for (int i=0;i<route_roadSegment.size();i++){
        node01 = route_node[i]; node02 = route_node[i+1];
        roadSegmentID = map_nodeID_2_roadID[make_pair(node01,node02)];
        route_roadSegment[i] = roadSegmentID;
    }

    /*
    // Print route_roadSegment
    cout << "route with roadSegment ID is: ";
    for (int i=0;i<route_roadSegment.size();i++){
        cout << route_roadSegment[i] << " ";
    }
    cout << endl;
    */

    return route_roadSegment;
}

void Graph::route_N2R(vector<vector<int>> &Pi){

    PiRoadSegmentID.resize(Pi.size());
    int node01, node02, roadSegmentID, routeID;
    for (int i=0;i<PiRoadSegmentID.size();i++){
        if (Pi[i].size() <= 1){
            int a = Pi[i].size();
            continue;
        }
        routeID = i; PiRoadSegmentID[i].resize(Pi[i].size()-1);
        for (int j=0;j<PiRoadSegmentID[i].size();j++){
            node01 = Pi[i][j]; node02 = Pi[i][j+1];
            roadSegmentID = map_nodeID_2_roadID[make_pair(node01,node02)];
            PiRoadSegmentID[i][j] = roadSegmentID;
        }
    }

    /*
    for (int i=0;i<PiRoadSegmentID.size();i++){
        cout << "Route ID: " << i << endl;
        for (int j=0;j<PiRoadSegmentID[i].size();j++){
            cout << "Road Segment ID: " << PiRoadSegmentID[i][j] << " ";
        }
        cout << endl;
    }
    */
}

int Graph::findNextRoadSegmentID(vector<int> route, int roadSegmentID){

    int nextRoadSegmentID;
    std::vector<int>::iterator itFind;

    itFind = std::find (route.begin(), route.end(), roadSegmentID);
    if (itFind != route.end() and itFind != --route.end()){
        ++itFind; nextRoadSegmentID = *itFind;
    }else{
        nextRoadSegmentID = -1;
        // cout << "Cannot Find The Next RoadSegmentID or It's The Last One." << endl;
    }

    return nextRoadSegmentID;
}


vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> Graph::updateOperation1st(pair<int,int> RoadSegmentID, int inTime, pair<int, vector<int>> newRoutePair) {

    // Step 1: Initialization
    // -------------------------------------------
    int TrafficFlow = 0; int TrafficFlowInsert = 0;

    int TerminalCondition = 0; int RecordTime = 0;

    map<int, vector<vector<int>>> Insert; vector<int> Deletion;
    Insert.insert(pair<int, vector<vector<int>>>(0, {{0, 0, 0}}));

    map<int, vector<vector<int>>> InsertNext; vector<int> DeletionNext;

    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> Null;

    int newRouteID = newRoutePair.first; vector<int> newRoute = newRoutePair.second;

    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> roadSegmentAffected;
    vector<int> roadSegmentAffectedID; int NewTimeLeave;

    // Step 2: Insertion Operation
    // -------------------------------------------

    // Step 2.1: Target Time Records
    // -------------------------------------------
    int hour = Time2Hour(inTime); int index = Hour2Index(hour);
    map<int, vector<vector<int>>> timeRecords = timeRecordsChecked[RoadSegmentID.first][index];

    // Step 2.2: Find Time Value or Insert
    // -------------------------------------------
    int preFlow; vector<int> features = {newRouteID, 1, 0};
    map<int, vector<vector<int>>>::iterator itFind; itFind = timeRecords.find(inTime);

    if (itFind == timeRecords.end()) { // not find

        timeRecords.insert(pair<int, vector<vector<int>>>(inTime, {features}));

        // Step 2.3: Find Previous Value for Traffic Flow
        // -------------------------------------------
        map<int, vector<vector<int>>>::iterator itPre;
        itPre = timeRecords.find(inTime);
        if (itPre != timeRecords.end()) { // find
            if (itPre == timeRecords.begin()) {
                preFlow = 0;
                /*
                cout << "time is the first one with flow : " << preFlow << endl;
                */
            } else {
                itPre = --itPre;
                preFlow = itPre->second.back()[2];
                /*
                cout << "previous time: " << itPre->first << " flow: " << preFlow << endl;
                */
            }
        } else {
            cout << "No value found. Please check Insertion." << endl;
            return Null;
        }
    } else { // find
        preFlow = itFind->second.back()[2];
        timeRecords[inTime].push_back(features);
    }

    // Step 2.4: Estimate New Time Record's Travel Time
    // -------------------------------------------

    TrafficFlow = preFlow; TrafficFlowInsert = preFlow + 1;

    // Update Traffic Flow of Driving In Time Record
    tuple<int, map<int, vector<vector<int>>>, vector<int>> tempReturn;
    map<int, vector<vector<int>>>::iterator itFind1;

    int nextRoadSegmentID, routeID;
    nextRoadSegmentID = findNextRoadSegmentID(newRoutePair.second, RoadSegmentID.first);

    itFind1 = timeRecords.find(inTime);
    itFind1->second.back()[2] = TrafficFlowInsert;

    // Estimate New Time Record's Leaving Time
    int current_node = map_roadID_2_nodeID[RoadSegmentID.first].first; int next_node = map_roadID_2_nodeID[RoadSegmentID.first].second;
    int tm = map_nodeID_2_minTime[make_pair(current_node, next_node)]; int te = tm * (1 + sigma * pow(TrafficFlowInsert / varphi, beta));

    if (tm < 1)
    {
        return Null;
    }
    NewTimeLeave = inTime + te;

    // Put New Time Record's Leaving Time Record in Insert Set
    if (Insert.find(NewTimeLeave) == Insert.end()){
        Insert.insert(pair<int, vector<vector<int>>>((NewTimeLeave), {{newRouteID, 0, 0}}));
    }
    else
    {
        Insert[NewTimeLeave].push_back({newRouteID, 0, 0});
    }


    // Step 3: Update Operation
    // -------------------------------------------

    // Step 3.1: Check New Time Record is Last One
    // -------------------------------------------
    map<int, vector<vector<int>>>::iterator itrInser; itrInser = timeRecords.find(inTime);
    if (itrInser == timeRecords.end()) {
        cout << "Error. Time Records Do Not Contain Others Except the New Driving In One" << endl;
    }

    // Step 3.2: For Each Time Record Start From New Next (Core Design).
    // -------------------------------------------
    int timeCurrent, RoadIDCurrent;

    map<int, vector<vector<int>>>::iterator itr;

    for (itr = ++itrInser; itr != timeRecords.end(); ++itr) {
        for (int i = 0; i < itr->second.size(); i++) {
            timeCurrent = itr->first;
            RoadIDCurrent = itr->second[i][0];

            // Step 3.3: Compare Time Records in Insert Set
            // -------------------------------------------
            int RouteIDI;

            map<int, vector<vector<int>>>::iterator itrI;

            for (itrI = ++Insert.begin(); itrI != Insert.end(); ++itrI) {
                int timeI = itrI->first;
                if (timeI < timeCurrent) {
                    TrafficFlowInsert -= 1;

                    RouteIDI = itrI->second[i][0];

                    if (itrI->second[i][1] == 1) { // Status In.
                        if (timeRecords.find(timeI) == timeRecords.end()) {
                            timeRecords.insert(
                                    pair<int, vector<vector<int>>>(timeI, {{RouteIDI, 1, TrafficFlowInsert}}));
                        } else {
                            timeRecords[timeI].push_back({RouteIDI, 1, TrafficFlowInsert});
                        }
                    } else { // Status Out.
                        if (timeRecords.find(timeI) == timeRecords.end()) {
                            timeRecords.insert(
                                    pair<int, vector<vector<int>>>(timeI, {{RouteIDI, 0, TrafficFlowInsert}}));
                        } else {
                            timeRecords[timeI].push_back({RouteIDI, 0, TrafficFlowInsert});
                        }
                    }

                    itrI = Insert.erase(itrI);
                    itrI = --itrI;
                }
            }

            // Step 3.4: If Time Record is Driving In Status.
            // -------------------------------------------
            map<int, vector<vector<int>>>::iterator itrIPre;
            int RouteIDIPre, teCurrent;
            if (itr->second[i][1] == 1) {
                if (TerminalCondition == 1) {
                    if (itr == timeRecords.end()) {
                        break;
                    }

                    if (timeCurrent <= RecordTime) {
                        if (TrafficFlow != TrafficFlowInsert) {
                            TerminalCondition = 0;
                            TrafficFlow += 1;
                            TrafficFlowInsert += 1;

                            itr->second[i][2] = TrafficFlowInsert;
                            routeID = itr->second[i][0];

                            teCurrent = tm * (1 + sigma * pow(TrafficFlowInsert / varphi, beta));

                            if (Insert.find(timeCurrent + teCurrent) == Insert.end()) {
                                Insert.insert(pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                             {{RoadIDCurrent, 0, 0}}));
                            } else {
                                Insert[timeCurrent + teCurrent].push_back({RoadIDCurrent, 0, 0});
                            }

                            Deletion.push_back(routeID);

                            vector<int> affected_route = Pi[routeID];
                            vector<int> affected_route_w_roadSegmentID = route_node2roadSegment(affected_route);
                            int unaffected_next_roadSegmentID = findNextRoadSegmentID(affected_route_w_roadSegmentID,
                                                                                      RoadSegmentID.first);

                            // 查找下一个roadSegment是不是newRoute的
                            std::vector<int>::iterator itrFindNew;
                            itrFindNew = std::find(newRoute.begin(), newRoute.end(), unaffected_next_roadSegmentID);

                            // 查找下一个roadSegment是否已经存在于roadSegmentAffected中
                            roadSegmentAffectedID.clear();
                            for (int i = 0; i < roadSegmentAffected.size(); i++) {
                                int tempRoadSegmentID = roadSegmentAffected[i].first.first;
                                roadSegmentAffectedID.push_back(tempRoadSegmentID);
                            }

                            std::vector<int>::iterator itFindExit;
                            itFindExit = std::find(roadSegmentAffectedID.begin(),
                                                   roadSegmentAffectedID.end(), unaffected_next_roadSegmentID);

                            if (itFindExit != roadSegmentAffectedID.end()) { // 已经添加
                                // int position = itFindExit - newRoute.begin();

                                int position = itFindExit - roadSegmentAffectedID.begin();

                                tempReturn = roadSegmentAffected[position].second;

                                if (get<1>(tempReturn).find(timeCurrent + teCurrent) == get<1>(tempReturn).end()) {
                                    get<1>(tempReturn).insert(pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                                             {{RoadIDCurrent, 1, 0}}));
                                } else {
                                    get<1>(tempReturn)[timeCurrent + teCurrent].push_back({RoadIDCurrent, 1, 0});
                                }
                                get<2>(tempReturn).push_back(routeID);

                                if (itrFindNew != newRoute.end()) { // roadSegment属于newRoute
                                    get<0>(tempReturn) = NewTimeLeave;
                                    roadSegmentAffected[position].second = tempReturn;
                                } else {
                                    get<0>(tempReturn) = 0;
                                    roadSegmentAffected[position].second = tempReturn;
                                }
                            } else {
                                if (itrFindNew != newRoute.end()) {
                                    get<0>(tempReturn) = NewTimeLeave;

                                    get<1>(tempReturn).clear();
                                    if (get<1>(tempReturn).find(timeCurrent + teCurrent) == get<1>(tempReturn).end()) {
                                        get<1>(tempReturn).insert(
                                                pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                               {{RoadIDCurrent, 1, 0}}));
                                    } else {
                                        get<1>(tempReturn)[timeCurrent + teCurrent].push_back({RoadIDCurrent, 1, 0});
                                    }

                                    get<2>(tempReturn).clear();
                                    get<2>(tempReturn).push_back(routeID);

                                    roadSegmentAffected.push_back(make_pair(make_pair(nextRoadSegmentID,routeID), tempReturn));
                                } else {
                                    get<0>(tempReturn) = 0;
                                    get<1>(tempReturn).clear();

                                    if (get<1>(tempReturn).find(timeCurrent + teCurrent) == get<1>(tempReturn).end()) {
                                        get<1>(tempReturn).insert(
                                                pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                               {{RoadIDCurrent, 1, 0}}));
                                    } else {
                                        get<1>(tempReturn)[timeCurrent + teCurrent].push_back({RoadIDCurrent, 1, 0});
                                    }

                                    get<2>(tempReturn).clear();
                                    get<2>(tempReturn).push_back(routeID);

                                    roadSegmentAffected.push_back(make_pair(make_pair(unaffected_next_roadSegmentID,routeID), tempReturn));
                                }
                            }

                        }else{
                            TrafficFlow += 1;
                            TrafficFlowInsert += 1;
//                            itr->second[i][2] = TrafficFlowInsert;
                        }
                    } else {
                        itr = --timeRecords.end();
                        break;
                    }
                } else {
                    TrafficFlow += 1;
                    TrafficFlowInsert += 1;

                    if (TrafficFlow == TrafficFlowInsert) {
                        itr->second[i][2] = TrafficFlowInsert;
                        TerminalCondition = 1;

                        teCurrent = tm * (1 + sigma * pow(TrafficFlowInsert / varphi, beta));
                        RecordTime = timeCurrent + teCurrent;
                    } else {
                        itr->second[i][2] = TrafficFlowInsert;

                        routeID = itr->second[i][0];
                        teCurrent = tm * (1 + sigma * pow(TrafficFlowInsert / varphi, beta));

                        if (Insert.find(timeCurrent + teCurrent) == Insert.end()) {
                            Insert.insert(
                                    pair<int, vector<vector<int>>>((timeCurrent + teCurrent), {{RoadIDCurrent, 0, 0}}));
                        } else {
                            Insert[timeCurrent + teCurrent].push_back({RoadIDCurrent, 0, 0});
                        }
                        Deletion.push_back(routeID);

                        vector<int> affected_route = Pi[routeID];
                        vector<int> affected_route_w_roadSegmentID = route_node2roadSegment(affected_route);
                        int unaffected_next_roadSegmentID = findNextRoadSegmentID(affected_route_w_roadSegmentID,
                                                                                  RoadSegmentID.first);

                        // 查找下一个roadSegment是不是newRoute的
                        std::vector<int>::iterator itrFindNew;
                        itrFindNew = std::find(newRoute.begin(), newRoute.end(), unaffected_next_roadSegmentID);

                        // 查找下一个roadSegment是否已经存在于roadSegmentAffected中
                        roadSegmentAffectedID.clear();
                        for (int i = 0; i < roadSegmentAffected.size(); i++) {
                            int tempRoadSegmentID = roadSegmentAffected[i].first.first;
                            roadSegmentAffectedID.push_back(tempRoadSegmentID);
                        }

                        std::vector<int>::iterator itFindExit;
                        itFindExit = std::find(roadSegmentAffectedID.begin(),
                                               roadSegmentAffectedID.end(), unaffected_next_roadSegmentID);

                        // vector<pair<int, tuple<int, map<int, vector<int>, vector<int>>>>> roadSegmentAffected;
                        if (itFindExit != roadSegmentAffectedID.end()) { // 已经添加
                            int position = itFindExit - roadSegmentAffectedID.begin();
                            tempReturn = roadSegmentAffected[position].second;

                            if (get<1>(tempReturn).find(timeCurrent + teCurrent) == get<1>(tempReturn).end()) {
                                get<1>(tempReturn).insert(pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                                         {{RoadIDCurrent, 1, 0}}));
                            } else {
                                get<1>(tempReturn)[timeCurrent + teCurrent].push_back({RoadIDCurrent, 1, 0});
                            }

                            get<2>(tempReturn).push_back(routeID);

                            if (itrFindNew != newRoute.end()) { // roadSegment属于newRoute
                                get<0>(tempReturn) = NewTimeLeave;
                                roadSegmentAffected[position].second = tempReturn;
                            } else {
                                get<0>(tempReturn) = 0;
                                roadSegmentAffected[position].second = tempReturn;
                            }
                        } else {
                            if (itrFindNew != newRoute.end()) {
                                get<0>(tempReturn) = NewTimeLeave;

                                get<1>(tempReturn).clear();
                                if (get<1>(tempReturn).find(timeCurrent + teCurrent) == get<1>(tempReturn).end()) {
                                    get<1>(tempReturn).insert(pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                                             {{RoadIDCurrent, 1, 0}}));
                                } else {
                                    get<1>(tempReturn)[timeCurrent + teCurrent].push_back({RoadIDCurrent, 1, 0});
                                }

                                get<2>(tempReturn).clear();
                                get<2>(tempReturn).push_back(routeID);

                                roadSegmentAffected.push_back(make_pair(make_pair(nextRoadSegmentID,routeID), tempReturn));
                            } else {
                                get<0>(tempReturn) = 0;

                                get<1>(tempReturn).clear();
                                if (get<1>(tempReturn).find(timeCurrent + teCurrent) == get<1>(tempReturn).end()) {
                                    get<1>(tempReturn).insert(pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                                             {{RoadIDCurrent, 1, 0}}));
                                } else {
                                    get<1>(tempReturn)[timeCurrent + teCurrent].push_back({RoadIDCurrent, 1, 0});
                                }

                                get<2>(tempReturn).clear();
                                get<2>(tempReturn).push_back(routeID);

                                roadSegmentAffected.push_back(make_pair(make_pair(unaffected_next_roadSegmentID,routeID), tempReturn));
                            }
                        }
                    }
                }
            }


                // Step 3.5: If Time Record is Driving Out Status.
                // -------------------------------------------
            else {

                // Step 3.6: Compare Time Records with Element in the Deletion Set
                // -------------------------------------------
                if (Deletion.size() != 0) {
                    if (RoadIDCurrent == Deletion[0]) { // (3.45e-06)
                        TrafficFlow -= 1;
                        itr = timeRecords.erase(itr);
                        itr = --itr;

                        vector<int>::iterator k = Deletion.begin();
                        Deletion.erase(k);
                    } else {
                        TrafficFlowInsert -= 1;
                        TrafficFlow -= 1;
                        itr->second[i][2] = TrafficFlowInsert;
                    }
                }else{
                    TrafficFlowInsert -= 1; TrafficFlow -= 1;
                    itr->second[i][2] = TrafficFlowInsert;
                }
            }
        }
    }


    // Step 3.7 Insert Rest Insert Elements
    // -------------------------------------------
    map<int, vector<vector<int>>>::iterator itrEnd; itrEnd = --timeRecords.end();

    int FlowEnd = itrEnd->second.back()[2];

/*    if (FlowEnd < 0) {
        cout << "Error! Number of Traffic Flow Is Smaller Than 0." << endl;
//        return Null;
    }*/

    int timeRest, RouteIDRest, FlowRest;

    map<int, vector<vector<int>>>::iterator itrRestI;

    for (itrRestI = ++Insert.begin(); itrRestI != Insert.end(); ++itrRestI) {

        for (int i=0;i<itrRestI->second.size();i++){

            timeRest = itrRestI->first; RouteIDRest = itrRestI->second[i][0];

            FlowRest = FlowEnd - 1;

/*            if (FlowRest < 0) {
                cout << "Error! Number of Traffic Flow Is Smaller Than 0." << endl;
            }*/

            if (timeRecords.find(timeRest) == timeRecords.end())
            {
                timeRecords.insert(pair<int, vector<vector<int>>>(timeRest, {{RouteIDRest, 0, FlowRest}}));
            }
            else
            {
                timeRecords[timeRest].push_back({RouteIDRest, 0, FlowRest});
            }
        }

    }

    // Step 4: Print Time Records
    // -------------------------------------------
    /*
    map<int, vector<vector<int>>>::iterator itrPrint;
    cout << "TimeRecord Size is: " << timeRecords.size() << endl;
    cout << "RoadSegmentID: " << RoadSegmentID << " with TimeSlice: " << index << endl;
    for (itrPrint = timeRecords.begin(); itrPrint != timeRecords.end(); ++itrPrint) {
        for (int i=0;i<itrPrint->second.size();i++)
        {
            cout << " time " << itrPrint->first << " routeID " << itrPrint->second[i][0];
            cout << " status " << itrPrint->second[i][1] << " flow " << itrPrint->second[i][2] << "||";
        }
    }
    cout << endl;
    */

    // Step 5: Update Result
    // -------------------------------------------
    timeRecordsChecked[RoadSegmentID.first][index] = timeRecords;

    // Step 6: Define Returned Value Even No Affected Propagated Route
    // -------------------------------------------
    /*
    cout << "NewTimeLeave is: " << NewTimeLeave << " nextRoadSegmentID is: " << nextRoadSegmentID << endl;
    */

    std::vector<int>::iterator itr_findNextExist;
    roadSegmentAffectedID.clear();
    for (int i=0;i<roadSegmentAffected.size();i++){
        roadSegmentAffectedID.push_back(roadSegmentAffected[i].first.first);
    }
    itr_findNextExist = std::find(roadSegmentAffectedID.begin(), roadSegmentAffectedID.end(), nextRoadSegmentID);

    tuple<int, map<int, vector<vector<int>>>, vector<int>> fillReturn;

    // 如果没有从该路段继承受影响的time records，并且new route中该road segment不是最后一个，定义输出结果
    if (nextRoadSegmentID != -1 and itr_findNextExist == roadSegmentAffectedID.end()) {
        get<0>(fillReturn) = NewTimeLeave; get<1>(fillReturn).clear(); get<2>(fillReturn).clear();
        roadSegmentAffected.push_back(make_pair(make_pair(nextRoadSegmentID,0), fillReturn));
    }

    // Step 7: Print Further Affected Road Segment
    // -------------------------------------------
    /*
    cout << endl;
    cout << "Step 4: Returned Results" << endl;
    cout << "-----------------------------------------" << endl;
    for (int i=0;i<roadSegmentAffected.size();i++){
        cout << " route ID is: " << roadSegmentAffected[i].first << " with entrance time: " << get<0>(roadSegmentAffected[i].second) << endl;
        map<int, vector<vector<int>>>::iterator itrInsertPrint;
        for (itrInsertPrint = get<1>(roadSegmentAffected[i].second).begin(); itrInsertPrint != get<1>(roadSegmentAffected[i].second).end(); ++itrInsertPrint) {
            for (int i=0;i<itrInsertPrint->second.size();i++)
            {
                cout << " time " << itr->first << " routeID " << itr->second[i][0];
            }
        }
        cout << endl;
    }
    cout << endl;
    */

    return roadSegmentAffected;
}


vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> Graph::updateOperationFurther(
        pair<int,int> RoadSegmentID, int inTime, pair<int, vector<int>> newRoutePair, map<int, vector<vector<int>>> InsertPre, vector<int> DeletionPre){

    // Step 1: Initialization
    // -------------------------------------------
    int TrafficFlow = 0; int TrafficFlowInsert = 0;

    int TerminalCondition = 0; int RecordTime = 0;

    map<int, vector<vector<int>>> Insert; vector<int> Deletion;

    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> Null;

    int newRouteID = newRoutePair.first; vector<int> newRoute = newRoutePair.second;

    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> roadSegmentAffected;

    vector<int> roadSegmentAffectedID;

    if (inTime == 0 and InsertPre.size() == 0 and DeletionPre.size() == 0)
    {
        return Null;
    }

    // Step 2: Insertion Operation
    // -------------------------------------------
    int NewTimeLeave; map<int, vector<vector<int>>> timeRecords;

    int tm, current_node, next_node, te; int index;

    current_node = map_roadID_2_nodeID[RoadSegmentID.first].first; next_node = map_roadID_2_nodeID[RoadSegmentID.first].second;
    tm = map_nodeID_2_minTime[make_pair(current_node, next_node)];

    if (tm < 1){

        int iCurrentRoad = RoadSegmentID.first;
        int iRoadID = RoadSegmentID.second;
        vector<int> affected_route = Pi[iRoadID];
        vector<int> affected_route_w_roadSegmentID = route_node2roadSegment(affected_route);
        int iNextRoadID = findNextRoadSegmentID(affected_route_w_roadSegmentID, RoadSegmentID.first);

        if (iNextRoadID == -1)
        {
            return Null;
        }
        else
        {
            tuple<int, map<int, vector<vector<int>>>, vector<int>> tempReturn;
            get<0>(tempReturn) = inTime;
            get<1>(tempReturn) = InsertPre;
            get<2>(tempReturn) = DeletionPre;
            roadSegmentAffected.push_back(make_pair(make_pair(iNextRoadID,iRoadID),tempReturn));

            return roadSegmentAffected;
        }
    }

    if (inTime != 0){ // 如果该路段不属于new route

        // Step 2.1: Find Time Records
        // -------------------------------------------
        int hour = Time2Hour(inTime); index = Hour2Index(hour);
        timeRecords = timeRecordsChecked[RoadSegmentID.first][index];

        int error_check = timeRecords_correct_check(timeRecords);
        if (error_check != 0){return Null;}

        // print time records before manipulation
        map<int, vector<vector<int>>>::iterator itrPrintV1;

        cout << endl;
        cout << "Step 2: time records before manipulation" << endl;
        cout << "-----------------------------------------" << endl;
        cout << "TimeRecord Size is: " << timeRecords.size() << endl;
        cout << "RoadSegmentID: " << RoadSegmentID.first << " with TimeSlice: " << index << endl;
        for (itrPrintV1 = timeRecords.begin(); itrPrintV1 != timeRecords.end(); ++itrPrintV1) {
            for (int i=0;i<itrPrintV1->second.size();i++)
            {
                cout << " time " << itrPrintV1->first << " routeID " << itrPrintV1->second[i][0];
                cout << " status " << itrPrintV1->second[i][1] << " flow " << itrPrintV1->second[i][2] << "||";
            }
        }
        cout << endl;


        // Step 2.2: Find Time Value or Insert
        // -------------------------------------------
        int preFlow; vector<int> features = {newRouteID, 1, 0};
        map<int, vector<vector<int>>>::iterator itFind; itFind = timeRecords.find(inTime);

        if (itFind == timeRecords.end()){ // not find
            timeRecords.insert(pair<int, vector<vector<int>>>(inTime, {features}));

            // Step 2.3: Find Previous Value for Traffic Flow
            // -------------------------------------------
            map<int, vector<vector<int>>>::iterator itPre;
            itPre = timeRecords.find(inTime);
            if (itPre != timeRecords.end()){ // find
                if (itPre == timeRecords.begin()){
                    preFlow = 0;
                    /*
                    cout << "time is the first one with flow : " << preFlow << endl;
                    */
                } else{
                    itPre = --itPre; preFlow = itPre->second.back()[2];
                    /*
                    cout << "previous time: " << itPre->first << " flow: " << preFlow << endl;
                    */
                }
            } else{
                cout << "Error. no value found, please check Insertion." << endl;
                return Null;
            }
        } else{ // find
            preFlow = itFind->second.back()[2];
            timeRecords[inTime].push_back(features);
        }

        // Step 2.4: Estimate New Time Record's Travel Time
        // -------------------------------------------

        TrafficFlow = preFlow; TrafficFlowInsert = preFlow + 1;

        // Update Traffic Flow of Driving In Time Record
        map<int, vector<vector<int>>>::iterator itFind1;
        itFind1 = timeRecords.find(inTime); itFind1->second.back()[2] = TrafficFlowInsert;

        // Estimate New Time Record's Leaving Time
        te = tm * (1 + sigma * pow(TrafficFlowInsert/varphi, beta));
        NewTimeLeave = inTime+te;

        // Put New Time Record's Leaving Time Record in Insert Set
        if (Insert.find(NewTimeLeave) == Insert.end()){
            Insert.insert(pair<int, vector<vector<int>>>((NewTimeLeave), {{newRouteID, 0, 0}}));
        }
        else
        {
            Insert[NewTimeLeave].push_back({newRouteID, 0, 0});
        }

    }else{
        std::map<int, vector<vector<int>>>::iterator itr_find1st;

/*
        if (InsertPre.begin()->first == 0 and InsertPre.size() > 1)
*/
        if (InsertPre.size() != 0){
            itr_find1st = InsertPre.begin();

            int hour = Time2Hour(itr_find1st->first); index = Hour2Index(hour);

            timeRecords = timeRecordsChecked[RoadSegmentID.first][index];

            NewTimeLeave = 0;

            int error_check = timeRecords_correct_check(timeRecords);
            if (error_check != 0){return Null;}

            // print time records before manipulation

            map<int, vector<vector<int>>>::iterator itrPrintV1;
            cout << endl;
            cout << "Step 2: time records before manipulation" << endl;
            cout << "-----------------------------------------" << endl;
            cout << "TimeRecord Size is: " << timeRecords.size() << endl;
            cout << "RoadSegmentID: " << RoadSegmentID.first << " with TimeSlice: " << index << endl;
            for (itrPrintV1 = timeRecords.begin(); itrPrintV1 != timeRecords.end(); ++itrPrintV1) {
                for (int i=0;i<itrPrintV1->second.size();i++)
                {
                    cout << " time " << itrPrintV1->first << " routeID " << itrPrintV1->second[i][0];
                    cout << " status " << itrPrintV1->second[i][1] << " flow " << itrPrintV1->second[i][2] << "||";
                }
            }
            cout << endl;


        }else{
            cout << "I' size equals to 0." << endl;
        }
    }

    // Step 3: Update Operation
    // -------------------------------------------

    // Check New Time Record is Last One
    map<int, vector<vector<int>>>::iterator itrInser;

    // For Each Time Record Start From New Next.
    int timeCurrent, RoadIDCurrent, teCurrent, RouteIDIPre, nextRoadSegmentID;

    nextRoadSegmentID = findNextRoadSegmentID(newRoutePair.second, RoadSegmentID.first);

    tuple<int, map<int, vector<vector<int>>>, vector<int>> tempReturn;

    map<int, vector<vector<int>>>::iterator itr;

    if (inTime != 0){ // 如果该路段属于new route，从插入的后一位time record开始
        itrInser = ++timeRecords.find(inTime);
    }else{ // 如果该路段不属于new route，从第一个time record开始
        itrInser = timeRecords.begin();
    }

    /*for (itr = itrInser; itr != timeRecords.end(); ++itr)*/
    for (itr = itrInser; itr != timeRecords.end();){

        bool bBreak = false;
        bool bBreak2 = false;
        for (int i = 0; i < itr->second.size(); i++)
        {
            timeCurrent = itr->first; RoadIDCurrent = itr->second[i][0];


            int temp = TrafficFlowInsert;

            int temp_size = itr->second.size();

            int insetionPre_check = 0;

            map<int, vector<vector<int>>>::iterator itrInPre; int RouteIDInPre;

            /*for (itrInPre = ++InsertPre.begin(); itrInPre != InsertPre.end(); ++itrInPre)*/
            for (itrInPre = InsertPre.begin(); itrInPre != InsertPre.end();){

                int timeIPre = itrInPre->first;

                if (timeIPre < timeCurrent)
                {
                    // ---
                    map<int, vector<vector<int>>>::iterator itrI0; int RouteIDI0;
                    for (itrI0 = Insert.begin(); itrI0 != Insert.end();){

                        int timeI0 = itrI0->first;

                        if (timeI0 < timeIPre){
                            for (int j = 0; j < itrI0->second.size(); j++)
                            {
                                // Compare Each Time Record in Insert Set.
                                TrafficFlowInsert -= 1; RouteIDI0 = itrI0->second[j][0];

                                if (itrI0->second[j][1] == 1){ // Status In.
                                    cout << "Error. element in I is not leaving status." << endl;
                                }
                                else
                                { // Status Out.
                                    if (timeRecords.find(timeI0) == timeRecords.end()) {
                                        timeRecords.insert(
                                                pair<int, vector<vector<int>>>(timeI0, {{RouteIDI0, 0, TrafficFlowInsert}}));
                                    } else {
                                        timeRecords[timeI0].push_back({RouteIDI0, 0, TrafficFlowInsert});
                                    }
                                }
                            }
                            itrI0 = Insert.erase(itrI0);
                        }
                        else
                        {
                            ++itrI0;
                        }
                    }
                    // --

                    int inside_time_current, inside_roadID_current;
                    for (int j = 0; j < itrInPre->second.size(); j++)
                    {
                        TrafficFlowInsert += 1;

                        RouteIDInPre = itrInPre->second[j][0];

                        if (itrInPre->second[j][1] == 1){ // if (itrI->second[1] == 1){
                            if (timeRecords.find(timeIPre) == timeRecords.end()) {
                                timeRecords.insert(
                                        pair<int, vector<vector<int>>>(timeIPre, {{RouteIDInPre, 1, TrafficFlowInsert}}));
                            } else {
                                timeRecords[timeIPre].push_back({RouteIDInPre, 1, TrafficFlowInsert});
                            }
                        }else{ // Status Out.
                            cout << "Error. element in I' is not driving in status." << endl;
                        }

                        inside_time_current = itrInPre->first; inside_roadID_current = itrInPre->second[j][0];
                        // inside_roadID_current = itr->second.back()[0];

                        insetionPre_check = 1;

                        teCurrent = tm * (1 + sigma * pow(TrafficFlowInsert/varphi, beta));

                        if (Insert.find(NewTimeLeave) == Insert.end()){
                            Insert.insert(
                                    pair<int, vector<vector<int>>>((inside_time_current+teCurrent), {{inside_roadID_current, 0, 0}}));
                        }
                        else
                        {
                            Insert[inside_time_current+teCurrent].push_back({inside_roadID_current, 0, 0});
                        }

                        Deletion.push_back(inside_roadID_current);

                        int routeID =  itrInPre->second[j][0];

                        int unaffected_next_roadSegmentID;

                        if (routeID == newRouteID){
                            unaffected_next_roadSegmentID = nextRoadSegmentID;
                        }else{
                            vector<int> affected_route = Pi[routeID];
                            vector<int> affected_route_w_roadSegmentID = route_node2roadSegment(affected_route);
                            unaffected_next_roadSegmentID = findNextRoadSegmentID(affected_route_w_roadSegmentID, RoadSegmentID.first);
                        }

                        std::vector<int>::iterator itrFindNew;
                        itrFindNew = std::find(newRoute.begin(), newRoute.end(), unaffected_next_roadSegmentID);

                        // 查找下一个roadSegment是否已经存在于roadSegmentAffected中
                        roadSegmentAffectedID.clear();
                        for (int k=0;k<roadSegmentAffected.size();k++){
                            int tempRoadSegmentID = roadSegmentAffected[k].first.first;
                            roadSegmentAffectedID.push_back(tempRoadSegmentID);
                        }

                        std::vector<int>::iterator itFindExit;
                        itFindExit = std::find(roadSegmentAffectedID.begin(), roadSegmentAffectedID.end(), unaffected_next_roadSegmentID);

                        if (itFindExit != roadSegmentAffectedID.end()){ // 已经添加

                            int position = itFindExit - roadSegmentAffectedID.begin();

                            tempReturn = roadSegmentAffected[position].second;

                            if (get<1>(tempReturn).find(inside_time_current + teCurrent) == get<1>(tempReturn).end()) {
                                get<1>(tempReturn).insert(
                                        pair<int, vector<vector<int>>>((inside_time_current + teCurrent),{{inside_roadID_current, 1, 0}}));
                            } else {
                                get<1>(tempReturn)[inside_time_current + teCurrent].push_back({inside_roadID_current, 1, 0});
                            }

                            get<2>(tempReturn).push_back(inside_roadID_current);

                            if (itrFindNew != newRoute.end()){ // roadSegment属于newRoute
                                get<0>(tempReturn) = NewTimeLeave;
                                roadSegmentAffected[position].second = tempReturn;
                            }else{
                                get<0>(tempReturn) = 0;
                                roadSegmentAffected[position].second = tempReturn;
                            }
                        }else{
                            if (itrFindNew != newRoute.end()){
                                get<0>(tempReturn) = NewTimeLeave;

                                get<1>(tempReturn).clear();
                                if (get<1>(tempReturn).find(inside_time_current + teCurrent) == get<1>(tempReturn).end()) {
                                    get<1>(tempReturn).insert(
                                            pair<int, vector<vector<int>>>((inside_time_current + teCurrent),{{inside_roadID_current, 1, 0}}));
                                } else {
                                    get<1>(tempReturn)[inside_time_current + teCurrent].push_back({inside_roadID_current, 1, 0});
                                }

                                get<2>(tempReturn).clear();get<2>(tempReturn).push_back(inside_roadID_current);
                                roadSegmentAffected.push_back(make_pair(make_pair(unaffected_next_roadSegmentID,routeID), tempReturn));
                            }else{
                                get<0>(tempReturn) = 0;

                                get<1>(tempReturn).clear();
                                if (get<1>(tempReturn).find(inside_time_current + teCurrent) == get<1>(tempReturn).end()) {
                                    get<1>(tempReturn).insert(
                                            pair<int, vector<vector<int>>>((inside_time_current + teCurrent),{{inside_roadID_current, 1, 0}}));
                                } else {
                                    get<1>(tempReturn)[inside_time_current + teCurrent].push_back({inside_roadID_current, 1, 0});
                                }

                                get<2>(tempReturn).clear();get<2>(tempReturn).push_back(inside_roadID_current);
                                roadSegmentAffected.push_back(make_pair(make_pair(unaffected_next_roadSegmentID,routeID), tempReturn));
                            }
                        }
                    }
                    itrInPre = InsertPre.erase(itrInPre);
                }
                else
                {
                    ++itrInPre;
                }
            }

            // Step 4.1: Compare Time Records in Insert Set
            // -------------------------------------------
            map<int, vector<vector<int>>>::iterator itrI; int RouteIDI;
            /*for (itrI = ++Insert.begin(); itrI != Insert.end(); ++itrI)*/
            for (itrI = Insert.begin(); itrI != Insert.end();){

                int timeI = itrI->first;

                if (timeI < timeCurrent){
                    for (int j = 0; j < itrI->second.size(); j++)
                    {
                        // Compare Each Time Record in Insert Set.
                        TrafficFlowInsert -= 1; RouteIDI = itrI->second[j][0];

                        if (itrI->second[j][1] == 1){ // Status In.
                            cout << "Error. element in I is not leaving status." << endl;
                        }
                        else
                        { // Status Out.
                            if (timeRecords.find(timeI) == timeRecords.end()) {
                                timeRecords.insert(
                                        pair<int, vector<vector<int>>>(timeI, {{RouteIDI, 0, TrafficFlowInsert}}));
                            } else {
                                timeRecords[timeI].push_back({RouteIDI, 0, TrafficFlowInsert});
                            }
                        }
                    }
                    itrI = Insert.erase(itrI);
                }
                else
                {
                    ++itrI;
                }
            }


            if (inTime == 0 and InsertPre.size() == 0 and DeletionPre.size() == 0 and Insert.size() == 0 and Deletion.size() == 0){
                bBreak2 = true;
                break;
            }

            // Step 4.2: Iteration of Update Operation
            // -------------------------------------------
            // If Time Record is Driving In Status.
            map<int, vector<vector<int>>>::iterator itrIPre;

            if (itr->second[i][1] == 1){

                if (TerminalCondition == 1 and InsertPre.size() == 0 and DeletionPre.size() == 0 and Insert.size() == 0 and Deletion.size() == 0){
                    bBreak2 = true;
                    break;
                }

                if (DeletionPre.size() != 0){
                    if (RoadIDCurrent == DeletionPre[0]){
                        TrafficFlow += 1;
/*
                        timeRecords.insert(pair<int, vector<vector<int>>>(0, {{0,0,0}}));
*/
                        // i = itr->second.size();

                        itr->second.erase(itr->second.begin() + i);
                        i--;
/*
                        itr = timeRecords.erase(itr); itr = --itr;
*/

                        vector<int>::iterator k = DeletionPre.begin();

                        DeletionPre.erase(k);

                        // continue;

                        if(itr->second.size() == 0)
                        {
                            itr = timeRecords.erase(itr);
                            bBreak = true;
                            break;
                        }

                        continue;

                    }else{
                        if (TerminalCondition == 1){
                            if (itr == timeRecords.end() and i == itr->second.size()){
                                // itr = --timeRecords.end();
                                bBreak2 = true;
                                break;
                            }
                            if (timeCurrent <= RecordTime){
                                if (TrafficFlow != TrafficFlowInsert){
                                    TerminalCondition = 0; TrafficFlow += 1; TrafficFlowInsert += 1;

                                    itr->second[i][2] = TrafficFlowInsert;

                                    teCurrent = tm * (1 + sigma * pow(TrafficFlowInsert/varphi, beta));

                                    if (Insert.find(timeCurrent + teCurrent) == Insert.end()) {
                                        Insert.insert(pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                                     {{RoadIDCurrent, 0, 0}}));
                                    } else {
                                        Insert[timeCurrent + teCurrent].push_back({RoadIDCurrent, 0, 0});
                                    }

                                    Deletion.push_back(RoadIDCurrent);

                                    // nextRoadSegmentID = findNextRoadSegmentID(newRoutePair.second, RoadSegmentID);
                                    int routeID =  itr->second[i][0];

                                    int unaffected_next_roadSegmentID;

                                    if (routeID == newRouteID){
                                        unaffected_next_roadSegmentID = nextRoadSegmentID;
                                    }else{
                                        vector<int> affected_route = Pi[routeID];
                                        vector<int> affected_route_w_roadSegmentID = route_node2roadSegment(affected_route);
                                        unaffected_next_roadSegmentID = findNextRoadSegmentID(affected_route_w_roadSegmentID, RoadSegmentID.first);
                                    }

                                    // 查找下一个roadSegment是不是newRoute的
                                    std::vector<int>::iterator itrFindNew;

                                    itrFindNew = std::find (newRoute.begin(), newRoute.end(), unaffected_next_roadSegmentID);

                                    // 查找下一个roadSegment是否已经存在于roadSegmentAffected中
                                    roadSegmentAffectedID.clear();
                                    for (int k=0;k<roadSegmentAffected.size();k++){
                                        int tempRoadSegmentID = roadSegmentAffected[k].first.first;
                                        roadSegmentAffectedID.push_back(tempRoadSegmentID);
                                    }

                                    std::vector<int>::iterator itFindExit;
                                    itFindExit = std::find(roadSegmentAffectedID.begin(), roadSegmentAffectedID.end(), unaffected_next_roadSegmentID);

                                    if (itFindExit != roadSegmentAffectedID.end()){ // 已经添加
                                        // int position = itFindExit - newRoute.begin();

                                        int position = itFindExit - roadSegmentAffectedID.begin();

                                        tempReturn = roadSegmentAffected[position].second;

                                        if (get<1>(tempReturn).find(timeCurrent + teCurrent) == get<1>(tempReturn).end()) {
                                            get<1>(tempReturn).insert(pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                                                     {{RoadIDCurrent, 1, 0}}));
                                        } else {
                                            get<1>(tempReturn)[timeCurrent + teCurrent].push_back({RoadIDCurrent, 1, 0});
                                        }

                                        get<2>(tempReturn).push_back(RoadIDCurrent);

                                        if (itrFindNew != newRoute.end()){ // roadSegment属于newRoute
                                            get<0>(tempReturn) = NewTimeLeave;
                                            roadSegmentAffected[position].second = tempReturn;
                                        }else{
                                            get<0>(tempReturn) = 0;
                                            roadSegmentAffected[position].second = tempReturn;
                                        }
                                    }else{
                                        if (itrFindNew != newRoute.end()){
                                            get<0>(tempReturn) = NewTimeLeave;

                                            get<1>(tempReturn).clear();
                                            if (get<1>(tempReturn).find(timeCurrent + teCurrent) == get<1>(tempReturn).end()) {
                                                get<1>(tempReturn).insert(pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                                                         {{RoadIDCurrent, 1, 0}}));
                                            } else {
                                                get<1>(tempReturn)[timeCurrent + teCurrent].push_back({RoadIDCurrent, 1, 0});
                                            }

                                            get<2>(tempReturn).clear();get<2>(tempReturn).push_back(RoadIDCurrent);
                                            roadSegmentAffected.push_back(make_pair(make_pair(unaffected_next_roadSegmentID,routeID), tempReturn));
                                        }else{
                                            get<0>(tempReturn) = 0;

                                            get<1>(tempReturn).clear();
                                            if (get<1>(tempReturn).find(timeCurrent + teCurrent) == get<1>(tempReturn).end()) {
                                                get<1>(tempReturn).insert(pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                                                         {{RoadIDCurrent, 1, 0}}));
                                            } else {
                                                get<1>(tempReturn)[timeCurrent + teCurrent].push_back({RoadIDCurrent, 1, 0});
                                            }

                                            get<2>(tempReturn).clear();get<2>(tempReturn).push_back(RoadIDCurrent);
                                            roadSegmentAffected.push_back(make_pair(make_pair(unaffected_next_roadSegmentID,routeID), tempReturn));
                                        }
                                    }
                                }else{
                                    TrafficFlow += 1; TrafficFlowInsert += 1;
                                }
                            }else{

                                if (InsertPre.size() == 0 and DeletionPre.size() == 0){
                                    bBreak2 = true;
                                    break;
                                }

                                TrafficFlow += 1; TrafficFlowInsert += 1;

                                itr->second[i][2] = TrafficFlowInsert;
                            }
                        }else{
                            TrafficFlow += 1; TrafficFlowInsert += 1;

                            if (TrafficFlow == TrafficFlowInsert){
                                itr->second[i][2] = TrafficFlowInsert;

                                TerminalCondition = 1;

                                teCurrent = tm * (1 + sigma * pow(TrafficFlowInsert/varphi, beta));
                                RecordTime = timeCurrent+teCurrent;
                            }else{
                                itr->second[i][2] = TrafficFlowInsert; RoadIDCurrent =  itr->second[i][0];

                                teCurrent = tm * (1 + sigma * pow(TrafficFlowInsert/varphi, beta));

                                if (Insert.find(timeCurrent + teCurrent) == Insert.end()) {
                                    Insert.insert(pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                                 {{RoadIDCurrent, 0, 0}}));
                                } else {
                                    Insert[timeCurrent + teCurrent].push_back({RoadIDCurrent, 0, 0});
                                }

                                Deletion.push_back(RoadIDCurrent);

                                int routeID =  itr->second[i][0];

                                int unaffected_next_roadSegmentID;

                                if (routeID == newRouteID){
                                    unaffected_next_roadSegmentID = nextRoadSegmentID;
                                }else{
                                    vector<int> affected_route = Pi[routeID];
                                    vector<int> affected_route_w_roadSegmentID = route_node2roadSegment(affected_route);
                                    unaffected_next_roadSegmentID = findNextRoadSegmentID(affected_route_w_roadSegmentID, RoadSegmentID.first);
                                }

                                // 查找下一个roadSegment是不是newRoute的
                                std::vector<int>::iterator itrFindNew;
                                itrFindNew = std::find (newRoute.begin(), newRoute.end(), unaffected_next_roadSegmentID);

                                // 查找下一个roadSegment是否已经存在于roadSegmentAffected中
                                roadSegmentAffectedID.clear();
                                for (int k=0;k<roadSegmentAffected.size();k++){
                                    int tempRoadSegmentID = roadSegmentAffected[k].first.first;
                                    roadSegmentAffectedID.push_back(tempRoadSegmentID);
                                }

                                std::vector<int>::iterator itFindExit;
                                itFindExit = std::find(roadSegmentAffectedID.begin(), roadSegmentAffectedID.end(), unaffected_next_roadSegmentID);

                                if (itFindExit != roadSegmentAffectedID.end()){ // 已经添加

                                    int position = itFindExit - roadSegmentAffectedID.begin();
                                    tempReturn = roadSegmentAffected[position].second;

                                    if (get<1>(tempReturn).find(timeCurrent + teCurrent) == get<1>(tempReturn).end()) {
                                        get<1>(tempReturn).insert(pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                                                 {{RoadIDCurrent, 1, 0}}));
                                    } else {
                                        get<1>(tempReturn)[timeCurrent + teCurrent].push_back({RoadIDCurrent, 1, 0});
                                    }

                                    get<2>(tempReturn).push_back(RoadIDCurrent);

                                    if (itrFindNew != newRoute.end()){ // roadSegment属于newRoute
                                        get<0>(tempReturn) = NewTimeLeave;
                                        roadSegmentAffected[position].second = tempReturn;
                                    }else{
                                        get<0>(tempReturn) = 0;
                                        roadSegmentAffected[position].second = tempReturn;
                                    }
                                }else{
                                    if (itrFindNew != newRoute.end()){
                                        get<0>(tempReturn) = NewTimeLeave;

                                        get<1>(tempReturn).clear();
                                        if (get<1>(tempReturn).find(timeCurrent + teCurrent) == get<1>(tempReturn).end()) {
                                            get<1>(tempReturn).insert(pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                                                     {{RoadIDCurrent, 1, 0}}));
                                        } else {
                                            get<1>(tempReturn)[timeCurrent + teCurrent].push_back({RoadIDCurrent, 1, 0});
                                        }

                                        get<2>(tempReturn).clear();get<2>(tempReturn).push_back(RoadIDCurrent);
                                        roadSegmentAffected.push_back(make_pair(make_pair(unaffected_next_roadSegmentID,routeID), tempReturn));
                                    }else{
                                        get<0>(tempReturn) = 0;

                                        get<1>(tempReturn).clear();
                                        if (get<1>(tempReturn).find(timeCurrent + teCurrent) == get<1>(tempReturn).end()) {
                                            get<1>(tempReturn).insert(pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                                                     {{RoadIDCurrent, 1, 0}}));
                                        } else {
                                            get<1>(tempReturn)[timeCurrent + teCurrent].push_back({RoadIDCurrent, 1, 0});
                                        }

                                        get<2>(tempReturn).clear();get<2>(tempReturn).push_back(RoadIDCurrent);
                                        roadSegmentAffected.push_back(make_pair(make_pair(unaffected_next_roadSegmentID,routeID), tempReturn));
                                    }
                                }

                            }
                        }
                    }
                }else{
                    if (inTime != 0 and insetionPre_check != 1){
                        TrafficFlowInsert += 1; TrafficFlow += 1;
                        itr->second[i][2] = TrafficFlowInsert;
                    }
                    else
                    {
/*                        if (inTime == 0 and InsertPre.size() == 0 and DeletionPre.size() == 0 and TerminalCondition == 1)
                        {
                            TrafficFlowInsert += 1; TrafficFlow += 1;
                            itr->second[i][2] = TrafficFlowInsert;
                        }*/
                        if (InsertPre.size() == 0 and DeletionPre.size() == 0)
                        {
                            TrafficFlowInsert += 1; TrafficFlow += 1;
                            itr->second[i][2] = TrafficFlowInsert;
                        }
                    }
                }

            }else{

                if (DeletionPre.size() != 0 or inTime != 0 or Deletion.size() != 0){
                    if (Deletion.size() != 0 and RoadIDCurrent == Deletion[0]){ // (3.45e-06)
                        TrafficFlow -= 1;

/*                        i = itr->second.size();
                        itr = timeRecords.erase(itr); itr = --itr;*/

                        itr->second.erase(itr->second.begin() + i);
                        i--;

                        vector<int>::iterator k = Deletion.begin(); Deletion.erase(k);

                        if(itr->second.size() == 0)
                        {
                            itr = timeRecords.erase(itr);
                            bBreak = true;
                            break;
                        }

                    }else{
                        if (itr->second[i][1] == 0){
                            TrafficFlowInsert -= 1;

                            TrafficFlow -= 1;
                            itr->second[i][2] = TrafficFlowInsert;
                        }
                    }
                }

            }
        }
        if(bBreak)
            continue;
        if(bBreak2)
            break;
    ++itr;

    }


    map<int, vector<vector<int>>>::iterator itrInPreRest; int RouteIDInPre;
    if (InsertPre.size() != 0){
        for (itrInPreRest = InsertPre.begin(); itrInPreRest != InsertPre.end(); ++itrInPreRest){
            for (int i=0;i<itrInPreRest->second.size();i++){
                // Compare Each Time Record in Insert Set.
                int timeIPre = itrInPreRest->first;

                // compare each time record in I' with time records in I
                map<int, vector<vector<int>>>::iterator itrI2; int RouteIDI;
                for (itrI2 = Insert.begin(); itrI2 != Insert.end();){
                    int timeI2 = itrI2->first;
                    if (timeI2 < timeIPre)
                    {
                        for (int j = 0; j < itrI2->second.size(); j++)
                        {
                            // Compare Each Time Record in Insert Set.
                            TrafficFlowInsert -= 1; RouteIDI = itrI2->second[j][0];

                            if (itrI2->second[j][1] == 1){ // Status In.
                                cout << "Error. element in I is not leaving status." << endl;
                            }else{ // Status Out.
                                if (timeRecords.find(timeI2) == timeRecords.end()) {
                                    timeRecords.insert(
                                            pair<int, vector<vector<int>>>(timeI2, {{RouteIDI, 0, TrafficFlowInsert}}));
                                } else {
                                    timeRecords[timeI2].push_back({RouteIDI, 0, TrafficFlowInsert});
                                }
                            }
                        }
                        itrI2 = Insert.erase(itrI2);
                    }
                    else
                    {
                        ++itrI2;
                    }
                }

                TrafficFlowInsert += 1; RouteIDInPre = itrInPreRest->second[i][0];

                if (timeRecords.find(timeIPre) == timeRecords.end())
                {
                    timeRecords.insert(pair<int, vector<vector<int>>>(timeIPre, {{RouteIDInPre,1,TrafficFlowInsert}}));
                }
                else
                {
                    timeRecords[timeIPre].push_back({RouteIDInPre,1,TrafficFlowInsert});
                }

                int timeCurrent = itrInPreRest->first; int RoadIDCurrent = itrInPreRest->second[i][0];
                teCurrent = tm * (1 + sigma * pow(TrafficFlowInsert/varphi, beta));

                if (Insert.find(timeCurrent + teCurrent) == Insert.end()) {
                    Insert.insert(pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                 {{RoadIDCurrent, 0, 0}}));
                } else {
                    Insert[timeCurrent + teCurrent].push_back({RoadIDCurrent, 0, 0});
                }

                Deletion.push_back(RoadIDCurrent);

                int routeID =  itrInPreRest->second[i][0];
                int unaffected_next_roadSegmentID;

                if (routeID == newRouteID){
                    unaffected_next_roadSegmentID = nextRoadSegmentID;
                }else{
                    vector<int> affected_route = Pi[routeID];
                    vector<int> affected_route_w_roadSegmentID = route_node2roadSegment(affected_route);
                    unaffected_next_roadSegmentID = findNextRoadSegmentID(affected_route_w_roadSegmentID, RoadSegmentID.first);
                }

                std::vector<int>::iterator itrFindNew;
                itrFindNew = std::find(newRoute.begin(), newRoute.end(), unaffected_next_roadSegmentID);

                // 查找下一个roadSegment是否已经存在于roadSegmentAffected中
                roadSegmentAffectedID.clear();
                for (int i=0;i<roadSegmentAffected.size();i++){
                    int tempRoadSegmentID = roadSegmentAffected[i].first.first;
                    roadSegmentAffectedID.push_back(tempRoadSegmentID);
                }

                std::vector<int>::iterator itFindExit;
                itFindExit = std::find(roadSegmentAffectedID.begin(), roadSegmentAffectedID.end(), unaffected_next_roadSegmentID);

                if (itFindExit != roadSegmentAffectedID.end()){ // 已经添加

                    int position = itFindExit - roadSegmentAffectedID.begin();

                    tempReturn = roadSegmentAffected[position].second;

                    if (get<1>(tempReturn).find(timeCurrent + teCurrent) == get<1>(tempReturn).end()) {
                        get<1>(tempReturn).insert(pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                                 {{RoadIDCurrent, 1, 0}}));
                    } else {
                        get<1>(tempReturn)[timeCurrent + teCurrent].push_back({RoadIDCurrent, 1, 0});
                    }

                    get<2>(tempReturn).push_back(RoadIDCurrent);

                    if (itrFindNew != newRoute.end()){ // roadSegment属于newRoute
                        get<0>(tempReturn) = NewTimeLeave;
                        roadSegmentAffected[position].second = tempReturn;
                    }else{
                        get<0>(tempReturn) = 0;
                        roadSegmentAffected[position].second = tempReturn;
                    }
                }else{
                    if (itrFindNew != newRoute.end()){
                        get<0>(tempReturn) = NewTimeLeave;

                        get<1>(tempReturn).clear();
                        if (get<1>(tempReturn).find(timeCurrent + teCurrent) == get<1>(tempReturn).end()) {
                            get<1>(tempReturn).insert(pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                                     {{RoadIDCurrent, 1, 0}}));
                        } else {
                            get<1>(tempReturn)[timeCurrent + teCurrent].push_back({RoadIDCurrent, 1, 0});
                        }

                        get<2>(tempReturn).clear();get<2>(tempReturn).push_back(RoadIDCurrent);
                        roadSegmentAffected.push_back(make_pair(make_pair(unaffected_next_roadSegmentID,routeID), tempReturn));
                    }else{
                        get<0>(tempReturn) = 0;

                        get<1>(tempReturn).clear();
                        if (get<1>(tempReturn).find(timeCurrent + teCurrent) == get<1>(tempReturn).end()) {
                            get<1>(tempReturn).insert(pair<int, vector<vector<int>>>((timeCurrent + teCurrent),
                                                                                     {{RoadIDCurrent, 1, 0}}));
                        } else {
                            get<1>(tempReturn)[timeCurrent + teCurrent].push_back({RoadIDCurrent, 1, 0});
                        }

                        get<2>(tempReturn).clear();get<2>(tempReturn).push_back(RoadIDCurrent);
                        roadSegmentAffected.push_back(make_pair(make_pair(unaffected_next_roadSegmentID,routeID), tempReturn));
                    }
                }
            }

        }
    }

    // Rest Deletion Element
    if (Deletion.size() != 0 and RoadIDCurrent == Deletion[0]) {
        TrafficFlow -= 1;

        map<int, vector<vector<int>>>::iterator itrend;
        itrend = --timeRecords.end();

        for (int i = 0; i < itrend->second.size(); i++)
        {
            if (itrend->second[i][1] == 0)
            {
//                timeRecords.erase(itrend);

                itrend->second.erase(itrend->second.begin() + i);
                i--;

                vector<int>::iterator k = Deletion.begin();
                Deletion.erase(k);

                if(itrend->second.size() == 0)
                {
                    timeRecords.erase(itrend);
                    break;
                }
            }
            else
            {
                continue;
            }
        }
    }


    // Rest Insert Elements
    map<int, vector<vector<int>>>::iterator itrEnd; int FlowEnd;
    itrEnd = --timeRecords.end();
    FlowEnd = itrEnd->second.back()[2];

    map<int, vector<vector<int>>>::iterator itrRestI;
    int timeRest, RouteIDRest, FlowRest;

    if (Insert.size() != 0){
        for (itrRestI = Insert.begin();itrRestI != Insert.end();++itrRestI){
            for (int i=0;i<itrRestI->second.size();i++){
                timeRest = itrRestI->first; RouteIDRest = itrRestI->second[i][0];

                FlowRest = FlowEnd - 1;;
                FlowEnd -= 1;

                if (timeRecords.find(timeRest) == timeRecords.end())
                {
                    timeRecords.insert(pair<int, vector<vector<int>>>(timeRest, {{RouteIDRest, 0, FlowRest}}));
                }
                else
                {
                    timeRecords[timeRest].push_back({RouteIDRest, 0, FlowRest});
                }

/*                if (FlowRest < 0){
                    cout << "Error! Number of Traffic Flow Is Smaller Than 0." << endl;
                }*/
            }

        }
    }


/*    // 处理如果timeRecords的第一个时间是0，由于站位造成的
    std::map<int, vector<vector<int>>>::iterator itr_check1st;

    itr_check1st = timeRecords.begin();

    if (itr_check1st->first == 0){
        itr_check1st = timeRecords.erase(itr_check1st);
    }*/

    // Step 5: Print Result
    // -------------------------------------------


    cout << endl;
    cout << "Step 3: time records after manipulation" << endl;
    cout << "-----------------------------------------" << endl;
    map<int, vector<vector<int>>>::iterator itrPrint;
    cout << "TimeRecord Size is: " << timeRecords.size() << endl;
    cout << "RoadSegmentID: " << RoadSegmentID.first << " with TimeSlice: " << index << endl;
    for (itrPrint = timeRecords.begin(); itrPrint != timeRecords.end(); ++itrPrint) {
        for (int i=0;i<itrPrint->second.size();i++)
        {
            cout << " time " << itrPrint->first << " routeID " << itrPrint->second[i][0];
            cout << " status " << itrPrint->second[i][1] << " flow " << itrPrint->second[i][2] << "||";
        }
    }
    cout << endl;

    // 检验结果正确性
    int error_check = timeRecords_correct_check(timeRecords);
    if (error_check != 0) {cout << "Error somewhere." << endl;}

    // 逃课
    /*
    if (error_check != 0)
    {
        return Null;
    }
    */

    // Step 5: Update Result
    // -------------------------------------------
    timeRecordsChecked[RoadSegmentID.first][index] = timeRecords;

    std::vector<int>::iterator itr_findNextExist;
    roadSegmentAffectedID.clear();
    for (int i=0;i<roadSegmentAffected.size();i++){
        roadSegmentAffectedID.push_back(roadSegmentAffected[i].first.first);
    }
    itr_findNextExist = std::find(roadSegmentAffectedID.begin(), roadSegmentAffectedID.end(), nextRoadSegmentID);

    tuple<int, map<int, vector<vector<int>>>, vector<int>> fillReturn;

    if (nextRoadSegmentID != -1 and itr_findNextExist == roadSegmentAffectedID.end()){
        get<0>(fillReturn) = NewTimeLeave; get<1>(fillReturn).clear(); get<2>(fillReturn).clear();
        roadSegmentAffected.push_back(make_pair(make_pair(nextRoadSegmentID,0),fillReturn));
    }

    /*
    cout << endl;
    cout << "Step 4: Returned Results" << endl;
    cout << "-----------------------------------------" << endl;
    for (int i=0;i<roadSegmentAffected.size();i++){
        cout << " roadSegmentAffected route ID is: " << roadSegmentAffected[i].first << " with entrance time: " << get<0>(roadSegmentAffected[i].second) << endl;
        map<int, vector<vector<int>>>::iterator itrInsertPrint;
        for (itrInsertPrint = get<1>(roadSegmentAffected[i].second).begin(); itrInsertPrint != get<1>(roadSegmentAffected[i].second).end(); ++itrInsertPrint) {
            for (int i=0;i<itrInsertPrint->second.size();i++)
            {
                cout << " time " << itrInsertPrint->first << " routeID " << itrInsertPrint->second[i][0];
                cout << " status " << itrInsertPrint->second[i][1] << " flow " << itrInsertPrint->second[i][2] << "||";
            }
        }
        cout << endl;
    }
    cout << endl;
    */

    return roadSegmentAffected;
}


vector<int> Graph::one_route_parallel_update(pair<int, vector<int>> newRoute, int inTime, vector<Semaphore*>& vLock){

    // Step 1: 初始化储存受影响的roadSegmentID的集合，把newRoute的路段添加进去
    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> roadSegmentSet; // RoadSegment, <0/leaveTime, I', D'>
    int RoadSegmentID1st = newRoute.second[0]; int count = 0;

    vector<int> affected_RoadSegmentID; affected_RoadSegmentID.clear();
    affected_RoadSegmentID.push_back(RoadSegmentID1st);

    /*
    cout << "=======================================================================" << endl;
    cout << "count: " << count << endl;
    */

    vLock[RoadSegmentID1st]->Wait();
    roadSegmentSet = updateOperation1st(make_pair(RoadSegmentID1st,newRoute.first), inTime, newRoute);
    vLock[RoadSegmentID1st]->Signal();

    /*
    cout << "roadSegmentSet Size is: " << roadSegmentSet.size() << endl;
    */

    // Step 2:  集合不为空的时候，
    pair<int, tuple<int, map<int, vector<vector<int>>>, vector<int>>> roadSegmentTemp;
    int leaveTime; map<int, vector<vector<int>>> insert; vector<int> deletion; pair<int,int> roadSegmentIDTemp;
    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> roadSegmentSetTemp;

    vector<int> passed_roadSegmentID; passed_roadSegmentID.clear();
    passed_roadSegmentID.push_back(newRoute.second[0]);

    while(roadSegmentSet.size() != 0){ // !roadSegmentSet.empty()
        count += 1;

        std::vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>>::iterator itr;
        itr = roadSegmentSet.begin();

        roadSegmentIDTemp = itr->first; leaveTime = get<0>(itr->second);

        vector<int>::iterator itr_findDup;
        itr_findDup = std::find (passed_roadSegmentID.begin(), passed_roadSegmentID.end(), roadSegmentIDTemp.first);

        if (roadSegmentIDTemp.first == -1 or itr_findDup != passed_roadSegmentID.end()){ // 或者已经计算过了
            roadSegmentSet.erase(itr);
            continue;
        }

        if (roadSegmentIDTemp.first != -1){

            /*
            cout << "=======================================================================" << endl;

            cout << endl;
            cout << "Step 1: round start" << endl;
            cout << "-----------------------------------------" << endl;
            cout << "count: " << count << endl;
            cout << "road segment ID is: " << roadSegmentIDTemp << " with leaveTime: " << leaveTime << endl;
            */

            insert = get<1>(itr->second); deletion = get<2>(itr->second);
        }

        roadSegmentSet.erase(itr);

        /*
        cout << "Insert set size is: " << insert.size() << " Deletion size is: " << deletion.size() << endl;
        */

        vLock[roadSegmentIDTemp.first]->Wait();
        roadSegmentSetTemp = updateOperationFurther(roadSegmentIDTemp, leaveTime, newRoute, insert, deletion);
        vLock[roadSegmentIDTemp.first]->Signal();

        affected_RoadSegmentID.push_back(roadSegmentIDTemp.first);

        passed_roadSegmentID.push_back(roadSegmentIDTemp.first);

        for (int i=0;i<roadSegmentSetTemp.size();i++){
            roadSegmentSet.push_back(roadSegmentSetTemp[i]);
        }

    }

    /*
    cout << "This Route Update Done." << endl;
    cout << "=======================================================================" << endl;
    */
    affected_parallel_roadID.clear();
    affected_parallel_roadID = affected_RoadSegmentID;
    return affected_RoadSegmentID;

}

//int Graph::flowEvaluation(vector<vector<map<int, vector<int>>>> &timeRecordsChecked, int searchTime, int RoadSegmentID){
//
//    // 找到目标路段以及时间区间
//    map<int, vector<int>> timeRecords; int Null;
//    int hour = Time2Hour(searchTime); int index = Hour2Index(hour);
//    timeRecords = timeRecordsChecked[RoadSegmentID][index];
//
//    int preFlow; vector<int> features = {searchTime, 1, 0};
//    // 循环判断目标时间是否存在
//    map<int, vector<int>>::iterator itFind; itFind = timeRecords.find(searchTime);
//    if (itFind == timeRecords.end()){ // not find
//        timeRecords.insert(pair<int, vector<int>>(searchTime, features));
//
//        // Find Previous Value for Traffic Flow
//        // -------------------------------------------
//        map<int, vector<int>>::iterator itPre;
//        itPre = timeRecords.find(searchTime);
//        if (itPre != timeRecords.end()){ // find
//            if (itPre == timeRecords.begin()){
//                preFlow = 0;
//                /*
//                cout << "time is the first one with flow : " << preFlow << endl;
//                */
//                timeRecords.erase(itPre);
//            } else{
//                itPre = --itPre; preFlow = itPre->second[2];
//                // cout << "previous time: " << itPre->first << " flow: " << preFlow << endl;
//                itPre = ++itPre; timeRecords.erase(itPre);
//            }
//        } else{
//            cout << "No value found. Please check Insertion." << endl;
//            return Null;
//        }
//    } else{ // find
//        preFlow = itFind->second[2];
//        cout << "time record has already existed. Insertion failed.";
//        return Null;
//    }
//
//    map<int, vector<int>>::iterator itrPrint;
//    cout << "TimeRecord Size is: " << timeRecords.size() << endl;
//    cout << "RoadSegmentID: " << RoadSegmentID << " with TimeSlice: " << index << endl;
//    for (itrPrint = timeRecords.begin(); itrPrint != timeRecords.end(); ++itrPrint) {
//        cout << " time " << itrPrint->first << " routeID " << itrPrint->second[0];
//        cout << " status " << itrPrint->second[1] << " flow " << itrPrint->second[2] << "||";
//    }
//    cout << endl;
//
//    return preFlow;
//}


void Graph::multi_route_update(vector<pair<pair<int, vector<int>>, int>> &multi_test){

    vLock.reserve(new_edgenum);
    for(int i = 0; i < new_edgenum; i++)
    {
        Semaphore *sm = new Semaphore(1);
        vLock.push_back(sm);
    }

    boost::thread_group threadf;

    vector<int> id_temp;
    for (int i=0;i<multi_test.size();i++)
    {
        // one_route_parallel_update(multi_test[i].first, multi_test[i].second, vLock);
        id_temp = one_route_update(multi_test[i].first, multi_test[i].second);
        // threadf.add_thread(new boost::thread(&Graph::one_route_parallel_update, this, multi_test[i].first, multi_test[i].second, vLock));

        threadf.join_all();

        cout << "round number is: " << i << endl;

/*        // print manipulated time records.
        for (int j=0;j<id_temp.size();j++)
        {
            map<int, vector<vector<int>>> timeRecords; int inTime = 5296701;
            int RoadSegmentID = id_temp[j];

            int hour = Time2Hour(inTime); int index = Hour2Index(hour);
            timeRecords = timeRecordsChecked[RoadSegmentID][index];

            cout << endl;
            cout << "count " << j << endl;
            cout << "-----------------------------------------" << endl;
            map<int, vector<vector<int>>>::iterator itrPrint;

            cout << "TimeRecord Size is: " << timeRecords.size() << endl;
            cout << "RoadSegmentID: " << RoadSegmentID << " with TimeSlice: " << index << endl;
            for (itrPrint = timeRecords.begin(); itrPrint != timeRecords.end(); ++itrPrint) {
                for (int k=0;k<itrPrint->second.size();k++)
                {
                    cout << " time " << itrPrint->first << " routeID " << itrPrint->second[k][0];
                    cout << " status " << itrPrint->second[k][1] << " flow " << itrPrint->second[k][2] << "||";
                }
            }
            cout << endl;
        }*/

    }

    // threadf.join_all();

}

vector<int> Graph::one_route_update(pair<int, vector<int>> newRoute, int inTime){

    // Step 1: 初始化储存受影响的roadSegmentID的集合，把newRoute的路段添加进去
    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> roadSegmentSet; // RoadSegment, <0/leaveTime, I', D'>
    int RoadSegmentID1st = newRoute.second[0]; int count = 0;

    vector<int> affected_RoadSegmentID; affected_RoadSegmentID.clear();
    affected_RoadSegmentID.push_back(RoadSegmentID1st);


    cout << "=======================================================================" << endl;
    cout << "count: " << count << endl;


    //vMutex[RoadSegmentID1st].lock();
    roadSegmentSet = updateOperation1st(make_pair(RoadSegmentID1st,newRoute.first), inTime, newRoute);
    //vMutex[RoadSegmentID1st].unlock();

    /*
    cout << "roadSegmentSet Size is: " << roadSegmentSet.size() << endl;
    */

    // Step 2:  集合不为空的时候，
    pair<int, tuple<int, map<int, vector<vector<int>>>, vector<int>>> roadSegmentTemp;
    int leaveTime; map<int, vector<vector<int>>> insert; vector<int> deletion; pair<int,int> roadSegmentIDTemp;
    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> roadSegmentSetTemp;

    vector<int> passed_roadSegmentID; passed_roadSegmentID.clear();
    passed_roadSegmentID.push_back(newRoute.second[0]);

    while(roadSegmentSet.size() != 0){ // !roadSegmentSet.empty()
        count += 1;

        std::vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>>::iterator itr;
        itr = roadSegmentSet.begin();

        roadSegmentIDTemp = itr->first; leaveTime = get<0>(itr->second);

        vector<int>::iterator itr_findDup;
        itr_findDup = std::find (passed_roadSegmentID.begin(), passed_roadSegmentID.end(), roadSegmentIDTemp.first);

        if (roadSegmentIDTemp.first == -1 or itr_findDup != passed_roadSegmentID.end()){ // 或者已经计算过了
            roadSegmentSet.erase(itr);
            continue;
        }

        if (roadSegmentIDTemp.first != -1){


            cout << "=======================================================================" << endl;

            cout << endl;
            cout << "Step 1: round start" << endl;
            cout << "-----------------------------------------" << endl;
            cout << "count: " << count << endl;
            cout << "road segment ID is: " << roadSegmentIDTemp.first << " with leaveTime: " << leaveTime << endl;

            insert = get<1>(itr->second); deletion = get<2>(itr->second);
        }

        roadSegmentSet.erase(itr);

        cout << "Insert set size is: " << insert.size() << " Deletion size is: " << deletion.size() << endl;
        map<int, vector<vector<int>>>::iterator itrInsert;
        for (itrInsert = insert.begin(); itrInsert != insert.end();++itrInsert)
        {
            cout << "inserted time is: " << itrInsert->first << " with size: " << itrInsert->second.size() << endl;
        }


        if (count == 1 and roadSegmentIDTemp.first == 484213 and itrInsert->first == 4972){
            cout << endl;
        }

        //vMutex[roadSegmentIDTemp].lock();
        roadSegmentSetTemp = updateOperationFurther(roadSegmentIDTemp, leaveTime, newRoute, insert, deletion);
        //vMutex[roadSegmentIDTemp].unlock();

        affected_RoadSegmentID.push_back(roadSegmentIDTemp.first);

        passed_roadSegmentID.push_back(roadSegmentIDTemp.first);

        for (int i=0;i<roadSegmentSetTemp.size();i++){
            roadSegmentSet.push_back(roadSegmentSetTemp[i]);
        }

    }


    cout << "This Route Update Done." << endl;
    cout << "=======================================================================" << endl;


    return affected_RoadSegmentID;

}

void Graph::data_generation(vector<vector<int>> &route_data,  vector<vector<int>> &query_data,
                            string route_file, string depar_file, string Pi_file, int adj_num){

    vector<vector<int>> Pi_temp;
    for (int i = 0; i < adj_num; i++)
    {
        Pi_temp.push_back(route_data[i]);
    }

    ofstream outfile_Pi;
    outfile_Pi.open(Pi_file);

    for (int i = 0; i < Pi_temp.size(); i++)
    {
        outfile_Pi << Pi_temp[i].size() << " ";

        for (int j = 0; j < Pi_temp[i].size(); j++)
        {
            outfile_Pi << Pi_temp[i][j] << " ";
        }

        outfile_Pi << endl;
    }

    vector<vector<int>> new_path;
    for (int i = 0; i < adj_num; i++)
    {
        new_path.push_back(route_data[i]);
/*
        Pi.push_back(route_data[i]);
*/
    }


    vector<vector<int>> route_roadSegment;
    for (int i = 0; i < new_path.size(); i++)
    {
        route_roadSegment.push_back(route_node2roadSegment(new_path[i]));
    }

    //打开文件
    ofstream outfile_route;
    outfile_route.open(route_file);

    for (int i = 0; i < route_roadSegment.size(); i++)
    {
        outfile_route << route_roadSegment[i].size() << " ";

        for (int j = 0; j < route_roadSegment[i].size(); j++)
        {
            outfile_route << route_roadSegment[i][j] << " ";
        }

        outfile_route << endl;
    }

    outfile_route.close();

    ofstream outfile_depar;
    outfile_depar.open(depar_file);

    srand((unsigned)time(NULL));
    vector<int> depar_time_set; int depar_time, gener_depar_time;
    int time_range_right;
    for (int i = 0; i < route_roadSegment.size(); i++)
    {
        depar_time = query_data[i][2];
        time_range_right = depar_time + 2*60;
        gener_depar_time = (rand() % (time_range_right - depar_time + 1)) + depar_time;
//        depar_time_set.push_back(gener_depar_time);
        outfile_depar << gener_depar_time << " " << endl;
    }

    outfile_depar.close();

}


void Graph::one_route_update_check_debug(string route_file, string depar_file, string Pi_file){

    // 打开两个文件

    ifstream IF_route(route_file);
    if(!IF_route){
        cout<<"Cannot open Map "<<route_file<<endl;
    }

 //   int route_num = CountLines(route_file);

    vector<vector<int>> route_roadSegment; int vertex, ver_num;

    //for (int i = 0; i < route_num; i++)
    while(IF_route >> ver_num)
    {
        //IF_route >> ver_num;
        // route_roadSegment[i].resize(ver_num);
        vector<int> route(ver_num, 0);
        for (int j = 0; j < ver_num; j++)
        {
            IF_route >> route[j];

        }
        route_roadSegment.push_back(route);

    }

    vector<pair<int, vector<int>>> pNew_route;
    for (int i = 0; i < route_roadSegment.size(); i++)
    {
        pNew_route.push_back(make_pair(i+route_data_size, route_roadSegment[i]));
    }


    ifstream IF_depar(depar_file);
    if(!IF_depar){
        cout<<"Cannot open Map "<<depar_file<<endl;
    }

    int depar_num = CountLines(depar_file);

    vector<int> depar_time_set; int gener_depar_time;

    for (int i = 0; i < depar_num; i++)
    {
        IF_depar >> gener_depar_time;
        depar_time_set.push_back(gener_depar_time);
    }


    ifstream IF_Pi(Pi_file);
    if(!IF_Pi){
        cout<<"Cannot open Map "<<Pi_file<<endl;
    }

    int Pi_vertex, Pi_ver_num;

    //for (int i = 0; i < route_num; i++)
    while(IF_Pi >> Pi_ver_num)
    {
        //IF_route >> ver_num;
        // route_roadSegment[i].resize(ver_num);
        vector<int> route_Pi(Pi_ver_num, 0);
        for (int j = 0; j < Pi_ver_num; j++)
        {
            IF_Pi >> route_Pi[j];
        }
        Pi.push_back(route_Pi);
    }


    vector<int> affected_roadSegmentID; affected_roadSegmentID.clear();

    std::chrono::high_resolution_clock::time_point t0, t1;
    std::chrono::duration<double> time_span1;
    t0=std::chrono::high_resolution_clock::now();

/*
    for (int i = 0; i < depar_time_set.size(); i++)
*/
    for (int i = 0; i < depar_time_set.size(); i++)
    {

        affected_roadSegmentID = one_route_update(pNew_route[i], depar_time_set[i]);

        /*
        for (int j=0;j<affected_roadSegmentID.size();j++)
        {
            map<int, vector<vector<int>>> timeRecords; int inTime = 5296701;
            int RoadSegmentID = affected_roadSegmentID[j];

            int hour = Time2Hour(inTime); int index = Hour2Index(hour);
            timeRecords = timeRecordsChecked[RoadSegmentID][index];

            cout << endl;
            cout << "count " << j << endl;
            cout << "-----------------------------------------" << endl;
            map<int, vector<vector<int>>>::iterator itrPrint;

            cout << "TimeRecord Size is: " << timeRecords.size() << endl;
            cout << "RoadSegmentID: " << RoadSegmentID << " with TimeSlice: " << index << endl;
            for (itrPrint = timeRecords.begin(); itrPrint != timeRecords.end(); ++itrPrint) {
                for (int k=0;k<itrPrint->second.size();k++)
                {
                    cout << " time " << itrPrint->first << " routeID " << itrPrint->second[k][0];
                    cout << " status " << itrPrint->second[k][1] << " flow " << itrPrint->second[k][2] << "||";
                }
            }
            cout << endl;
        }
        */
    }

    t1=std::chrono::high_resolution_clock::now();
    time_span1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t0);
    cout << "\n" << "one route update time consumption without threads is: " << time_span1.count() << endl;
}


void Graph::one_route_update_check(vector<vector<int>> &route_data,  vector<vector<int>> &query_data, int ad_num){

    // 随机生成输入的新数据：路径 + 出发时间

    vector<vector<int>> new_path(ad_num);
    for (int i = 0; i < ad_num; i++)
    {
        new_path[i] = route_data[i];
    }

    vector<vector<int>> route_roadSegment(ad_num);
    for (int i = 0; i < new_path.size(); i++)
    {
        route_roadSegment[i] = route_node2roadSegment(new_path[i]);
    }

    vector<pair<int, vector<int>>> pNew_route(ad_num);
    for (int i = 0; i < route_roadSegment.size(); i++)
    {
        pNew_route[i] = make_pair(i, route_roadSegment[i]);
    }

    srand((unsigned)time(NULL));
    vector<int> depar_time_set; int depar_time, gener_depar_time;
    int time_range_right;
    for (int i = 0; i < pNew_route.size(); i++)
    {
        depar_time = query_data[i][2];
        time_range_right = depar_time + 2*60;
        gener_depar_time = (rand() % (time_range_right - depar_time + 1)) + depar_time;
        depar_time_set.push_back(gener_depar_time);
    }


    vector<int> affected_roadSegmentID; affected_roadSegmentID.clear();

    std::chrono::high_resolution_clock::time_point t0, t1;
    std::chrono::duration<double> time_span1;
    t0=std::chrono::high_resolution_clock::now();

    for (int i = 0; i < depar_time_set.size(); i++)
    {

        affected_roadSegmentID = one_route_update(pNew_route[i], depar_time_set[i]);

        /*
        for (int j=0;j<affected_roadSegmentID.size();j++)
        {
            map<int, vector<vector<int>>> timeRecords; int inTime = 5296701;
            int RoadSegmentID = affected_roadSegmentID[j];

            int hour = Time2Hour(inTime); int index = Hour2Index(hour);
            timeRecords = timeRecordsChecked[RoadSegmentID][index];

            cout << endl;
            cout << "count " << j << endl;
            cout << "-----------------------------------------" << endl;
            map<int, vector<vector<int>>>::iterator itrPrint;

            cout << "TimeRecord Size is: " << timeRecords.size() << endl;
            cout << "RoadSegmentID: " << RoadSegmentID << " with TimeSlice: " << index << endl;
            for (itrPrint = timeRecords.begin(); itrPrint != timeRecords.end(); ++itrPrint) {
                for (int k=0;k<itrPrint->second.size();k++)
                {
                    cout << " time " << itrPrint->first << " routeID " << itrPrint->second[k][0];
                    cout << " status " << itrPrint->second[k][1] << " flow " << itrPrint->second[k][2] << "||";
                }
            }
            cout << endl;
        }
        */
    }

    t1=std::chrono::high_resolution_clock::now();
    time_span1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t0);
    cout << "\n" << "one route update time consumption without threads is: " << time_span1.count() << endl;
}

vector<pair<pair<int, vector<int>>, int>> Graph::multi_task_initial(vector<vector<int>> &route_data,  vector<vector<int>> &query_data, int iNum){

    vector<pair<pair<int, vector<int>>, int>> multi_test;

    // vector<int> new_path = {97918, 278344, 99162, 96950, 99390, 97946};
    vector<vector<int>> new_path;
    for (int i = 0; i < iNum; i++)
    {
        new_path.push_back(route_data[i]);
        Pi.push_back(route_data[i]);
    }

    // vector<int> route_roadSegment = route_node2roadSegment(new_path);
    vector<vector<int>> route_roadSegment;
    for (int i = 0; i < new_path.size(); i++)
    {
        route_roadSegment.push_back(route_node2roadSegment(new_path[i]));
    }

    // pair<int, vector<int>> pNew_route = make_pair(0, route_roadSegment);
    vector<pair<int, vector<int>>> pNew_route;
    for (int i = 0; i < route_roadSegment.size(); i++)
    {
        pNew_route.push_back(make_pair(i, route_roadSegment[i]));
    }

    srand((unsigned)time(NULL));
    vector<int> depar_time_set; int depar_time, gener_depar_time;
    int time_range_right;
    for (int i = 0; i < pNew_route.size(); i++)
    {
        depar_time = query_data[i][2];
        time_range_right = depar_time + 2*60;
        gener_depar_time = (rand() % (time_range_right - depar_time + 1)) + depar_time;
        depar_time_set.push_back(gener_depar_time);
    }

    for (int i = 0; i < depar_time_set.size(); i++)
    {
        multi_test.push_back(make_pair(pNew_route[i], depar_time_set[i]));
    }

    return multi_test;
}

void Graph::one_route_parallel_update_check(vector<pair<pair<int, vector<int>>, int>> multi_test){

    std::chrono::high_resolution_clock::time_point t0, t1;
    std::chrono::duration<double> time_span1;
    t0=std::chrono::high_resolution_clock::now();

//    vector<int> new_path = {97918, 278344, 99162, 96950, 99390, 97946};
//    vector<int> route_roadSegment = route_node2roadSegment(new_path);
//    pair<int, vector<int>> pNew_route = make_pair(0, route_roadSegment);
//
//    vector<pair<pair<int, vector<int>>, int>> multi_test;
//    multi_test.push_back(make_pair(pNew_route, 5296690));
//    multi_test.push_back(make_pair(pNew_route, 5296685));
//    multi_test.push_back(make_pair(pNew_route, 5296680));
//    multi_test.push_back(make_pair(pNew_route, 5296675));
//    multi_test.push_back(make_pair(pNew_route, 5296701));

    multi_route_update(multi_test);

    t1=std::chrono::high_resolution_clock::now();
    time_span1 = std::chrono::duration_cast<std::chrono::duration<double>>(t1-t0);
    cout << "one route update time consumption with threads is: " << time_span1.count() << endl;

    /*
    // print manipulated time records.
    for (int i=0;i<affected_parallel_roadID.size();i++)
    {
        map<int, vector<vector<int>>> timeRecords; int inTime = 5296701;
        int RoadSegmentID = affected_parallel_roadID[i];

        int hour = Time2Hour(inTime); int index = Hour2Index(hour);
        timeRecords = timeRecordsChecked[RoadSegmentID][index];

        cout << endl;
        cout << "count " << i << endl;
        cout << "-----------------------------------------" << endl;
        map<int, vector<vector<int>>>::iterator itrPrint;

        cout << "TimeRecord Size is: " << timeRecords.size() << endl;
        cout << "RoadSegmentID: " << RoadSegmentID << " with TimeSlice: " << index << endl;
        for (itrPrint = timeRecords.begin(); itrPrint != timeRecords.end(); ++itrPrint) {
            for (int i=0;i<itrPrint->second.size();i++)
            {
                cout << " time " << itrPrint->first << " routeID " << itrPrint->second[i][0];
                cout << " status " << itrPrint->second[i][1] << " flow " << itrPrint->second[i][2] << "||";
            }
        }
        cout << endl;
    }
    */

}

void Graph::data_preparation(){


    // DATA PREPARATION
    // 读取BJ道路ID文件
    ReadRoadID(beijingNodeNew, BJ_nodeRoadID);

    // 读取北京路网文件
    ReadGraph(BJ);

    // 读取限速信息，并计算提取理想出行时间
    speedLim2vector(beijingRoadSpeedLimit);
    ExportminTime(BJ_nodeRoadID, BJ_minTravleTime);
    ReadNodeIDLonLat(beijingNodeNew, BJ_NodeIDLonLat);
    ReadRoadID(beijingNodeNew, BJ_nodeRoadID);
    ReadWeight(beijingNodeNew, BJ_NodeWeight);
    ReadRoadIDWeight(BJ_NodeWeight,BJ_nodeRoadID,BJ_RoadIDWeight);

    // 创建roadID的map：pingfu -> LeiLi
    CreateMap(beijingRoadMap);

    // 把轨迹数据的roadID做更新
    modify_trajectory(path1, path2, 1453651200, 97456);

    // 把roadID和nodeID做对应
    CreateRoadIDMapNodeID(BJ_nodeRoadID);

    // 提取NodeID1, NodeID2, time1, time2的轨迹数据
    export_trajectory(path2, path3, 97456);

    // 提取查询和轨迹点文件
    export_query_trajectory(path3,export_route,export_query, 97456);

    // 给两个相邻nodeID赋值新的roadID，从而(node01, node02)和(node02, node01)不共享roadID
    BJ_nodeRoadID_w_new_roadID(BJ_nodeRoadID, BJ_node_new_RoadID);

    // read routes by numerous single file
    int num_files = 110022;
    vector<vector<int>> Pi = ReadTrajectory(Base + "trajectory_data/export_route/31/result_", num_files);
    /* cout << "Pi size is: " << Pi.size() << endl; */

    // combine numerous route single file into one
    RouteCombine(Pi, Base + "route_combine/31_combine");
}

void Graph::map_construction(){

    //construct_map_nodeID_2_roadID(BJ_node_new_RoadID);
    //cout << "N2R and R2N map construction done." << endl;
    // map construction
    construct_map_nodeID_2_minTime(BJ_minTravleTime);
    cout << "min travel time construction done." << endl;


  //  construct_map_roadID_2_nodeID(BJ_node_new_RoadID);


}

void Graph::road_network_construction(){

    // read road network with length as weight
    ReadGraph(BJ);
    // read road network with time as weight
    RoadNetwork = ReadRoadNetwork(BJ_minTravleTime);
    cout << "road network construction done." << endl;

}

pair<vector<vector<int>>, vector<vector<int>>> Graph::read_query_route(string query_path, string route_path, int read_num){

    pair<vector<vector<int>>, vector<vector<int>>> query_route;

    // read queries
    /*vector<vector<int>> Q = ReadQuery(query_path);*/
    vector<vector<int>> Q = ReadQuery_w_num(query_path, read_num);
    query_route.first = Q;  int query_size = Q.size();

    // read route
    /*vector<vector<int>> Pi = ReadRoutes(route_path, query_size);*/
    vector<vector<int>> Pi = ReadRoutes_w_num(route_path, query_size, read_num);
    query_route.second = Pi;

    cout <<  "raw query data size is: " << query_size << " & raw route data size is: " << Pi.size() << endl;

    return query_route;

}