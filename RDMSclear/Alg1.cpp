//
// Created by 徐子卓 on 1/4/22.
//

#include "head.h"

Graph g;

void Graph::ReadGraph(string filename){

    /*
     * Description: read road network into program.
     *
     * Parameters:
     * string filename -> road network. node ID1, node ID2, weight (length in meters).
     *
     * Return:
     * void.
     */

    ifstream IF(filename);
    if(!IF){
        cout<<"Cannot open Map "<<filename<<endl;
    }

    IF>>nodenum>>edgenum; // 296710 774660

    eNum=0;
    set<pair<int,int>> eSet;

    vector<pair<int,int>> vecp; vecp.clear();
    Neighbor.assign(nodenum, vecp);
    NeighborRoad.assign(nodenum, vecp);
    vRoad.reserve(edgenum);

    set<int> setp; setp.clear();
    AdjacentNodes.assign(nodenum, setp);

    //to avoid the redundant information
    set<pair<int,int>> EdgeRedun;

    int ID1, ID2, weight;
    for(int i=0;i<edgenum;i++){
        IF>>ID1>>ID2>>weight;
        Road r;
        r.ID1 = ID1;
        r.ID2 = ID2;
        r.roadID = i;
        r.length = weight;
        r.travelTime = -1;
        vRoad.push_back(r);
        map_nodeID_2_roadID.insert(make_pair(make_pair(ID1, ID2), i));
        map_roadID_2_nodeID.insert(make_pair(i, make_pair(ID1, ID2)));
        map_nodeID_2_minTime.insert(make_pair(make_pair(ID1, ID2), -1));

        if(EdgeRedun.find(make_pair(ID1,ID2))==EdgeRedun.end()){
            Neighbor[ID1].push_back(make_pair(ID2, weight));
            NeighborRoad[ID1].push_back(make_pair(ID2, i));
            AdjacentNodes[ID1].insert(ID2);
        }
        EdgeRedun.insert(make_pair(ID1,ID2));
    }

    /*
    // print road network
    for (int i=0;i<Neighbor.size();i++){
        cout << i << ": ";
        for (int j=0;j<Neighbor[i].size();j++){
            cout << Neighbor[i][j].first << " " << Neighbor[i][j].second << " ";
        }
        cout << endl;
    }
     */

}

int Graph::Dij(int ID1, int ID2){

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

    if(ID1==ID2) return 0;
    //if(NodeOrder[ID1]==-1 || NodeOrder[ID2]==-1) return INF;
    benchmark::heap<2, int, int> pqueue(nodenum);
    pqueue.update(ID1,0);

    vector<bool> closed(nodenum, false);
    vector<int> distance(nodenum, INF);

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
                }
            }
        }
    }
    return d;
}

void Graph::ReadRoadID(string filename, string export_filename){

    /*
     * Description: export node ID and road ID from original file "beijingNodeNew".
     *
     * Parameters:
     * string filename -> original file "beijingNodeNew".
     * string export_filename -> exported file: node ID1, node ID2, road ID
     *
     * Return:
     * void.
     */

    ifstream IF(filename);
    if(!IF){
        cout<<"Cannot open Map "<<filename<<endl;
    }

    ofstream outfile;
    outfile.open(export_filename);

    int nodenum; // 296710
    float lon1, lon2, lat1, lat2;
    IF >> nodenum >> lon1 >> lat1 >> lon2 >> lat2;
    outfile << nodenum << " " << endl;

    int ID1, unk1, unk2, tol1, tol2, tol3, ID2, dist, roadID;
    float lon, lat;

    int total = 0;

    for(int i=0; i<nodenum; i++){
        IF >> ID1 >> unk1 >> lon >> lat >> unk2;

        IF >> tol1;
        for(int j=0;j<tol1;j++){
            IF >> ID2 >> dist;
        }
        total = total + tol1;

        IF >> tol2;
        for(int k=0; k<tol2; k++){
            IF >> ID2 >> roadID;
            outfile << ID1 << " ";
            outfile << ID2 << " " << roadID << endl;
        }

        IF >> tol3;
        for(int l=0;  l<tol3; l++){
            IF >> ID2 >> roadID;
        }
    }
    outfile.close();
}

void Graph::ReadWeight(string filename, string export_filename){

    /*
     * Description: export node ID and weight from original file "beijingNodeNew".
     *
     * Parameters:
     * string filename -> original file "beijingNodeNew".
     * string export_filename -> exported file: node ID1, node ID2, weight (length in meters).
     *
     * Return:
     * void.
     */

    ifstream IF(filename);
    if(!IF){
        cout<<"Cannot open Map "<<filename<<endl;
    }

    ofstream outfile;
    outfile.open(export_filename);

    int nodenum; // 296710
    float lon1, lon2, lat1, lat2;
    IF >> nodenum >> lon1 >> lat1 >> lon2 >> lat2;
    outfile << nodenum << " " << endl;

    int ID1, unk1, unk2, tol1, tol2, tol3, ID2, dist, roadID;
    float lon, lat;

    for(int i=0; i<nodenum; i++){
        IF >> ID1 >> unk1 >> lon >> lat >> unk2;

        IF >> tol1;
        for(int j=0;j<tol1;j++){
            IF >> ID2 >> dist;
            outfile << ID1 << " ";
            outfile << ID2 << " " << dist << endl;
        }

        IF >> tol2;
        for(int k=0; k<tol2; k++){
            IF >> ID2 >> roadID;
        }

        IF >> tol3;
        for(int l=0;  l<tol3; l++){
            IF >> ID2 >> roadID;
        }
    }

    outfile.close();
}

void Graph::ReadRoadIDWeight(string filename1, string filename2, string export_filename){

    /*
     * Description: export node ID, road ID and weight in a file.
     *
     * Parameters:
     * string filename1 -> BJ_NodeWeight: node ID1, node ID2, weight (length in meters).
     * string filename1 -> BJ_nodeRoadID: node ID1, node ID2, road ID.
     * string export_filename -> exported file: node ID1, node ID2, road ID, weight (length in meters).
     *
     * Return:
     * void.
     */

    ifstream IF1(filename1);
    if(!IF1){
        cout<<"Cannot open Map "<<filename1<<endl;
    }

    ifstream IF2(filename2);
    if(!IF2){
        cout<<"Cannot open Map "<<filename2<<endl;
    }

    ofstream outfile;
    outfile.open(export_filename);

    int nodenum1; // 296710
    int nodenum2; // 296710

    IF1 >> nodenum1;
    IF2 >> nodenum2;

    outfile << nodenum1 << " " << endl;

    int ID1_1, ID1_2, ID2_1, ID2_2, value1, value2;

    int lines = CountLines(filename1);

    for(int i=0; i<lines-1; i++){
        IF1 >> ID1_1 >> ID1_2 >> value1;
        IF2 >> ID2_1 >> ID2_2 >> value2;

        outfile << ID1_1 << " " << ID1_2 << " " << value2 << " " << value1 << " " << endl;
    }
    outfile.close();
}

void Graph::ReadNodeIDLonLat(string filename, string export_filename){

    /*
     * Description: export node ID's latitude and longitude from original file "beijingNodeNew".
     *
     * Parameters:
     * string filename -> original file "beijingNodeNew".
     * string export_filename -> exported file: node ID, longitude, latitude.
     *
     * Return:
     * void.
     */

    ifstream IF(filename);
    if(!IF) {
        cout << "Cannot open Map " << filename << endl;
    }

    ofstream outfile;
    outfile.open(export_filename);

    int nodenum; // 296710
    float lon1, lon2, lat1, lat2;
    IF >> nodenum >> lon1 >> lat1 >> lon2 >> lat2;
    outfile << nodenum << " " << endl;

    int ID1, unk1, unk2, tol1, tol2, tol3, ID2, dist, roadID;
    float lon, lat;

    for(int i=0; i<nodenum; i++){
        IF >> ID1 >> unk1 >> lat >> lon >> unk2;
        outfile << ID1 << " " << lon << " " << lat << " " << endl;

        IF >> tol1;
        for(int j=0;j<tol1;j++){
            IF >> ID2 >> dist;
        }

        IF >> tol2;
        for(int k=0; k<tol2; k++){
            IF >> ID2 >> roadID;
        }

        IF >> tol3;
        for(int l=0;  l<tol3; l++){
            IF >> ID2 >> roadID;
        }
    }

    outfile.close();
}

void Graph::speedLim2vector(string filename){

    /*
     * Description: create vector to store speed limitation.
     *
     * Parameters:
     * string filename -> speed limitations file: roadID, speed limitaion (in km/h)
     *
     * Return:
     * void.
     */

    ifstream IF(filename);
    if(!IF){
        cout<<"Cannot open Map "<<filename<<endl;
    }

    int nodenum; // 387587 roadID
    IF >> nodenum;

    speedLim.assign(nodenum, INF);

    int roadID, speed;
    for(int i=0;i<nodenum+1;i++){
        IF >> roadID >> speed;
        speedLim[roadID] = speed;
    }
}

void Graph::ExportminTime(string filename, string export_filename){

    /*
     * Description: create vector to store speed limitation.
     *
     * Parameters:
     * string filename -> file contains road ID and related two node IDs: node ID1, node ID2, road ID
     * string export_filename) -> exported file: node ID1, node ID2, min travel time (round as int)
     *
     * Return:
     * void.
     */

    ifstream IF(filename);
    if(!IF){
        cout<<"Cannot open Map "<<filename<<endl;
    }

    ofstream outfile;
    outfile.open(export_filename);

    int nodenum;
    IF >> nodenum;
    outfile << nodenum << " " << endl;

    int ID1, ID2, roadID;
    int distance;

    int lines = CountLines(filename);

    for (int i = 0; i<lines-1; i++){
        IF >> ID1 >> ID2 >> roadID;

        for (int j=0;j<Neighbor[ID1].size();j++){
            if(Neighbor[ID1][j].first == ID2){
                distance = Neighbor[ID1][j].second;
            }
        }

        int TspeedLim = speedLim[roadID] / 3.6;
        int minTime = distance / TspeedLim;

        outfile << ID1 << " " << ID2 << " ";
        outfile << minTime << endl;
    }

    outfile.close();
}

//vector<vector<int>> Graph::Toward(string filename, vector<vector<int>> Q){
//    // 读取数据中的node数量和edge数量
//    ifstream IF(filename);
//    if(!IF){cout<<"Cannot open Map "<<filename<<endl;}
//    IF>>nodenum>>edgenum;
//
//    // 定义优先队列H
//    benchmark::heap<2, vector<int>, int> H(Q.size()); // label ID, departure time
//    // 定义储存路径的vector Pi(Qxn)
//    vector<vector<int>> Pi(Q.size());
//    // 定义return的最终结果Rpi(Qxn)
//    vector<vector<int>> Rpi(Q.size());
//    // 定义查询的label(Qx3)
//    vector<vector<int>> label(Q.size());
//    // 定义储存真实出行时间的Tpi(Qxn)
//    vector<vector<int>> Tpi;
//
//    // 初始化每个查询的label，并添加到优先队列H
//    for (int i=0;i<Q.size();i++){
//        // 每个初始label(Qx3)包含：label的index，label的起始点，label的起始时间
//        label[i].push_back({i,Q[i][0],Q[i][2]});
//        // 储存走过路线(Qxn)
//        Pi[i].push_back({Q[i][0]});
//
//        // 添加所有的label到优先队列H中，根据label的起始时间排序
//        H.update(label[i],label[i][2]); // ?
//    }
//
//    // topLabel大小(1x3), topTime表示优先级最高的起始时间
//    int topLabel, topTime;
//    // NNodeID表示邻居ID的值，NTime表示从当前NodeID到NNodeID的最优时间
//    int NNodeID, NTime;
//    // 储蓄更新的查询的label信息
//    vector<int> label_1;
//
//    // 储存当前路段上的车辆数量
//    vector<vector<pair<int,int>>> flow;
//
//    // 初始化路段上车辆数量，每个路段初始化数量为0
//    for(int i=0;i<nodenum;i++){
//        flow[i] = Neighbor[i];
//        for(int j=0;j<Neighbor[i].size();j++){
//            flow[i][j].first = Neighbor[i][j].first;
//            flow[i][j].second = 0;
//        }
//    }
//
//    // 当优先队列H不为空，执行循环
//    while(!H.empty()){
//        // 选择优先队列中优先级别最高的查询对应的label
//        H.extract_min(topLabel, topTime);
//        // queryIndex表示查询的label的ID
//        int queryIndex = topLabel[0];
//        // topNodeID表示当前优先级最高的NodeID
//        int topNodeID = topLabel[1];
//
//        // 遍历当前node，topNodeID的邻居节点，从中选择最优解
//        for(int i=0; i<Neighbor[topNode].size();i++){
//            NNodeID=Neighbor[topNode][i].first; // NNodeID表示邻居ID的值
//            NTime=Neighbor[topNode][i].second; // NTime表示从当前NodeID到NNodeID的最优时间
//
//            label_1[0]=queryIndex; // 更新label的index
//            label_1[1]=NNodeID; // 更新label的node到邻居节点
//            label_1[2]=topLabel[2]; // 暂时保留起始时间为为更新的起始时间，之后通过计算更新
//            Pi[queryIndex].push_back({NNodeID}); // 更新label的行驶路线
//
//            // 如果行驶路线不包含当前邻居节点，并且从邻居点的到d的最短行驶时间小于当前点到d的行驶时间
//            if (!(contains(Pi[queryIndex], NNodeID)) and Dij(NNodeID,Q[queryIndex][1]) < Dij(topNodeID,Q[queryIndex][1])){
//                // 在当前路径下添加traffic flow数量
//                flow[topNodeID][i].second += 1;
//                // 设置Tflow储存当前路段的traffic flow
//                int Tflow = flow[topNodeID][i].second;
//                // 设置根据traffic flow估计出行时间的参数，并计算estimated time
//                int alpha = 0.1;
//                int Etime = NTime*(1+alpha*Tflow);
//                // 计算全局出行时间EGT
//                vector<int> EGT[i] = accmulate(Tpi[queryIndex]) + Etime + Dij(NNodeID,Q[queryIndex][1]);
//
//                // 找到能够实现最小全局出行的邻居node
//                int minValue = *min_element(EGT.begin(),EGT.end());
//
//                if (EGT[i]==minValue){
//                    label_1[2] = label[queryIndex][2] + Etime
//                }
//            }
//        }
//        // 更新查询的label
//        label[queryIndex] = label_1;
//        // 把预估的时间添加到esitmated time中
//        Tpi[queryIndex].push_back({t});
//
//        // 判断当前label是否走到destination point，如果到达，把路径添加到Rpi中
//        if (Pi[queryIndex].back()==Q[queryIndex][1]){
//            Rpi[queryIndex].push_back(Pi[queryIndex]);
//        }
//        // 否则，把更新查询的label添加到优先队列H中
//        else{
//            H.update(label[i],label[i][2]);
//        }
//    }
//    return Rpi;
//}

void Graph::CreateMap(string filename){

    /*
     * Description: create a map for road ID to transfer road ID from Pingfu's version to us.
     *
     * Parameters:
     * string filename -> file with two versions road ID: road ID version 1, road ID version 2.
     *
     * Return:
     * void.
     */

    ifstream IF(filename);
    if(!IF){
        cout<<"Cannot open Map "<<filename<<endl;
    }

    int lines = CountLines(filename);

    int roadID1, roadID2;

    for (int i=0;i<lines;i++){
        IF >> roadID1 >> roadID2;
        mapRoadID.insert(map<int, int>::value_type (roadID1, roadID2));
    }

    /* find and print single values

    map<int, int>::iterator iter;

    iter = mapRoadID.find(89929037);

    if(iter != mapRoadID.end())
        cout<<"Find, the value is "<<iter->second<<endl;
    else
        cout<<"Do not Find"<<endl;
    */

    /* print all values

    map<int, int>::iterator iter;

    for(iter = mapRoadID.begin(); iter != mapRoadID.end(); iter++)
        cout<<iter->first<<' '<<iter->second<<endl;
     */

}

void Graph::modify_trajectory(string path1, string path2, int time, int fileNum){

    /*
     * Description: transfer trajectory data by mapping road ID from Pingfu's version to our version, and transfer time
     * from UTC time to Beijing time zone which start from 2016.03.01 00:00:00
     *
     * Parameters:
     * string path1 -> path contains original trajectory data: road ID in Pingfu's version, time 1, time 2.
     * string path2 -> path to store converted trajectory data:  road ID in our version, time 1, time 2.
     * int time -> change time into smaller num in the same day.
     * int fileNum -> number of files.
     *
     * Return:
     * void.
     */

    // 读取文件
    for (int i=0; i<fileNum; i++){

        string index = to_string(i);
        string data_name = path1 + index + ".txt";
        string export_filename = path2 + index + ".txt";

        // 打开文件，如果文件不存在继续往下读取
        ifstream file_name(data_name);
        if (!file_name.is_open()){
            cout << "file " << (index+".txt") << " doesn't exist." << endl;
            continue;
        }

        ofstream outfile; outfile.open(export_filename);

        // 计算该轨迹文件有多少行
        int lines = CountLines(data_name);

        // 更改该轨迹文件每行的时间从UTC到北京
        for (int j=0;j<lines;j++){
            int roadID, roadIDBeijing;
            time_t time1, time2;

            file_name >> roadID >> time1 >> time2;

            // find and print single values
            map<int, int>::iterator iter;
            iter = mapRoadID.find(abs(roadID));
            if(iter != mapRoadID.end())
                if(roadID<0){
                    roadIDBeijing = -1 * (iter->second);
                }
                else{
                    roadIDBeijing = iter->second;
                }
            else
                cout<<"Do not Find"<<endl;

            if (abs(roadIDBeijing) > 387587){
                cout << "roadID in this file: " << (index+".txt") << " is overloaded" << endl;
                break;
            }

            // 把UTC时间转换成北京时间
            time1 = time1 + 8*60*60 - time;
            time2 = time2 + 8*60*60 - time;

            // 读入储存文件
            outfile << roadIDBeijing << " " << time1 << " " << time2 << " " << endl;
        }

        // 关闭储存文件
        file_name.close();
        // 关闭储存文件
        outfile.close();
    }
}

void Graph::CreateRoadIDMapNodeID(string filename){

    /*
     * Description: create a map for road ID to node ID.
     *
     * Parameters:
     * string filename -> file contains road ID and related node ID.
     *
     * Return:
     * void.
     */

    ifstream IF(filename);
    if(!IF){
        cout<<"Cannot open Map "<<filename<<endl;
    }

    int lines = CountLines(filename);

//    map<int, int> mapStudent;

    int nodenum, NodeID1, NodeID2, RoadID;

    IF >> nodenum;


    for (int i=0;i<lines;i++){
        IF >> NodeID1 >> NodeID2 >> RoadID;
        mapRoadIDNodeID.insert(map<int, pair<int,int>>::value_type (RoadID, make_pair(NodeID1, NodeID2)));
    }

//    map<int, pair<int, int>>::iterator iter;
//
//    for(iter = mapRoadIDNodeID.begin(); iter != mapRoadIDNodeID.end(); iter++)
//
//        cout<<iter->first<<' '<<(iter->second).first<<" "<<(iter->second).second<<endl;

}

void Graph::export_trajectory(string path1, string path2, int fileNum){

    /*
     * Description: convert trajectory data (road ID with related time) to trajectory data (node ID with related data).
     *
     * Parameters:
     * string path1 -> trajectory data contains road ID, time 1, time 2.
     * string path2 -> trajectory data contains node ID1, node ID2, time 1, time 2.
     *
     * Return:
     * void.
     */

    // 读取文件
    for (int i=0; i<fileNum; i++){

        string index = to_string(i);
        string data_name = path1 + index + ".txt";
        string export_filename = path2 + index + ".txt";

        // 打开文件，如果文件不存在继续往下读取
        ifstream file_name(data_name);
        if (!file_name.is_open()){
            cout << "file " << (index +".txt") << " doesn't exist." << endl;
            continue;
        }

        ofstream outfile; outfile.open(export_filename);

        // 计算该轨迹文件有多少行
        int lines = CountLines(data_name);

        //
        for (int j=0;j<lines;j++){
            int roadID, NodeID1, NodeID2;
            time_t time1, time2;

            file_name >> roadID >> time1 >> time2;

            // find and print single values
            map<int, pair<int,int>>::iterator iter;
            iter = mapRoadIDNodeID.find(abs(roadID));
            if(iter != mapRoadIDNodeID.end()){

                NodeID1 = (iter->second).first;
                NodeID2 = (iter->second).second;
            }
            else{
                cout<<"Do not Find II"<<endl;
            }

            // 读入储存文件
            outfile << NodeID1 << " " << NodeID2 << " " << time1 << " " << time2 << " " << endl;
        }
        // 关闭储存文件
        file_name.close();
        // 关闭储存文件
        outfile.close();
    }
}

void Graph::export_query_trajectory(string path1, string path2, string filename, int fileNum){

    /*
     * Description: export trajectory vertices and queries from trajectory data.
     *
     * Parameters:
     * string path1 -> path of trajectory data: node ID1, node ID2, time1, time2.
     * string path2 -> path to store trajectory vertices: node ID.
     * string filename -> path to store queries: source node ID, departure time, destination node ID.
     *
     * Return:
     * void.
     */

    string export_query = filename;

    ofstream outfile_query;
    outfile_query.open(export_query);

    for (int i=0; i<fileNum; i++){

        string index = to_string(i);
        string data_name = path1 + index + ".txt";
        string export_trajectory = path2 + index + ".txt";


        // 打开文件，如果文件不存在继续往下读取
        ifstream file_name(data_name);
        if (!file_name.is_open()){
            cout << "file " << (index +".txt") << " doesn't exist." << endl;
            continue;
        }

        ofstream outfile_trajectory;
        outfile_trajectory.open(export_trajectory);

        // 计算该轨迹文件有多少行
        int lines = CountLines(data_name);

        // 提取前两行内容
        int depNodeID1, depNodeID2, NodeID1, NodeID2;
        time_t deptime1, deptime2, time1, time2;
        file_name >> depNodeID1 >> depNodeID2 >> deptime1 >> deptime2;
        file_name >> NodeID1 >> NodeID2 >> time1 >> time2;

        int CDiff_Node;

        if(depNodeID1==NodeID1 or depNodeID1==NodeID2){
            outfile_trajectory << depNodeID2 << endl;
            outfile_trajectory << depNodeID1 << endl;
            if(NodeID1 == depNodeID1){
                outfile_trajectory << NodeID2 << endl;
                CDiff_Node = NodeID2;
            }
            else{
                outfile_trajectory << NodeID1 << endl;
                CDiff_Node = NodeID1;
            }
            outfile_query << depNodeID2 << " " << deptime1 << " ";
        }
        else{
            outfile_trajectory << depNodeID1 << endl;
            outfile_trajectory << depNodeID2 << endl;
            if(NodeID1 == depNodeID2){
                outfile_trajectory << NodeID2 << endl;
                CDiff_Node = NodeID2;
            }
            else{
                outfile_trajectory << NodeID1 << endl;
                CDiff_Node = NodeID1;
            }
            outfile_query << depNodeID1 << " " << deptime1 << " ";
        }

        // 中间行的内容
        int NNodeID1, NNodeID2;
        time_t Ntime1, Ntime2;
        for (int j=0;j<lines-2;j++){

            file_name >> NNodeID1 >> NNodeID2 >> Ntime1 >> Ntime2;

            if(NNodeID1 == CDiff_Node){
                outfile_trajectory << NNodeID2 << endl;
                CDiff_Node = NNodeID2;
            }
            else{
                outfile_trajectory << NNodeID1 << endl;
                CDiff_Node = NNodeID1;
            }
        }

        outfile_query << CDiff_Node << endl;



        // 关闭储存文件
        file_name.close();
        // 关闭储存文件
        outfile_trajectory.close();
    }
    outfile_query.close();
}

vector<vector<int>> Graph::ReadQuery(string filename){

    /*
     * Description: read data from query file into a variable.
     *
     * Parameters:
     * string filename -> path to query file.
     *
     * Return:
     * vector<vector<int>> -> variable store query information.
     */

    vector<vector<int>> Q;

    ifstream file_name(filename);

    if(!file_name){cout<<"Cannot open Map "<<filename<<endl;}

/*    int lines = CountLines(filename);*/

    int DepartureID, DestinationID, DepartureTime;

    while(file_name >> DepartureID)
    {
        file_name >> DepartureTime >> DestinationID;
        vector<int> query_temp = {DepartureID,DepartureTime,DestinationID};

        //for (int i = 0; i < 3; i++)
       // {
         //   file_name >> query_temp[i];

        //}
        Q.push_back(query_temp);
    }

/*    for (int i=0;i<lines;i++){

        file_name >> DepartureID >> DepartureTime >> DestinationID;

        Q.push_back({DepartureID,DestinationID,DepartureTime});
    }*/

    // 关闭储存文件
    file_name.close();

    return Q;
}

vector<vector<int>> Graph::ReadQuery_w_num(string filename, int num){

    /*
     * Description: read data from query file into a variable.
     *
     * Parameters:
     * string filename -> path to query file.
     *
     * Return:
     * vector<vector<int>> -> variable store query information.
     */

    vector<vector<int>> Q;

    ifstream file_name(filename);

    if(!file_name){cout<<"Cannot open Map "<<filename<<endl;}

    int lines = CountLines(filename);

    int input_num;

    if (num == 0){
        input_num = lines;
    }else{
        input_num = num;
    }

    for (int i=0;i<input_num;i++){
        int DepartureID, DestinationID, DepartureTime;

        // 针对老数据形式
        /*file_name >> DepartureID >> DepartureTime >> DestinationID;*/

        // 针对新数据形式
        file_name >> DepartureID >> DestinationID >> DepartureTime;

        Q.push_back({DepartureID,DestinationID,DepartureTime});
    }

    // 关闭储存文件
    file_name.close();

    return Q;
}

vector<vector<int>> Graph::ReadTrajectory(string path, int fileNum){

    /*
     * Description: read data from trajectory file into a variable.
     *
     * Parameters:
     * string filename -> path to trajectory file.
     *
     * Return:
     * vector<vector<int>> -> variable store trajectory information.
     */

    vector<vector<int>> Pi;

    for (int i=0; i<fileNum; i++){

        string index = to_string(i);
        string data_name = path + index + ".txt";

        // 打开文件，如果文件不存在继续往下读取
        ifstream file_name(data_name);
        if (!file_name.is_open()){
//            cout << "file " << (index +".tx\t") << " doesn't exist." << endl;
            continue;
        }

        int Vertex;
        vector<int> pi;

        int lines = CountLines(data_name);

        for (int i=0;i<lines;i++) {
            file_name >> Vertex;
            pi.push_back(Vertex);
        }

        Pi.push_back(pi);

        // 关闭储存文件
        file_name.close();
    }
    return Pi;
}

vector<vector<pair<int,int>>> Graph::ReadRoadNetwork(string filename){

    /*
     * Description: read road network into program.
     *
     * Parameters:
     * string filename -> road network. node ID1, node ID2, min travel time (length in s).
     *
     * Return:
     * vector<vector<pair<int,int>>> -> variable stores road network.
     */

    ifstream IF(filename);
    if(!IF){
        cout<<"Cannot open Map "<<filename<<endl;
    }

    int nodenum;
    IF>>nodenum; // 296710 774660

    eNum=0;
    set<pair<int,int>> eSet;

    vector<pair<int,int>> vecp; vecp.clear();
    vector<vector<pair<int,int>>> RoadNetwork;
    RoadNetwork.assign(nodenum, vecp);

    set<int> setp; setp.clear();
    AdjacentNodes.assign(nodenum, setp);

    //to avoid the redundant information
    set<pair<int,int>> EdgeRedun;

    int ID1, ID2;
    double weight;
    for(int i=0;i<edgenum;i++){
        IF>>ID1>>ID2>>weight;
        weight = (int) weight;
        if(EdgeRedun.find(make_pair(ID1,ID2))==EdgeRedun.end()){
            RoadNetwork[ID1].push_back(make_pair(ID2, weight));
            AdjacentNodes[ID1].insert(ID2);
        }
        EdgeRedun.insert(make_pair(ID1,ID2));
    }

//    for (int i=0;i<RoadNetwork.size();i++){
//        cout << i << ": ";
//        for (int j=0;j<RoadNetwork[i].size();j++){
//            cout << RoadNetwork[i][j].first << " " << RoadNetwork[i][j].second << " ";
//        }
//        cout << endl;
//    }

    return RoadNetwork;
}

//vector<vector<pair<int, int>>> Graph::Algorithm1(vector<vector<pair<int, int>>> &G,
//                                                 vector<vector<int>> &Q, vector<vector<int>> &Pi){
//    /*
//     * Description: This function is the baseline of traffic trajectory project algorithm I.
//     * It calculates the estimated travel time on each trajectory's rodd segment,
//     * by considering the affect of traffic congestion based on traffic flow.
//     *
//     * Parameters:
//     * &G -> road network.
//     * &Q -> queries. contains queries and each query contains source node, destination node, and departure time.
//     * &Pi -> trajectories. vertices passed by each query.
//     *
//     * Return:
//     * GResult -> estimated travel time on each trajectory's rodd segment
//     */
//
//    // Initialization
//    benchmark::heap<2, int, int> H(Q.size());
//    vector<vector<int>> label(Q.size());
//    vector<vector<pair<int,int>>> GResult = G;
//    vector<vector<pair<int,int>>> F = G;
//
//    // initialize each weight as infinity
//    for (int i=0;i<GResult.size();i++){
//        for (int j=0;j<GResult[i].size();j++){
//            GResult[i][j].second = INF;
//        }
//    }
//
//    // initialize labels and update into priority queue
//    int NodeIndex = 0;
//    for (int i=0;i<Q.size();i++){
//        label[i] = {i,NodeIndex,Q[i][2]};
//        H.update(i,label[i][2]); // query index, departure time
//    }
//
//    // initialize traffic flow on each road segment
//    for (int i=0;i<F.size();i++){
//        for (int j=0;j<F[i].size();j++){
//            F[i][j].second = 0;
//        }
//    }
//
//    int CLabelIndex, CTime, CNodeIndex;
//    int count  = 0;
//    while (!H.empty()){
//        H.extract_min(CLabelIndex, CTime);
//
//        int CDesNode = Q[CLabelIndex][1];
//        CNodeIndex = label[CLabelIndex][1];
//        int CNode = Pi[CLabelIndex][CNodeIndex];
//
//        // define parameter of traffic flow function
//        float sigma = 0.15;
//        float varphi = 20;
//        float beta = 2;
//
//        if( CNode == CDesNode){
//            continue;
//        }
//        else{
//            int NextNode = Pi[CLabelIndex][CNodeIndex+1];
//            // ID1: ID2 w ID3 w
//            if (CNodeIndex == 0){
//                for (int i=0;i<F[CNode].size();i++){
//                    if(F[CNode][i].first == NextNode){
//                        F[CNode][i].second = F[CNode][i].second + 1;
//
//                        int tm = G[CNode][i].second;
//                        int Cflow = F[CNode][i].second;
//                        int te = tm * (1 + sigma * pow(Cflow/varphi, beta));
//
//                        label[CLabelIndex][2] = label[CLabelIndex][2] + te;
//                        label[CLabelIndex][1] = label[CLabelIndex][1] + 1;
//                        GResult[CNode][i].second = te;
//                        H.update(label[CLabelIndex][0], label[CLabelIndex][2]);
//                        int a = label[CLabelIndex][2];
//                        cout << label[CLabelIndex][2] << endl;
//                    }
//                }
//            }
//            else{
//                int PreviousNode = Pi[CLabelIndex][CNodeIndex-1];
//
//                for (int i=0;i<F[PreviousNode].size();i++){
//                    if(F[PreviousNode][i].first == CNode){
//                        F[PreviousNode][i].second = F[PreviousNode][i].second - 1;
//                    }
//                }
//
//                for (int i=0;i<F[CNode].size();i++){
//                    if(F[CNode][i].first == NextNode){
//                        F[CNode][i].second = F[CNode][i].second + 1;
//
//                        int tm = G[CNode][i].second;
//                        int Cflow = F[CNode][i].second;
//                        int te = tm * (1 + sigma * pow(Cflow/varphi, beta));
//
//                        label[CLabelIndex][2] = label[CLabelIndex][2] + te;
//                        label[CLabelIndex][1] = label[CLabelIndex][1] + 1;
//                        GResult[CNode][i].second = te;
//                        H.update(label[CLabelIndex][0], label[CLabelIndex][2]);
//                        int a = label[CLabelIndex][2];
//                        cout << label[CLabelIndex][2] << endl;
//                    }
//                }
//            }
//        }
//    }
//    return GResult;
//}

vector<vector<pair<int,int>>> Graph::Algorithm1(vector<vector<pair<int, int>>> &G,
                                                vector<vector<int>> &Q, vector<vector<int>> &Pi){
    /*
     * Description: This function is the baseline of traffic trajectory project algorithm I.
     * It calculates the estimated travel time on each trajectory's road segment,
     * by considering the affect of traffic congestion based on traffic flow.
     *
     * Parameters:
     * &G -> road network.
     * &Q -> queries. contains queries and each query contains source node, destination node, and departure time.
     * &Pi -> trajectories. vertices passed by each query.
     *
     * Return:
     * GResult -> estimated travel time on each trajectory's road segment
     */

    // Initialization
    benchmark::heap<2, int, int> H(Q.size());
    vector<vector<int>> label(Q.size());

    vector<vector<pair<int,int>>> GResult(Pi.size());
    for (int i=0;i<Pi.size();i++){
        GResult[i].resize(Pi[i].size());
    }

    /*
     * reserve,assign
     */

    vector<vector<pair<int,int>>> F = G;

    // initialize each weight as infinity
    for (int i=0;i<Pi.size();i++){
        for (int j=0;j<1;j++){
            GResult[i][j].first = Pi[i][j];
            GResult[i][j].second = 0;
        }
        for (int k=1;k<Pi[i].size();k++){
            GResult[i][k].first = Pi[i][k];
            GResult[i][k].second = INF;
        }
    }

    // initialize labels and update into priority queue
    int NodeIndex = 0;
    for (int i=0;i<Q.size();i++){
        label[i] = {i,NodeIndex,Q[i][2]};
        H.update(i,label[i][2]); // query index, departure time
    }

    // initialize traffic flow on each road segment
    for (int i=0;i<F.size();i++){
        for (int j=0;j<F[i].size();j++){
            F[i][j].second = 0;
        }
    }

    int CLabelIndex, CTime, CNodeIndex;
    while (!H.empty()){
        H.extract_min(CLabelIndex, CTime);

        CNodeIndex = label[CLabelIndex][1];
        int CNode = Pi[CLabelIndex][CNodeIndex];

//        if (CLabelIndex == 67610){
//            cout << "CNodeIndex is: " << CNodeIndex << endl;
//        }

        // define parameter of traffic flow function
        float sigma = 0.15;
        float varphi = 20;
        float beta = 2;

        if (CNodeIndex == (Pi[CLabelIndex].size()-1)){

            int PreviousNode = Pi[CLabelIndex][CNodeIndex-1];

            for (int i=0;i<F[PreviousNode].size();i++){
                if(F[PreviousNode][i].first == CNode){
                    F[PreviousNode][i].second = F[PreviousNode][i].second - 1;
                }
            }

            continue;
        }
        else{
            int NextNode = Pi[CLabelIndex][CNodeIndex+1];
            if (CNodeIndex == 0){
                for (int i=0;i<F[CNode].size();i++){
                    if(F[CNode][i].first == NextNode){
                        F[CNode][i].second = F[CNode][i].second + 1;

                        int tm = G[CNode][i].second;
                        int Cflow = F[CNode][i].second;
                        int te = tm * (1 + sigma * pow(Cflow/varphi, beta));

                        label[CLabelIndex][2] = label[CLabelIndex][2] + te;
                        label[CLabelIndex][1] = label[CLabelIndex][1] + 1;
                        GResult[CLabelIndex][CNodeIndex+1].second = te;
                        H.update(label[CLabelIndex][0], label[CLabelIndex][2]);
                    }
                }
            }
            else{
                int PreviousNode = Pi[CLabelIndex][CNodeIndex-1];

                for (int i=0;i<F[PreviousNode].size();i++){
                    if(F[PreviousNode][i].first == CNode){
                        F[PreviousNode][i].second = F[PreviousNode][i].second - 1;
                    }
                }

                for (int i=0;i<F[CNode].size();i++){
                    if(F[CNode][i].first == NextNode){
                        F[CNode][i].second = F[CNode][i].second + 1;

                        int tm = G[CNode][i].second;
                        int Cflow = F[CNode][i].second;
                        int te = tm * (1 + sigma * pow(Cflow/varphi, beta));

                        label[CLabelIndex][2] = label[CLabelIndex][2] + te;
                        label[CLabelIndex][1] = label[CLabelIndex][1] + 1;
                        GResult[CLabelIndex][CNodeIndex+1].second = te;
                        H.update(label[CLabelIndex][0], label[CLabelIndex][2]);
                    }
                }
            }
        }
    }
    return GResult;
}

vector<vector<pair<int,int>>> Graph::Algorithm2(vector<vector<pair<int, int>>> &G,
                                                vector<vector<vector<int>>> &Q, vector<vector<vector<int>>> &Pi) {
    /*
     * Description: This function is the baseline of traffic trajectory project algorithm II.
     * Except calculating estimated travel time by algorithm I, it can update the estimated travel time
     * by adding new queries and trajectories.
     *
     * Parameters:
     * &G -> road network.
     * &Q -> queries. contains queries and each query contains source node, destination node, and departure time.
     * &Pi -> trajectories. vertices passed by each query.
     *
     * Return:
     * GResult -> estimated travel time on each trajectory's rodd segment
     */

    vector<vector<int>> PiTemp;
//    for (int i=0;i<Pi.size();i++){
//        for (int j=0;j<Pi[i].size();j++){
//            PiTemp[i].resize(Pi[i][j].size());
//        }
//    }

    vector<vector<int>> QTemp;
//    for (int i=0;i<Q.size();i++){
//        for (int j=0;j<Q[i].size();j++){
//            QTemp[i].resize(Q[i][j].size());
//        }
//    }

    vector<vector<pair<int,int>>> result;

    for (int i=0;i<Pi.size();i++){
        for (int j=0;j<Pi[i].size();j++){
            PiTemp.push_back(Pi[i][j]);
            QTemp.push_back(Q[i][j]);
        }

        std::chrono::high_resolution_clock::time_point t1, t2;
        std::chrono::duration<double> time_span;
        t1=std::chrono::high_resolution_clock::now();

        result = Algorithm1(G, QTemp, PiTemp);

        t2=std::chrono::high_resolution_clock::now();
        time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
        cout << "The time consumption of round " << i << " with size " << Pi[i].size() << " is: ";
        cout << time_span.count() <<endl;
    }

    return result;
}

bool Graph::OverlapDet(vector<int> &Pi1, vector<int> &Pi2){

    /*
     * Description: This function check if two trajectory have overlap. If there are overlap, return true,
     * otherwise return false.
     *
     * Parameters:
     * vector<int> &Pi1 -> trajectory1.
     * vector<int> &Pi2 -> trajectory2.
     *
     * Return:
     * bool -> true or false.
     */

    for (int i=0;i<Pi1.size();i++){
        for (int j=0;j<Pi2.size();j++){
            if (Pi1[i] == Pi2[j]){
                return true;
            }
        }
    }
    return false;
}

int Graph::OverlapReturn(vector<int> &Pi1, vector<int> &Pi2){

    /*
     * Description: This function check if two trajectory have overlap, and return first overlap node ID index.
     *
     * Parameters:
     * vector<int> &Pi1 -> trajectory1.
     * vector<int> &Pi2 -> trajectory2.
     *
     * Return:
     * int -> first overlap node ID.
     */

    for (int i=0;i<Pi1.size();i++){
        for (int j=0;j<Pi2.size();j++){
            if (Pi1[i] == Pi2[j]){
                return i;
            }
        }
    }
    return -1;
}

vector<vector<pair<int,int>>> Graph::Algorithm1_V1(vector<vector<pair<int, int>>> &G,
                                                   vector<vector<int>> &Q, vector<vector<int>> &Pi){
    /*
     * Description: This function is the another version of algorithm I.
     * It calculates the leaving time on each trajectory's road segment,
     * by considering the affect of traffic congestion based on traffic flow.
     *
     * Parameters:
     * &G -> road network.
     * &Q -> queries. contains queries and each query contains source node, destination node, and departure time.
     * &Pi -> trajectories. vertices passed by each query.
     *
     * Return:
     * GResult -> leaving time on each trajectory's road segment.
     */

    // Initialization
    benchmark::heap<2, int, int> H(Q.size());
    vector<vector<int>> label(Q.size());
    vector<vector<pair<int,int>>> GResult(Pi.size());
    for (int i=0;i<Pi.size();i++){
        GResult[i].resize(Pi[i].size());
    }
    vector<vector<pair<int,int>>> F = G;

    // initialize each weight as infinity
    for (int i=0;i<Pi.size();i++){
        for (int j=0;j<1;j++){
            GResult[i][j].first = Pi[i][j];
            GResult[i][j].second = 0;
        }
        for (int k=1;k<Pi[i].size();k++){
            GResult[i][k].first = Pi[i][k];
            GResult[i][k].second = INF;
        }
    }

    // initialize labels and update into priority queue
    int NodeIndex = 0;
    for (int i=0;i<Q.size();i++){
        label[i] = {i,NodeIndex,Q[i][2]};
        H.update(i,label[i][2]); // query index, departure time
    }

    // initialize traffic flow on each road segment
    for (int i=0;i<F.size();i++){
        for (int j=0;j<F[i].size();j++){
            F[i][j].second = 0;
        }
    }

    int CLabelIndex, CTime, CNodeIndex;
    while (!H.empty()){
        H.extract_min(CLabelIndex, CTime);

        CNodeIndex = label[CLabelIndex][1];
        int CNode = Pi[CLabelIndex][CNodeIndex];

        // define parameter of traffic flow function
        float sigma = 0.15;
        float varphi = 20;
        float beta = 2;

        if (CNodeIndex == (Pi[CLabelIndex].size()-1)){
            continue;
        }
        else{
            int NextNode = Pi[CLabelIndex][CNodeIndex+1];
            if (CNodeIndex == 0){
                for (int i=0;i<F[CNode].size();i++){
                    if(F[CNode][i].first == NextNode){
                        F[CNode][i].second = F[CNode][i].second + 1;

                        int tm = G[CNode][i].second;
                        int Cflow = F[CNode][i].second;
                        int te = tm * (1 + sigma * pow(Cflow/varphi, beta));

                        label[CLabelIndex][2] = label[CLabelIndex][2] + te;
                        label[CLabelIndex][1] = label[CLabelIndex][1] + 1;
                        GResult[CLabelIndex][CNodeIndex+1].second = label[CLabelIndex][2];
                        H.update(label[CLabelIndex][0], label[CLabelIndex][2]);
                    }
                }
            }
            else{
                int PreviousNode = Pi[CLabelIndex][CNodeIndex-1];

                for (int i=0;i<F[PreviousNode].size();i++){
                    if(F[PreviousNode][i].first == CNode){
                        F[PreviousNode][i].second = F[PreviousNode][i].second - 1;
                    }
                }

                for (int i=0;i<F[CNode].size();i++){
                    if(F[CNode][i].first == NextNode){
                        F[CNode][i].second = F[CNode][i].second + 1;

                        int tm = G[CNode][i].second;
                        int Cflow = F[CNode][i].second;
                        int te = tm * (1 + sigma * pow(Cflow/varphi, beta));

                        label[CLabelIndex][2] = label[CLabelIndex][2] + te;
                        label[CLabelIndex][1] = label[CLabelIndex][1] + 1;
                        GResult[CLabelIndex][CNodeIndex+1].second = label[CLabelIndex][2];
                        H.update(label[CLabelIndex][0], label[CLabelIndex][2]);
                    }
                }
            }
        }
    }
    return GResult;
}

vector<vector<pair<int,int>>> Graph::Algorithm2_V1(vector<vector<pair<int, int>>> &G,
                                                   vector<vector<vector<int>>> &Q, vector<vector<vector<int>>> &Pi) {
    /*
     * Description: This function is the baseline of traffic trajectory project algorithm II.
     * Except calculating estimated travel time by algorithm I, it can update the estimated travel time
     * by adding new queries and trajectories.
     *
     * Parameters:
     * &G -> road network.
     * &Q -> queries. contains queries and each query contains source node, destination node, and departure time.
     * &Pi -> trajectories. vertices passed by each query.
     *
     * Return:
     * GResult -> estimated travel time on each trajectory's rodd segment
     */

    vector<vector<pair<int,int>>> result;
    vector<vector<pair<int,int>>> result_index;
    vector<vector<pair<int,int>>> resultTemp;
    vector<vector<pair<int,int>>> result_indexTemp;

    result = Algorithm1(G, Q[0], Pi[0]);
    result_index = Algorithm1_V1(G, Q[0], Pi[0]);

    vector<vector<int>> PiTemp;
    for (int i = 0; i < Pi[0].size(); i++) {
        PiTemp.push_back(Pi[0][i]);
    }
    vector<vector<int>> QTemp;
    for (int i = 0; i < Q[0].size(); i++) {
        QTemp.push_back(Q[0][1]);
    }

    vector<vector<int>> PiInput;
    vector<vector<int>> QInput;

    for (int i=1;i<Pi.size();i++){ // 选择某天的查询
        for (int j=0;j<Pi[i].size();j++){ // 某天查询的每个查询
            for (int k=0;k<PiTemp.size();k++){ // 目前路上的信息
                bool detect = OverlapDet(Pi[i][j],PiTemp[k]);
                if (detect == true){
                    // 进行下一步，时间的判断
                    int index = OverlapReturn(Pi[i][j],PiTemp[k]);
                    int time1 = result_index[k][PiTemp[k].size()-1].second;
                    int time2 = Q[i][j][2];
                    if (time2 > time1){
                        PiInput.push_back(Pi[i][j]);
                        QInput.push_back(Q[i][j]);
                    }
                    else{
                        // 老的拿出来，填进去
                        PiTemp.erase(PiTemp.begin()+k);
                        QTemp.erase(QTemp.begin()+k);
                        PiInput.push_back(PiTemp[k]);
                        QInput.push_back(QTemp[k]);
                        // 新的填进去
                        PiInput.push_back(Pi[i][j]);
                        QInput.push_back(Q[i][j]);
                    }
                }
                else{
                    // 初始化轨迹的所有信息，添加到优先队列中
                    PiInput.push_back(Pi[i][j]);
                    QInput.push_back(Q[i][j]);
                }
            }
        }
        resultTemp = Algorithm1(G, QInput, PiInput);
        result_indexTemp = Algorithm1_V1(G, QInput, PiInput);
        // 合并resultTemp和result，相当于添加进去
        // 判断是否是最后一个集合了，如果不是，把比较的单个的集合添加到大集合里去
    }

    // 返回最后的结果
    return result;
}

vector<vector<vector<int>>> Graph::ReadTrajectoryList(string path_part1, string path_part2, int Listnum, vector<vector<vector<int>>> Q_List){

    /*
     * Description: read data from trajectory file into a variable.
     *
     * Parameters:
     * string path_part1 -> first part to construct file.
     * string path_part2 -> second part to construct file.
     * int Listnum -> number of files read into list of trajectory.
     * vector<vector<vector<int>>> Q_List -> capture size of each query or route.
     *
     * Return:
     * vector<vector<vector<int>>> -> variable store a list of trajectory information.
     */

    vector<vector<vector<int>>> Pi_list;

    for (int i = 1; i < Listnum+1; i++) {
        string index = to_string(i);
        string path = path_part1 + index + path_part2;

        /*根据需要把ReadRoutes函数更改了，因此有需要再重新更改把*/
/*        vector<vector<int>> Pi_temp = ReadRoutes(path, Q_List[i-1].size());
        Pi_list.push_back(Pi_temp);
        cout << "read route " << i << " done" << endl;*/
    }

    return Pi_list;
}

vector<vector<vector<int>>> Graph::ReadQueryList(string path_part1, string path_part2, int filenum){

    /*
     * Description: read data from query file into a variable.
     *
     * Parameters:
     * string path_part1 -> first part to construct file.
     * string path_part2 -> second part to construct file.
     * int filenum -> number of files read into list of query.
     *
     * Return:
     * vector<vector<vector<int>>> -> variable store a list of query information.
     */

    vector<vector<vector<int>>> Q_list;

    for (int i = 1; i < filenum+1; i++) {
        string index = to_string(i);
        string path = path_part1 + index + path_part2;

        vector<vector<int>> Q_temp = ReadQuery(path);
        Q_list.push_back(Q_temp);
        cout << "read query " << i << " done" << endl;
    }

    return Q_list;
}

pair<vector<double>, vector<double>> Graph::Graph2Grids(double lat1, double lat2, double lon1, double lon2, int num){

    /*
     * Description: split Graph into num x num grids.
     *
     * Parameters:
     * double lat1 -> min latitude.
     * double lat2 -> max latitude.
     * double lon1 -> min longitude.
     * double lon2 -> max longitude.
     *
     * Return:
     * pair<vector<double>, vector<double>> result -> a pair of vector contains lat and log.
     */

    /*
    pair<vector<double>, vector<double>> grids = g.Graph2Grids(39.4167,	41.0833, 115.375, 117.5, 3);
    cout << "lat are: ";
    for (int i=0;i<grids.first.size();i++){
        cout << grids.first[i] << " ";
    }
    cout << endl;
    cout << "lon are: ";
    for (int i=0;i<grids.second.size();i++){
        cout << grids.second[i] << " ";
    }
    */

    pair<vector<double>, vector<double>> result;

    double lat_dist = (lat2 - lat1) / num;
    double lon_dist = (lon2 - lon1) / num;

    float lat_temp = lat1;
    float lon_temp = lon1;

    for (int i=0;i<num+1;i++){
        result.first.push_back(lat_temp);
        result.second.push_back(lon_temp);

        lat_temp = lat_temp + lat_dist;
        lon_temp = lon_temp + lon_dist;
    }

    return  result;
}

map<int,pair<double,double>> Graph::IDtoLocation (string filename){

    /*
     * Description: Read and create map for NodeID and its latitude and longitude.
     *
     * Parameters:
     * string filename -> path to file BJ_NodeIDLonLat.
     *
     * Return:
     * map<int,pair<double,double>> IDtoLocation -> map for NodeID with its latitude and longitude.
     */

    /*
    map<int,pair<double,double>> IDtoLocation = g.IDtoLocation(BJ_NodeIDLonLat);
    cout << IDtoLocation[296709].first << " " << IDtoLocation[296709].second << endl;
    */

    ifstream IF(filename);
    if(!IF){
        cout<<"Cannot open Map "<<filename<<endl;
    }

    int nodenum, NodeID;
    double lat, lon;
    IF>>nodenum; // 296710

    map<int,pair<double,double>> IDtoLocation;
    int lines = CountLines(filename);
    for (int i=0;i<lines-1;i++){
        IF>>NodeID>>lon>>lat;
        IDtoLocation[NodeID] = make_pair(lat,lon);
    }
    return IDtoLocation;
}

map<pair<int,int>, int> Graph::GridsIndex (int num){

    /*
     * Description: create name and index for each grid.
     *
     * Parameters:
     * int num -> n^2 is the number of grids.
     *
     * Return:
     * map<pair<int,int>, int> GridNames -> map for grids name.
     */

    /*
    map<pair<int,int>, int> value = GridsIndex(3);
    cout << value[make_pair(2,2)] << endl;
     */

    map<pair<int,int>, int> GridNames;
    int name = 0;

    for (int i=0;i<num;i++){
        for (int j=0;j<num;j++){
            pair<int, int> value = make_pair(i,j);
            GridNames[value] = name;
            name = name + 1;
        }
    }

    return GridNames;
}

void Graph::Algorithm1_Parallel(vector<vector<pair<int, int>>> &G,
                                vector<vector<int>> &Q, vector<vector<int>> &Pi,
                                vector<vector<pair<int,int>>>& GResult){
    /*
     * Description: This function is a copy of algorithm I for parallel computation.
     *
     * Parameters:
     * &G -> road network.
     * &Q -> queries. contains queries and each query contains source node, destination node, and departure time.
     * &Pi -> trajectories. vertices passed by each query.
     *
     * Return:
     * GResult -> estimated travel time on each trajectory's road segment
     */

    // Initialization
    benchmark::heap<2, int, int> H(Q.size());

    vector<vector<int>> label(Q.size());
    vector<vector<pair<int,int>>> F = G;

    // initialize each weight as infinity
    for (int i=0;i<Pi.size();i++){
        for (int j=0;j<1;j++){
            GResult[i][j].first = Pi[i][j];
            GResult[i][j].second = 0;
        }

        for (int j=1;j<Pi[i].size();j++){
            GResult[i][j].first = Pi[i][j];
            GResult[i][j].second = INF;
        }
    }

    // initialize labels and update into priority queue
    int NodeIndex = 0;
    for (int i=0;i<Q.size();i++){
        label[i] = {i,NodeIndex,Q[i][2]};
        H.update(i,label[i][2]); // query index, departure time
    }

    // initialize traffic flow on each road segment
    for (int i=0;i<F.size();i++){
        for (int j=0;j<F[i].size();j++){
            F[i][j].second = 0;
        }
    }

    int CLabelIndex, CTime, CNodeIndex, CNode;
    while (!H.empty()){
        H.extract_min(CLabelIndex, CTime);

        CNodeIndex = label[CLabelIndex][1];
        CNode = Pi[CLabelIndex][CNodeIndex];

        // define parameter of traffic flow function
        float sigma = 0.15;
        float varphi = 20;
        float beta = 2;

        if (CNodeIndex == (Pi[CLabelIndex].size()-1)){
            //
            int PreviousNode = Pi[CLabelIndex][CNodeIndex-1];

            for (int i=0;i<F[PreviousNode].size();i++){
                if(F[PreviousNode][i].first == CNode){
                    F[PreviousNode][i].second = F[PreviousNode][i].second - 1;
                }
            }

            continue;
        }
        else{
            int NextNode = Pi[CLabelIndex][CNodeIndex+1];
            if (CNodeIndex == 0){
                for (int i=0;i<F[CNode].size();i++){
                    if(F[CNode][i].first == NextNode){
                        F[CNode][i].second = F[CNode][i].second + 1;

                        int tm = G[CNode][i].second;
                        int Cflow = F[CNode][i].second;
                        int te = tm * (1 + sigma * pow(Cflow/varphi, beta));

                        label[CLabelIndex][2] = label[CLabelIndex][2] + te;
                        label[CLabelIndex][1] = label[CLabelIndex][1] + 1;
                        GResult[CLabelIndex][CNodeIndex+1].second = te;
                        H.update(label[CLabelIndex][0], label[CLabelIndex][2]);
                    }
                }
            }
            else{
                int PreviousNode = Pi[CLabelIndex][CNodeIndex-1];

                for (int i=0;i<F[PreviousNode].size();i++){
                    if(F[PreviousNode][i].first == CNode){
                        F[PreviousNode][i].second = F[PreviousNode][i].second - 1;
                    }
                }

                for (int i=0;i<F[CNode].size();i++){
                    if(F[CNode][i].first == NextNode){
                        F[CNode][i].second = F[CNode][i].second + 1;

                        int tm = G[CNode][i].second;
                        int Cflow = F[CNode][i].second;
                        int te = tm * (1 + sigma * pow(Cflow/varphi, beta));

                        label[CLabelIndex][2] = label[CLabelIndex][2] + te;
                        label[CLabelIndex][1] = label[CLabelIndex][1] + 1;
                        GResult[CLabelIndex][CNodeIndex+1].second = te;
                        H.update(label[CLabelIndex][0], label[CLabelIndex][2]);
                    }
                }
            }
        }
    }
}


vector<vector<pair<int,int>>> Graph::Parallel(vector<vector<pair<int, int>>> &G, vector<vector<int>> &Q, vector<vector<int>> &Pi, int num, string filename){

    /*
     * Description: Parallel computation on Algorithm I.
     *
     * Parameters:
     * vector<vector<pair<int, int>>> &G -> road network.
     * vector<vector<int>> &Q -> query.
     * vector<vector<int>> &Pi -> path.
     * int num -> num square is the number of grids.
     * string filename -> load Node ID and its location.
     *
     * Return:
     * vector<vector<pair<int,int>>> GResult -> record nodes and its related travel time on paths.
     */

    // Initialization
    // ---------------------------------------------------------------------------------------------

    // define return value and size
    vector<vector<pair<int,int>>> GResult(Pi.size());
    for (int i=0;i<Pi.size();i++){
        GResult[i].resize(Pi[i].size());
    }

    // split road network into grids by x-axis, y-axis
    map<int,pair<double,double>> IDtoLocations = IDtoLocation(filename);
    pair<vector<double>, vector<double>> grids = Graph2Grids(39.4167, 41.0833, 115.375, 117.5, num);
    map<pair<int,int>, int> GridIndex = GridsIndex(num);

    vector<vector<pair<int, pair<int, int>>>> temp1;
    vector<vector<vector<pair<int, pair<int, int>>>>> Pi_Grids(GridIndex.size(), temp1);
    vector<vector<int>> temp2;
    vector<vector<vector<int>>> Q_Grids(GridIndex.size(), temp2);
    vector<vector<int>> temp3;
    vector<vector<vector<int>>> Pi_Grids_input(GridIndex.size(), temp3);

    vector<vector<pair<int, pair<int, int>>>> Pi_copy(Pi.size());
    for (int i=0;i<Pi.size();i++){
        Pi_copy[i].resize(Pi[i].size());
    }
    for (int i=0;i<Pi_copy.size();i++){
        for (int j=0;j<Pi_copy[i].size();j++){ // (value, (path_index, value_index))
            Pi_copy[i][j].first = Pi[i][j];
            Pi_copy[i][j].second.first = i;
            Pi_copy[i][j].second.second = j;
        }
    }

    vector<vector<int>> Q_copy(Q.size());
    for (int i=0;i<Q.size();i++){
        Q_copy[i].resize(Q[i].size());
    }
    for (int i=0;i<Q.size();i++){
        for (int j=0;j<Q[i].size();j++){
            Q_copy[i][j] = Q[i][j];
        }
    }

    vector<vector<pair<int, pair<int, int>>>> Pi_rest;

//    vector<vector<pair<int,int>>> temp4;
    vector<vector<vector<pair<int,int>>>> GResult_temp;

    // 迭代计算
    // ---------------------------------------------------------------------------------------------

    int time = 0;

    while (!Pi_copy.empty()){

        time += 1;

        vector<pair<int, pair<int, int>>> pi_temp, pi_temp_rest;
        vector<int> pi_input_temp;

        // go through each path
        for (int i=0;i<Pi_copy.size();i++){

            pi_temp.clear(); pi_temp_rest.clear();
            pi_input_temp.clear();

            int source_ID = Pi_copy[i][0].first; // (value, (path_index, value_index))
            pair<int, pair<int, int>> SourceID_with_index = Pi_copy[i][0];
            double source_lat = IDtoLocations[source_ID].first;
            double source_lon = IDtoLocations[source_ID].second;

            int lat_value, lon_value;
            for (int j=0;j<num;j++){
                bool range_bool_lat = check_range(source_lat,grids.first[j], grids.first[j+1]);
                if (range_bool_lat == true){
                    lat_value = j; break;
                }
            }
            for (int j=0;j<num;j++){
                bool range_bool_lon = check_range(source_lon,grids.second[j], grids.second[j+1]);
                if (range_bool_lon == true){
                    lon_value = j; break;
                }
            }
            int Source_GridIndex = GridIndex[make_pair(lat_value,lon_value)];

            pi_temp.push_back(SourceID_with_index); // (value, (path_index, value_index))
            pi_input_temp.push_back(SourceID_with_index.first);

            int lat_value_other, lon_value_rest;
            for (int j=1;j<Pi_copy[i].size();j++){
                int ID = Pi_copy[i][j].first;
                pair<int, pair<int, int>> ID_with_index = Pi_copy[i][j];
                double lat = IDtoLocations[ID].first;
                double lon = IDtoLocations[ID].second;

                for (int k=0;k<num;k++){
                    bool range_bool_lat1 = check_range(lat,grids.first[k], grids.first[k+1]);
                    if (range_bool_lat1 == true){
                        lat_value_other = k; break;
                    }
                }
                for (int k=0;k<num;k++){
                    bool range_bool_lon1 = check_range(lon,grids.second[k], grids.second[k+1]);
                    if (range_bool_lon1 == true){
                        lon_value_rest = k; break;
                    }
                }
                int Other_GridIndex = GridIndex[make_pair(lat_value_other,lon_value_rest)];

                if (j != Pi_copy[i].size()-1){
                    if (Other_GridIndex == Source_GridIndex){
                        pi_temp.push_back(ID_with_index);
                        pi_input_temp.push_back(ID_with_index.first);
                    }
                    else{
                        pi_temp.push_back(ID_with_index);
                        pi_input_temp.push_back(ID_with_index.first);

                        pi_temp_rest.push_back(ID_with_index);

                        for (int k=j+1;k<Pi_copy[i].size();k++){
                            pi_temp_rest.push_back(Pi_copy[i][k]);
                        }
                        Pi_Grids[Source_GridIndex].push_back(pi_temp);
                        Pi_Grids_input[Source_GridIndex].push_back(pi_input_temp);
                        Q_Grids[Source_GridIndex].push_back(Q_copy[pi_temp[0].second.first]);

                        Pi_rest.push_back(pi_temp_rest);

                        break;
                    }
                }
                else{
                    pi_temp.push_back(ID_with_index);
                    pi_input_temp.push_back(ID_with_index.first);
                    Pi_Grids[Source_GridIndex].push_back(pi_temp);
                    Pi_Grids_input[Source_GridIndex].push_back(pi_input_temp);
                    Q_Grids[Source_GridIndex].push_back(Q_copy[pi_temp[0].second.first]);
                }
            }
        }

        vector<vector<pair<int,int>>> temp4;
        vector<pair<int,int>> temp5;
        GResult_temp.resize(GridIndex.size(), temp4);
        for (int i=0;i<Pi_Grids.size();i++){
            GResult_temp[i].resize(Pi_Grids[i].size(), temp5);
            for (int j=0;j<Pi_Grids[i].size();j++){
                GResult_temp[i][j].resize(Pi_Grids[i][j].size());
            }
        }

        std::chrono::high_resolution_clock::time_point t1, t2;
        std::chrono::duration<double> time_span;
        t1=std::chrono::high_resolution_clock::now();

        boost::thread_group threadf;

        for (int i=0;i<pow(num,2);i++){
//            threadf.add_thread(new boost::thread(&Algorithm1_Parallel, this, boost::ref(G), boost::ref(Q_Grids),
//                                                 boost::ref(Pi_Grids_input), boost::ref(GResult_temp), i));
            threadf.add_thread(new boost::thread(&Algorithm1_Parallel, boost::ref(G), boost::ref(Q_Grids[i]),
                                                 boost::ref(Pi_Grids_input[i]), boost::ref(GResult_temp[i])));
        }

        threadf.join_all();

        t2=std::chrono::high_resolution_clock::now();
        time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
        cout << "time consumption in " << time << " round is: "<< time_span.count() <<endl;

        for (int i=0;i<GResult_temp.size();i++){
            for (int j=0;j<GResult_temp[i].size();j++){
                for (int k=0;k<1;k++){
                    int path_index = Pi_Grids[i][j][k].second.first;
                    int vertex_index = Pi_Grids[i][j][k].second.second;
                    GResult[path_index][vertex_index].first = GResult_temp[i][j][k].first;
                }
                for (int k=1;k<GResult_temp[i][j].size();k++){
                    int result = GResult_temp[i][j][k].second;
                    int path_index = Pi_Grids[i][j][k].second.first;
                    int vertex_index = Pi_Grids[i][j][k].second.second;
                    GResult[path_index][vertex_index].first = GResult_temp[i][j][k].first;
                    GResult[path_index][vertex_index].second = result;
                }
            }
        }

        Pi_copy.clear();
        Pi_copy = Pi_rest;

        vector<int> sum_time(GResult.size());
        sum_time.clear();
        for (int i=0;i<GResult.size();i++){
            sum_time[i] = Q[i][2];
            for (int j=0;j<GResult[i].size();j++){
                if (GResult[i][j].second != INF){
                    sum_time[i] = sum_time[i] + GResult[i][j].second;
                }
            }
        }

        for (int i=0;i<Pi_rest.size();i++){
            int PiIndex =  Pi_rest[i].front().second.first;
            int vertexIndex = Pi_rest[i].front().second.second;
            Q_copy[PiIndex][0] = Pi_rest[i].front().first;
            Q_copy[PiIndex][1] = Pi_rest[i].back().first;
            Q_copy[PiIndex][2] = sum_time[PiIndex];
        }

        Pi_rest.clear();
        GResult_temp.clear();
//        for (int i=0;i<Pi_Grids_input.size();i++){
//            Pi_Grids_input[i].clear();
//        }
//        for (int i=0;i<Pi_Grids.size();i++){
//            Pi_Grids[i].clear();
//        }
//        for (int i=0;i<Q_Grids.size();i++){
//            Q_Grids[i].clear();
//        }
    }

    cout << "iteration times is: " << time << endl;
    return GResult;
}

void Graph::RouteCombine(vector<vector<int>> Pi, string export_file){

    /*
     * Description: Routes on Beijing Road Network are stored in numerous single files, and read them into program may
     * waste lots of time because open and close iteration. We hope to store each day's routes data into one file to
     * save time of reading data into program. First, we read each file into program stored by variable Pi.
     * Then, export values from Pi into a single file.
     *
     * Parameters:
     * vector<vector<int>> Pi -> variable stores route information.
     * string export_file -> file path to store all routes information in one file.
     *
     * Return:
     * void
     */

    // 把vector内的内容写入一个文件
    ofstream outfile;
    outfile.open(export_file);
    for (int i=0;i<Pi.size();i++){
        outfile << Pi[i].size() <<  " ";
        for (int j=0;j<Pi[i].size();j++){
            outfile << Pi[i][j] << " ";
        }
        outfile << endl;
    }
    outfile.close();
}

vector<vector<int>> Graph::ReadRoutes(string input_file){

    /*
     * Description: Read routes into program.
     *
     * Parameters:
     * string input_file -> file stores all routes in one day. Each row contains all vertex in a route and the first
     * element in one row is the number of vertex in this route.
     *
     * Return:
     * vector<vector<int>> -> variable stores all routes.
     */

    vector<vector<int>> Pi;

    ifstream file_name(input_file);
    if(!file_name){cout<<"Cannot open Map "<< input_file <<endl;}

    int num, Vertex;

    while(file_name >> num)
    {
        vector<int> route_temp(num, 0);
        for (int j = 0; j < num; j++)
        {
            file_name >> route_temp[j];

        }
        Pi.push_back(route_temp);
    }

/*    for (int i=0;i<lines;i++) {
        pi.clear();
        file_name >> num;
        for (int j=0;j<num;j++){
            file_name >> Vertex;
            pi.push_back(Vertex);
        }
        Pi.push_back(pi);
    }*/

    file_name.close();
    return Pi;
}

vector<vector<int>> Graph::ReadRoutes_w_num(string input_file, int lines, int read_num){

    /*
     * Description: Read routes into program.
     *
     * Parameters:
     * string input_file -> file stores all routes in one day. Each row contains all vertex in a route and the first
     * element in one row is the number of vertex in this route.
     *
     * Return:
     * vector<vector<int>> -> variable stores all routes.
     */

    vector<vector<int>> Pi;
    ifstream file_name(input_file);
    if(!file_name){cout<<"Cannot open Map "<< input_file <<endl;}

    int input_num;
    if (read_num == 0){
        input_num = lines;
    }else{
        input_num = read_num;
    }

    int num, Vertex;
    vector<int> pi;
    for (int i=0;i<input_num;i++) {
        pi.clear();
        file_name >> num;

/*        if (num == 0)
        {
            cout << "route num is 0 when read in." << endl;
        }*/

        for (int j=0;j<num;j++){
            file_name >> Vertex;
            pi.push_back(Vertex);
        }
        Pi.push_back(pi);
    }
    file_name.close();

/*
    // 检验route正确性
    vector<vector<int>> cleaned_rotue = only_route_raw_clean(Neighbor, Pi);
*/

    return Pi;
}

void print(std::unordered_set<int> const &s)
{
    std::copy(s.begin(),
              s.end(),
              std::ostream_iterator<int>(std::cout, " "));
}


pair<vector<vector<int>>,vector<vector<int>>> Graph::query_route_raw_clean(
        vector<vector<pair<int,int>>> RoadNetwork, vector<vector<int>> route_raw, vector<vector<int>> query_raw){

    /*
     * Description:
     * some vertex in a routes cannot connect, which may cause incorrect result. This function remove these
     * routes and related query for further analysis.
     */

    vector<vector<int>> route_data = route_raw;
    vector<vector<int>> query_data = query_raw;

    cout << "query data before clean is: " << query_data.size() << " & route data size is: " << route_data.size();
    cout << endl;

    pair<vector<vector<int>>,vector<vector<int>>> route_query_dataPair;

    for (int i=0;i<route_data.size();i++)
    {

        if (route_data[i].size() == 0 ){

/*            cout << "route " << i << "'s size is 0." << endl;
            cout << "departure node is: " << query_raw[i][0] << " with destination node: " << query_raw[i][1] << endl;

            cout << "neighbor id is: ";
            for (int j = 0; j < Neighbor[i].size(); j++)
            {
                cout <<  Neighbor[query_raw[i][0]][j].first << " ";
            }
            cout << "\n";*/

            route_data.erase(route_data.begin() + i);
            query_data.erase(query_data.begin() + i);
            i = i - 1;
            continue;
        }

        // check connection
        for (int j=0;j<route_data[i].size()-1;j++)
        {
            int Node = route_data[i][j]; int NextNode = route_data[i][j+1];
            int count = 0;
            for (int k=0;k<Neighbor[Node].size();k++)
            {
                if (Neighbor[Node][k].first != NextNode){
                    count += 1;
                }
            }
            if (count == Neighbor[Node].size())
            {

/*                cout << "removed route is: " << endl;
                for (int k = 0; k < route_data[i].size(); k++){
                    cout << route_data[i][k] << " ";
                }
                cout << endl;

                cout << "cannot connected points are: " << route_data[i][j] << " and " << route_data[i][j+1] << endl;*/

                route_data.erase(route_data.begin() + i);
                query_data.erase(query_data.begin() + i);
                i = i - 1;

                break;
            }
        }
    }

    // check duplicate
    for (int i=0;i<route_data.size();i++)
    {
        vector<int> one_route_w_roadID; one_route_w_roadID.clear();
        int node01, node02, roadID;

        for (int j=0;j<route_data[i].size()-1;j++)
        {
            node01 = route_data[i][j];
            node02 = route_data[i][j+1];
            roadID = map_nodeID_2_roadID[make_pair(node01,node02)];
            one_route_w_roadID.push_back(roadID);
        }

        std::set<int> duplicates = findDuplicates(one_route_w_roadID);

        if (duplicates.size() != 0)
        {

/*            cout << "removed route is: " << endl;
            for (int k = 0; k < route_data[i].size(); k++){
                cout << route_data[i][k] << " ";
            }
            cout << endl;

            set<int>::iterator it;
            for (it = duplicates.begin(); it != duplicates.end(); it++)
            {
                cout << *it << endl;  // 自动排序,打印结果为12345
            }

            cout << "duplicate happens." << endl;*/

            route_data.erase(route_data.begin() + i);
            query_data.erase(query_data.begin() + i);
            i = i - 1;

            // break;
        }
    }

    route_query_dataPair.first = route_data; route_query_dataPair.second = query_data;

    cout << "route and query data clean is done." << "\n";
    cout <<  "cleaned query data size is: " << query_data.size();
    cout << " & cleaned route data size is: " << route_data.size() << endl;

    return route_query_dataPair;
}


vector<vector<int>> Graph::only_route_raw_clean(
        vector<vector<pair<int,int>>> RoadNetwork, vector<vector<int>> route_raw){

    /*
     * Description:
     * some vertex in a routes cannot connect, which may cause incorrect result. This function remove these
     * routes and related query for further analysis.
     */

    vector<vector<int>> route_data = route_raw;

    for (int i=0;i<route_data.size();i++)
    {

        if (route_data[i].size() == 0 ){

            route_data.erase(route_data.begin() + i);
            i = i - 1;
            continue;
        }

        // check connection
        for (int j=0;j<route_data[i].size()-1;j++)
        {
            int Node = route_data[i][j]; int NextNode = route_data[i][j+1];
            int count = 0;
            for (int k=0;k<Neighbor[Node].size();k++)
            {
                if (Neighbor[Node][k].first != NextNode){
                    count += 1;
                }
            }
            if (count == Neighbor[Node].size())
            {

/*                cout << "removed route is: " << endl;
                for (int k = 0; k < route_data[i].size(); k++){
                    cout << route_data[i][k] << " ";
                }
                cout << endl;

                cout << "cannot connected points are: " << route_data[i][j] << " and " << route_data[i][j+1] << endl;*/

                cout << "cannot connect then erase." << endl;


                route_data.erase(route_data.begin() + i);
                i = i - 1;

                break;
            }
        }
    }

    // check duplicate
    for (int i=0;i<route_data.size();i++)
    {
        vector<int> one_route_w_roadID; one_route_w_roadID.clear();
        int node01, node02, roadID;

        for (int j=0;j<route_data[i].size()-1;j++)
        {
            node01 = route_data[i][j];
            node02 = route_data[i][j+1];
            roadID = map_nodeID_2_roadID[make_pair(node01,node02)];
            one_route_w_roadID.push_back(roadID);
        }

        std::set<int> duplicates = findDuplicates(one_route_w_roadID);

        if (duplicates.size() != 0)
        {

            cout << "removed route is: " << endl;
            for (int k = 0; k < route_data[i].size(); k++){
                cout << route_data[i][k] << " ";
            }
            cout << endl;

            set<int>::iterator it;
            for (it = duplicates.begin(); it != duplicates.end(); it++)
            {
                cout << *it << endl;  // 自动排序,打印结果为12345
            }

            cout << "duplicate happens." << endl;

            route_data.erase(route_data.begin() + i);
            i = i - 1;

            // break;
        }
    }


    cout << " & cleaned route data size is: " << route_data.size() << endl;

    return route_data;
}