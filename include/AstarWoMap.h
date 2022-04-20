//
// Created by lihao on 19-7-9.
//

#ifndef ASTARWOMAP_H
#define ASTARWOMAP_H

#include <iostream>
#include <queue>
#include <unordered_map>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


namespace pathplanning{

enum NodeType{
    obstacle = 0,
    free,
    inOpenList,
    inCloseList
};

struct Node{
    Point point;  // node coordinate
    int F, G, H;  // cost
    Node* parent; // parent node

    Node(Point _point = Point(0, 0)):point(_point), F(0), G(0), H(0), parent(NULL)
    {
    }
};

struct cmp
{
    bool operator() (pair<int, Point> a, pair<int, Point> b) // Comparison function for priority queue
    {
        return a.first > b.first; // min heap
    }
};

struct Obs
{ // can contain single obstacle info
    Eigen::Vector3d CoG; // value in absolute map (not in grid map)
    double yaw;
    double abs_width; // 가로
    double abs_depth; // 세로
    // vector<double> abs_height; // 높이
};

struct AstarConfig{
    bool Euclidean;         // true/false
    double InflateLength;   // unit: [m]

    AstarConfig(bool _Euclidean = true, double _InflateLength = -1.0):
        Euclidean(_Euclidean), InflateLength(_InflateLength)
    {
    }
};

class Astar{

public:
    // Interface function
    void InitAstar(Obs& _obs, double _map_abs_height, double _map_abs_width, double _map_grid_length, AstarConfig _config = AstarConfig());
    void AbsPos2IdxPos(Point2d _AbsPos, Point& _IdxPos);
    void IdxPos2AbsPos(Point _IdxPos, Point2d& _AbsPos);
    void PathPlanning(Point _startPoint, Point _targetPoint, vector<Point>& path);

    inline int point2index(Point point) {
        return point.y * map_widthIdx + point.x; // Assumption: map_widthIdx >= map_heightIdx
    }
    inline Point index2point(int index) {
        return Point(int(index / map_widthIdx), index % map_widthIdx);
    }

private:
    void ObsProcess(Obs& _obs);
    Node* FindPath();
    void GetPath(Node* TailNode, vector<Point>& path);

    int sat(int val, int min, int max);
    Point rotate(int x, int y, Point CoGIdx, double yaw);

private:
    bool inflate_flag = false;
    //Object
    double map_origin_abs_x;
    double map_origin_abs_y;
    int map_heightIdx; // 세로
    int map_widthIdx; // 가로
    double map_grid_length;
    Point startPoint, targetPoint;
    Mat neighbor;

    Mat LabelMap;
    AstarConfig config;

    priority_queue<pair<int, Point>, vector<pair<int, Point>>, cmp> OpenList; // open list
    unordered_map<int, Node*> OpenDict; // open dict
    vector<Node*> PathList;  // path list
};

}




#endif //ASTAR_H
