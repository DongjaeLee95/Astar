//
// Created by dongjae on 2022/04/19.
//

#include "AstarWoMap.h"

namespace pathplanning{


void Astar::InitAstar(Obs& _obs, double _map_abs_height, double _map_abs_width, double _map_grid_length, AstarConfig _config)
{
    char neighbor8[8][2] = {
            {-1, -1}, {-1, 0}, {-1, 1},
            {0, -1},            {0, 1},
            {1, -1},   {1, 0},  {1, 1}
    };

    config = _config;
    neighbor = Mat(8, 2, CV_8S, neighbor8).clone();
    map_grid_length = _map_grid_length;

    map_heightIdx = floor(_map_abs_height/_map_grid_length);
    map_widthIdx = floor(_map_abs_width/_map_grid_length);

    // std::cout << "map_heightIdx: " << map_heightIdx << std::endl;
    // std::cout << "map_widthIdx: " << map_widthIdx << std::endl;

    // Assumption: map origin is located at the center of the map
    map_origin_abs_x = _map_abs_width/2.0;
    map_origin_abs_y = _map_abs_height/2.0;

    ObsProcess(_obs);
}

void Astar::AbsPos2IdxPos(Point2d _AbsPos, Point& _IdxPos)
{
    _IdxPos.x = sat(round((_AbsPos.x + map_origin_abs_x)/map_grid_length),0,map_widthIdx);
    _IdxPos.y = sat(round((_AbsPos.y + map_origin_abs_y)/map_grid_length),0,map_heightIdx);
}

void Astar::IdxPos2AbsPos(Point _IdxPos, Point2d& _AbsPos)
{
    _AbsPos.x = ((double)_IdxPos.x)*map_grid_length - map_origin_abs_x;
    _AbsPos.y = ((double)_IdxPos.y)*map_grid_length - map_origin_abs_y;
}

void Astar::PathPlanning(Point _startPoint, Point _targetPoint, vector<Point>& path)
{
    // Get variables
    startPoint = _startPoint;
    targetPoint = _targetPoint;

    // Path Planning
    Node* TailNode = FindPath();
    GetPath(TailNode, path);
}

void Astar::ObsProcess( Obs& _obs )
{
    Point obs_start_xyIdx; // before rotation
    Point obs_end_xyIdx;   // before rotation
    Point obs_outercircle_start_xyIdx;
    Point obs_outercircle_end_xyIdx;
    double obs_radius;

    if(config.InflateLength > 0 && inflate_flag == false)
    {
        _obs.abs_width = _obs.abs_width + config.InflateLength;
        _obs.abs_depth = _obs.abs_depth + config.InflateLength;
        inflate_flag = true;
    }

    obs_radius = sqrt((_obs.abs_width/2.0)*(_obs.abs_width/2.0) + (_obs.abs_depth/2.0)*(_obs.abs_depth/2.0));

    Point2d obs_start_AbsPos, obs_end_AbsPos, obs_outercircle_start_AbsPos, obs_outercircle_end_AbsPos;
    obs_start_AbsPos.x = _obs.CoG(0) - _obs.abs_width/2.0;
    obs_start_AbsPos.y = _obs.CoG(1) - _obs.abs_depth/2.0;
    obs_end_AbsPos.x = _obs.CoG(0) + _obs.abs_width/2.0;
    obs_end_AbsPos.y = _obs.CoG(1) + _obs.abs_depth/2.0;
    // for smaller search size
    obs_outercircle_start_AbsPos.x = _obs.CoG(0) - obs_radius;
    obs_outercircle_start_AbsPos.y = _obs.CoG(1) - obs_radius;
    obs_outercircle_end_AbsPos.x = _obs.CoG(0) + obs_radius;
    obs_outercircle_end_AbsPos.y = _obs.CoG(1) + obs_radius;

    AbsPos2IdxPos(obs_start_AbsPos,obs_start_xyIdx);
    AbsPos2IdxPos(obs_end_AbsPos,obs_end_xyIdx);
    AbsPos2IdxPos(obs_outercircle_start_AbsPos,obs_outercircle_start_xyIdx);
    AbsPos2IdxPos(obs_outercircle_end_AbsPos,obs_outercircle_end_xyIdx);

    // std::cout << "obs_start_AbsPos: " << obs_start_AbsPos.x << ", " << obs_start_AbsPos.y << std::endl;
    // std::cout << "obs_end_AbsPos: " << obs_end_AbsPos.x << ", " << obs_end_AbsPos.y << std::endl;
    // std::cout << "obs_start_xyIdx: " << obs_start_xyIdx.x << ", " << obs_start_xyIdx.y << std::endl;
    // std::cout << "obs_end_xyIdx: " << obs_end_xyIdx.x << ", " << obs_end_xyIdx.y << std::endl;

    // Initial LabelMap
    LabelMap = Mat::zeros(map_heightIdx, map_widthIdx, CV_8UC1);

    // label map initialize
    for(int y=0;y<map_heightIdx;y++)
    {
        for(int x=0;x<map_widthIdx;x++)
        {
            LabelMap.at<uchar>(y, x) = free;
        }
    }
    
    Point2d _AbsCoG_xy;
    Point CoGIdx;
    _AbsCoG_xy.x = _obs.CoG(0);
    _AbsCoG_xy.y = _obs.CoG(1);

    AbsPos2IdxPos(_AbsCoG_xy,CoGIdx);

    for(int x=obs_outercircle_start_xyIdx.x;x<obs_outercircle_end_xyIdx.x;x++)
    {
        for(int y=obs_outercircle_start_xyIdx.y;y<obs_outercircle_end_xyIdx.y;y++)
        {
            // check if obstacle by reverting rotation
            Point rotated_idx;
            rotated_idx = rotate(x,y,CoGIdx,-_obs.yaw);
            if(obs_start_xyIdx.x < rotated_idx.x && rotated_idx.x < obs_end_xyIdx.x &&
                obs_start_xyIdx.y < rotated_idx.y && rotated_idx.y < obs_end_xyIdx.y)
                LabelMap.at<uchar>(y, x) = obstacle;
        }
    }

    // visualize LabelMap - for debugging
    // for(int x=(map_widthIdx-1);x>=0;x--)
    // {
    //     for(int y=(map_heightIdx-1);y>=0;y--)
    //     {
    //         if( LabelMap.at<uchar>(y,x) == free )
    //             std::cout << "0 ";
    //         if( LabelMap.at<uchar>(y,x) == obstacle )
    //             std::cout << "1 ";
    //     }
    //     std::cout << std::endl;
    // }
    
}

Node* Astar::FindPath()
{
    int width = map_widthIdx;
    int height = map_heightIdx;
    Mat _LabelMap = LabelMap.clone();

    // Add startPoint to OpenList
    Node* startPointNode = new Node(startPoint);
    OpenList.push(pair<int, Point>(startPointNode->F, startPointNode->point));
    int index = point2index(startPointNode->point);
    OpenDict[index] = startPointNode;
    _LabelMap.at<uchar>(startPoint.y, startPoint.x) = inOpenList;

    while(!OpenList.empty())
    {
        // Find the node with least F value
        Point CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);
        Node* CurNode = OpenDict[index];
        OpenDict.erase(index);

        int curX = CurPoint.x;
        int curY = CurPoint.y;
        _LabelMap.at<uchar>(curY, curX) = inCloseList;

        // Determine whether arrive the target point
        if(curX == targetPoint.x && curY == targetPoint.y)
        {
            return CurNode; // Find a valid path
        }

        // Traversal the neighborhood
        for(int k = 0;k < neighbor.rows;k++)
        {
            int y = curY + neighbor.at<char>(k, 0);
            int x = curX + neighbor.at<char>(k, 1);
            if(x < 0 || x >= width || y < 0 || y >= height)
            {
                continue;
            }
            if(_LabelMap.at<uchar>(y, x) == free || _LabelMap.at<uchar>(y, x) == inOpenList)
            {
                // Determine whether a diagonal line can pass
                int dist1 = abs(neighbor.at<char>(k, 0)) + abs(neighbor.at<char>(k, 1));
                if(dist1 == 2 && _LabelMap.at<uchar>(y, curX) == obstacle && _LabelMap.at<uchar>(curY, x) == obstacle)
                    continue;

                // Calculate G, H, F value
                int addG, G, H, F;
                if(dist1 == 2)
                {
                    addG = 14;
                }
                else
                {
                    addG = 10;
                }
                G = CurNode->G + addG;
                if(config.Euclidean)
                {
                    int dist2 = (x - targetPoint.x) * (x - targetPoint.x) + (y - targetPoint.y) * (y - targetPoint.y);
                    H = round(10 * sqrt(dist2));
                }
                else
                {
                    H = 10 * (abs(x - targetPoint.x) + abs(y - targetPoint.y));
                }
                F = G + H;

                // Update the G, H, F value of node
                if(_LabelMap.at<uchar>(y, x) == free)
                {
                    Node* node = new Node();
                    node->point = Point(x, y);
                    node->parent = CurNode;
                    node->G = G;
                    node->H = H;
                    node->F = F;
                    OpenList.push(pair<int, Point>(node->F, node->point));
                    int index = point2index(node->point);
                    OpenDict[index] = node;
                    _LabelMap.at<uchar>(y, x) = inOpenList;
                }
                else // _LabelMap.at<uchar>(y, x) == inOpenList
                {
                    // Find the node
                    int index = point2index(Point(x, y));
                    Node* node = OpenDict[index];
                    if(G < node->G)
                    {
                        node->G = G;
                        node->F = F;
                        node->parent = CurNode;
                    }
                }
            }
        }
    }

    return NULL; // Can not find a valid path
}

void Astar::GetPath(Node* TailNode, vector<Point>& path)
{
    PathList.clear();
    path.clear();

    // Save path to PathList
    Node* CurNode = TailNode;
    while(CurNode != NULL)
    {
        PathList.push_back(CurNode);
        CurNode = CurNode->parent;
    }

    // Save path to vector<Point>
    int length = PathList.size();
    for(int i = 0;i < length;i++)
    {
        path.push_back(PathList.back()->point);
        PathList.pop_back();
    }

    // Release memory
    while(OpenList.size()) {
        Point CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);
        Node* CurNode = OpenDict[index];
        delete CurNode;
    }
    OpenDict.clear();
}

int Astar::sat(int val, int min, int max)
{
    int return_ = val;
    if(val < min)
        return_ = min;
    else if(val > max)
        return_ = max;

    return return_;    
}

Point Astar::rotate(int x, int y, Point CoGIdx, double yaw)
{
    Eigen::Vector2i xy, CoGIdx_eigen;
    xy(0) = x; xy(1) = y;
    CoGIdx_eigen(0) = CoGIdx.x; CoGIdx_eigen(1) = CoGIdx.y;
    
    Eigen::Matrix2d Rot_mat;
    Rot_mat(0,0) = cos(yaw); Rot_mat(0,1) = -sin(yaw);
    Rot_mat(1,0) = sin(yaw); Rot_mat(1,1) = cos(yaw);
    Eigen::Vector2d temp_ = Rot_mat*(xy-CoGIdx_eigen).cast<double>();

    Point rotated_xy;
    rotated_xy.x = CoGIdx.x + round(temp_(0));
    rotated_xy.y = CoGIdx.y + round(temp_(1));

    return rotated_xy;
}

}