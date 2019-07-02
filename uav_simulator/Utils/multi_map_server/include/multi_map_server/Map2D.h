#ifndef MAP2D_H
#define MAP2D_H

#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;

class Map2D
{

private:
  nav_msgs::OccupancyGrid map;
  int  expandStep;  
  int  binning;  
  bool isBinningSet;
  bool updated;

public:
  Map2D() 
  {
    map.data.resize(0); 
    map.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);  
    expandStep   = 200; 
    binning      = 1; 
    isBinningSet = false; 
    updated      = false; 
  }

  Map2D(int _binning)
  {
    map.data.resize(0); 
    map.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);  
    expandStep   = 200; 
    binning      = _binning; 
    isBinningSet = true; 
    updated      = false; 
  }

  ~Map2D() {}

  double GetResolution() { return map.info.resolution; }
  double GetMinX() { return map.info.origin.position.x; }
  double GetMinY() { return map.info.origin.position.y; }
  double GetMaxX() { return map.info.origin.position.x + map.info.width  * map.info.resolution; }
  double GetMaxY() { return map.info.origin.position.y + map.info.height * map.info.resolution; }
  bool   Updated() { return updated; }  
  void   Reset()   { map = nav_msgs::OccupancyGrid(); }

  void SetBinning(int _binning) 
  { 
    if (!isBinningSet)
      binning = _binning; 
  }

  // Get occupancy value, 0: unknown; +ve: occupied; -ve: free
  signed char GetOccupiedFromWorldFrame(double x, double y) 
  {
    int xm = (x - map.info.origin.position.x) / map.info.resolution;
    int ym = (y - map.info.origin.position.y) / map.info.resolution;
    if (xm < 0 || xm > map.info.width-1 || ym < 0 || ym > map.info.height-1) 
      return 0;
    else 
      return map.data[ym*map.info.width+xm];
  }

  void Replace(nav_msgs::OccupancyGrid m)
  {
    // Check data
    if (m.data.size() == 0)
      return;
    isBinningSet = true;
    // Binning, conservative, take maximum
    if (binning > 1)
    {
      int _width         = m.info.width;
      m.info.width      /= binning;
      m.info.height     /= binning;
      m.info.resolution *= binning;
      vector<signed char> data(m.info.width * m.info.height);
      for (int i = 0; i < m.info.height; i++)
      {
        for (int j = 0; j < m.info.width; j++)
        {
          int val = -0xff;
          for (int _i = 0; _i < binning; _i++)
          {
            for (int _j = 0; _j < binning; _j++)
            {
              int v = m.data[(i*binning + _i) * _width + (j*binning + _j)];
              val = (v > val)?v:val;
            }
          }
          data[i * m.info.width + j] = val;
        }
      }
      m.data = data;
    }
    // Replace map
    map = m;
    updated = true;
  }

  // Merge submap
  void Update(nav_msgs::OccupancyGrid m)
  {
    // Check data
    if (m.data.size() == 0)
      return;
    isBinningSet = true;
    // Binning, conservative, take maximum
    if (binning > 1)
    {
      int _width         = m.info.width;
      m.info.width      /= binning;
      m.info.height     /= binning;
      m.info.resolution *= binning;
      vector<signed char> data(m.info.width * m.info.height);
      for (int i = 0; i < m.info.height; i++)
      {
        for (int j = 0; j < m.info.width; j++)
        {
          int val = -0xff;
          for (int _i = 0; _i < binning; _i++)
          {
            for (int _j = 0; _j < binning; _j++)
            {
              int v = m.data[(i*binning + _i) * _width + (j*binning + _j)];
              val = (v > val)?v:val;
            }
          }
          data[i * m.info.width + j] = val;
        }
      }
      m.data = data;
    }
    // Initialize and check resolution
    if (map.info.resolution == 0)
      map.info.resolution = m.info.resolution;
    else if (m.info.resolution != map.info.resolution)
      return;
    // Get Info
    double ox   = m.info.origin.position.x;
    double oy   = m.info.origin.position.y;
    double oyaw = tf::getYaw(m.info.origin.orientation);
    double syaw = sin(oyaw);
    double cyaw = cos(oyaw);
    int mx      = m.info.width;
    int my      = m.info.height;
    double xs[4];
    double ys[4];
    xs[0] = cyaw * (0)                      - syaw * (0)                      + ox;
    xs[1] = cyaw * (mx*map.info.resolution) - syaw * (0)                      + ox;
    xs[2] = cyaw * (0)                      - syaw * (mx*map.info.resolution) + ox;
    xs[3] = cyaw * (mx*map.info.resolution) - syaw * (my*map.info.resolution) + ox;    
    ys[0] = syaw * (0)                      + cyaw * (0)                      + oy;
    ys[1] = syaw * (mx*map.info.resolution) + cyaw * (0)                      + oy;
    ys[2] = syaw * (0)                      + cyaw * (my*map.info.resolution) + oy;
    ys[3] = syaw * (mx*map.info.resolution) + cyaw * (my*map.info.resolution) + oy;
    double minx = xs[0];
    double maxx = xs[0];
    double miny = ys[0];
    double maxy = ys[0];
    for (int k = 0; k < 4; k++)
    {
      minx = (xs[k] < minx)?xs[k]:minx;
      miny = (ys[k] < miny)?ys[k]:miny;
      maxx = (xs[k] > maxx)?xs[k]:maxx;
      maxy = (ys[k] > maxy)?ys[k]:maxy;
    }
    // Check whether resize map is needed
    bool   mapResetFlag = false;
    double prevOriginX  = map.info.origin.position.x;
    double prevOriginY  = map.info.origin.position.y;
    int    prevMapX     = map.info.width;
    int    prevMapY     = map.info.height;
    while (map.info.origin.position.x > minx)
    {
      map.info.width += expandStep;
      map.info.origin.position.x -= expandStep*map.info.resolution;
      mapResetFlag = true;
    }
    while (map.info.origin.position.y > miny)
    {
      map.info.height += expandStep;
      map.info.origin.position.y -= expandStep*map.info.resolution;
      mapResetFlag = true;
    }
    while (map.info.origin.position.x + map.info.width*map.info.resolution < maxx)
    {
      map.info.width += expandStep;
      mapResetFlag = true;
    }
    while (map.info.origin.position.y + map.info.height*map.info.resolution < maxy)
    {
      map.info.height += expandStep;
      mapResetFlag = true;
    }
    // Resize map
    if (mapResetFlag)
    {
      mapResetFlag = false;
      vector<signed char> _data = map.data;
      map.data.clear();
      map.data.resize(map.info.width*map.info.height,0);
      for (int x = 0; x < prevMapX; x++)
      {
        for(int y = 0; y < prevMapY; y++)
        {
          int xn = x + round((prevOriginX - map.info.origin.position.x) / map.info.resolution);
          int yn = y + round((prevOriginY - map.info.origin.position.y) / map.info.resolution);
          map.data[yn*map.info.width+xn] = _data[y*prevMapX+x];
        }
      }
    }
    // Merge map
    for (int x = 0; x < mx; x++)
    {
      for(int y = 0; y < my; y++)
      {
        int xn = (cyaw*(x*map.info.resolution)-syaw*(y*map.info.resolution)+ox-map.info.origin.position.x) / map.info.resolution;
        int yn = (syaw*(x*map.info.resolution)+cyaw*(y*map.info.resolution)+oy-map.info.origin.position.y) / map.info.resolution;
        if (abs((int)(map.data[yn*map.info.width+xn]) + (int)(m.data[y*mx+x])) <= 127)        
          map.data[yn*map.info.width+xn] += m.data[y*mx+x];
      }
    }
    updated = true;
  }

  const nav_msgs::OccupancyGrid& GetMap()
  {
    map.header.stamp       = ros::Time::now();
    map.info.map_load_time = ros::Time::now();
    map.header.frame_id    = string("/map");
    updated = false;
    return map;
  }
};

#endif
