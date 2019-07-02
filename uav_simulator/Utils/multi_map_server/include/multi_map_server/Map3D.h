#ifndef MAP3D_H
#define MAP3D_H

#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <armadillo>
#include <multi_map_server/SparseMap3D.h>

using namespace std;

// Occupancy probability of a sensor
#define PROB_OCCUPIED  0.95

// Free space probability of a sensor
#define PROB_FREE      0.01

// Threshold that determine a cell is occupied
#define PROB_OCCUPIED_THRESHOLD       0.75

// If beyond this threshold, the occupancy(occupied) of this cell is fixed, no decay
#define PROB_OCCUPIED_FIXED_THRESHOLD 0.95

// Threshold that determine a cell is free
#define PROB_FREE_THRESHOLD           0.25

// If bwlow this threshold, the occupancy(free) of this cell is fixed, no decay
#define PROB_FREE_FIXED_THRESHOLD     0.05

// Used integer value to store occupancy, create a large scale factor to enable small scale decay
#define LOG_ODD_SCALE_FACTOR  10000

// Decay factor per second
#define LOG_ODD_DECAY_RATE 1.00

// Binary value of the occupancy output
#define OCCUPIED 1
#define FREE    -1

// Cell Struct -----------------------------------------
struct OccupancyGrid
{
  int upper;
  int lower;
  int mass;
};

// Occupancy Grids List  --------------------------------
class OccupancyGridList
{
public:

  OccupancyGridList() { updateCounter = 0; }

  ~OccupancyGridList() { }

  void PackMsg(multi_map_server::VerticalOccupancyGridList &msg)
  {
    msg.x = x;
    msg.y = y;
    for (list<OccupancyGrid>::iterator k = grids.begin(); k != grids.end(); k++)
    {
      msg.upper.push_back(k->upper);
      msg.lower.push_back(k->lower);
      msg.mass.push_back(k->mass);
    }
  }

  void UnpackMsg(const multi_map_server::VerticalOccupancyGridList &msg)
  {
    x = msg.x;
    y = msg.y;
    updateCounter = 0;
    grids.clear();
    for (unsigned int k = 0; k < msg.mass.size(); k++)
    {
      OccupancyGrid c;
      c.upper = msg.upper[k];
      c.lower = msg.lower[k];
      c.mass  = msg.mass[k];
      grids.push_back(c);
    }
  }

  void GetOccupancyGrids(vector<OccupancyGrid>& _grids)
  {
    _grids.clear();
    for (list<OccupancyGrid>::iterator k = grids.begin(); k != grids.end(); k++)
      _grids.push_back((*k));
  }

  inline int GetUpdateCounter() { return updateCounter; }

  inline void SetUpdateCounterXY(int _updateCounter, double _x, double _y) 
  { 
    updateCounter = _updateCounter; 
    x = _x;
    y = _y;
  }

  inline int GetOccupancyValue(int mz)
  {
    for (list<OccupancyGrid>::iterator k = grids.begin(); k != grids.end(); k++)
      if (mz <= k->upper && mz >= k->lower)
        return k->mass / (k->upper - k->lower + 1);
    return 0;
  }

  inline void DeleteOccupancyGrid(int mz)
  {
    for (list<OccupancyGrid>::iterator k = grids.begin(); k != grids.end(); k++)
    {
      if (mz <= k->upper && mz >= k->lower)
      {
        grids.erase(k);
        return;
      }
    }
    return;
  } 
  
  inline void SetOccupancyValue(int mz, int value)
  {
    OccupancyGrid grid;
    grid.upper = mz;
    grid.lower = mz;
    grid.mass  = value;

    list<OccupancyGrid>::iterator gend = grids.end();
    gend--;

    if (grids.size() == 0)                   // Empty case
    {
      grids.push_back(grid);
      return;
    }
    else if (mz - grids.begin()->upper > 1)  // Beyond highest
    {
      grids.push_front(grid);
      return;
    }
    else if (mz - grids.begin()->upper == 1) // Next to highest
    {
      grids.begin()->upper += 1;
      grids.begin()->mass  += value;
      return;
    }
    else if (gend->lower - mz > 1)           // Below lowest
    {
      grids.push_back(grid);
      return;
    }
    else if (gend->lower - mz == 1)          // Next to lowest
    {
      grids.end()->lower -= 1;
      grids.end()->mass  += value;
      return;
    }
    else                                     // General case
    {
      for (list<OccupancyGrid>::iterator k = grids.begin(); k != grids.end(); k++)
      {
        if (mz <= k->upper && mz >= k->lower) // Within a grid
        {
          k->mass += value;
          return;
        }
        else if (k != gend)
        {
          list<OccupancyGrid>::iterator j = k;
          j++;
          if (k->lower - mz == 1 && mz - j->upper > 1) // ###*--###
          {
            k->lower -= 1;
            k->mass  += value;
            return;
          }
          else if (k->lower - mz > 1 && mz - j->upper == 1) // ###--*###
          {
            j->upper += 1;
            j->mass  += value;
            return;
          }
          else if (k->lower - mz == 1 && mz - j->upper == 1) // ###*###
          {
            k->lower = j->lower;
            k->mass += j->mass + value;
            grids.erase(j);
            return;
          }
          else if (k->lower - mz > 1 && mz - j->upper > 1) // ###-*-###
          {
            grids.insert(j, grid);
            return;
          }
        } 
      }
    }
  }

  // Merging two columns, merge the grids in input "gridList" into current column
  inline void Merge(const OccupancyGridList& gridsIn)
  {
    // Create a sorted list containing both upper and lower values
    list<pair<int, int> > lp;
    for (list<OccupancyGrid>::const_iterator k = grids.begin(); k != grids.end(); k++)
    {
      lp.push_back( make_pair(k->upper, k->mass));
      lp.push_back( make_pair(k->lower, -1));
    }
    list<pair<int, int> > lp2;
    for (list<OccupancyGrid>::const_iterator k = gridsIn.grids.begin(); k != gridsIn.grids.end(); k++)
    {
      lp2.push_back( make_pair(k->upper, k->mass));
      lp2.push_back( make_pair(k->lower, -1));
    }
    lp.merge(lp2, ComparePair());
    // Manipulate this list to get a minimum size list
    grids.clear();
    int currUpper = 0;
    int currLower = 0;
    int currMass  = 0;
    int upperCnt = 0;
    int lowerCnt = 0;
    list<pair<int, int> >::iterator lend = lp.end();
    lend--;
    for (list<pair<int, int> >::iterator k = lp.begin(); k != lp.end(); k++)
    {
      if (k->second > 0) 
      { 
        if (upperCnt == 0) currUpper = k->first;
        currMass = (k->second > currMass)?k->second:currMass; 
        upperCnt++; 
      }
      if (k->second < 0) 
      { 
        currLower = k->first;
        lowerCnt++; 
      }
      if (lowerCnt == upperCnt && k != lend)
      {
        list<pair<int, int> >::iterator j = k;
        if (k->first - (++j)->first == 1) continue;
      }
      if (lowerCnt == upperCnt)
      {
        OccupancyGrid c;
        c.upper = currUpper;
        c.lower = currLower;
        c.mass  = currMass;
        grids.push_back(c);
        upperCnt = lowerCnt = currUpper = currLower = currMass = 0;
      }
    }
  }

  inline void Decay(int upThr, int lowThr, double factor)
  {
    for (list<OccupancyGrid>::iterator k = grids.begin(); k != grids.end(); k++)
    {
      int val = k->mass / (k->upper - k->lower + 1);
      if (val < upThr && val > lowThr)
        k->mass *= factor;
    }
  }

private:

  struct ComparePair
  {
    bool operator()(pair<int, int> p1, pair<int, int> p2) 
    { 
      if (p1.first != p2.first) 
        return (p1.first > p2.first);
      else 
        return (p1.second > p2.second);
    }
  };

  // List of vertical occupancy values
  list<OccupancyGrid> grids;
  // Location of the list in world frame
  double x;
  double y;
  // Update indicator
  int updateCounter;

};

// 3D Map Object  ---------------------------------
class Map3D
{
public:

  Map3D() 
  {
    resolution = 0.1;
    decayInterval = -1;
    originX = -5;
    originY = -5;
    originZ =  0;
    mapX = 200;
    mapY = 200;
    expandStep = 200;
    updated = false;
    updateCounter = 1;
    updateList.clear();
    mapBase.clear();
    mapBase.resize(mapX*mapY, NULL);
    logOddOccupied = log(PROB_OCCUPIED/(1.0-PROB_OCCUPIED)) * LOG_ODD_SCALE_FACTOR;
    logOddFree = log(PROB_FREE/(1.0-PROB_FREE)) * LOG_ODD_SCALE_FACTOR;
    logOddOccupiedThr = log(1.0/(1.0-PROB_OCCUPIED_THRESHOLD) - 1.0) * LOG_ODD_SCALE_FACTOR;
    logOddOccupiedFixedThr = log(1.0/(1.0-PROB_OCCUPIED_FIXED_THRESHOLD) - 1.0) * LOG_ODD_SCALE_FACTOR;
    logOddFreeThr = log(1.0/(1.0-PROB_FREE_THRESHOLD) - 1.0) * LOG_ODD_SCALE_FACTOR;
    logOddFreeFixedThr = log(1.0/(1.0-PROB_FREE_FIXED_THRESHOLD) - 1.0) * LOG_ODD_SCALE_FACTOR;
  }
  
  Map3D(const Map3D& _map3d)
  {
    resolution = _map3d.resolution;
    decayInterval = _map3d.decayInterval;
    originX = _map3d.originX;
    originY = _map3d.originY;
    originZ = _map3d.originZ;
    mapX = _map3d.mapX;
    mapY = _map3d.mapY;
    expandStep = _map3d.expandStep;
    updated = _map3d.updated;
    updateCounter = _map3d.updateCounter;
    updateList = _map3d.updateList;
    mapBase = _map3d.mapBase;
    for (unsigned int k = 0; k < mapBase.size(); k++)
    {
      if (mapBase[k])
      {
        mapBase[k] = new OccupancyGridList;
        *mapBase[k] = *_map3d.mapBase[k];
      }
    }
    logOddOccupied = log(PROB_OCCUPIED/(1.0-PROB_OCCUPIED)) * LOG_ODD_SCALE_FACTOR;
    logOddFree = log(PROB_FREE/(1.0-PROB_FREE)) * LOG_ODD_SCALE_FACTOR;
    logOddOccupiedThr = log(1.0/(1.0-PROB_OCCUPIED_THRESHOLD) - 1.0) * LOG_ODD_SCALE_FACTOR;
    logOddOccupiedFixedThr = log(1.0/(1.0-PROB_OCCUPIED_FIXED_THRESHOLD) - 1.0) * LOG_ODD_SCALE_FACTOR;
    logOddFreeThr = log(1.0/(1.0-PROB_FREE_THRESHOLD) - 1.0) * LOG_ODD_SCALE_FACTOR;
    logOddFreeFixedThr = log(1.0/(1.0-PROB_FREE_FIXED_THRESHOLD) - 1.0) * LOG_ODD_SCALE_FACTOR;    
  }

  ~Map3D() 
  {
    for (unsigned int k = 0; k < mapBase.size(); k++)
    {
      if (mapBase[k])
      {
        delete mapBase[k];    
        mapBase[k] = NULL;
      }
    }
  }

  void PackMsg(multi_map_server::SparseMap3D &msg)
  {
    // Basic map info
    msg.header.stamp            = ros::Time::now();
    msg.header.frame_id         = string("/map");
    msg.info.map_load_time      = ros::Time::now();
    msg.info.resolution         = resolution;
    msg.info.origin.position.x  = originX;
    msg.info.origin.position.y  = originY;
    msg.info.origin.position.z  = originZ;
    msg.info.width              = mapX;
    msg.info.height             = mapY;
    msg.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);  
    // Pack columns into message
    msg.lists.clear();
    for (unsigned int k = 0; k < updateList.size(); k++)
    {
      multi_map_server::VerticalOccupancyGridList c;
      updateList[k]->PackMsg(c);
      msg.lists.push_back(c);
    }
    updateList.clear();
    updateCounter++;
  }

  void UnpackMsg(const multi_map_server::SparseMap3D &msg)
  {
    // Unpack column msgs, Replace the whole column
    for (unsigned int k = 0; k < msg.lists.size(); k++)
    {
      int mx, my, mz;
      WorldFrameToMapFrame(msg.lists[k].x, msg.lists[k].y, 0, mx, my, mz);
      ResizeMapBase(mx, my);
      if (mapBase[my*mapX+mx]) delete mapBase[my*mapX+mx];
      mapBase[my*mapX+mx] = new OccupancyGridList;
      mapBase[my*mapX+mx]->UnpackMsg(msg.lists[k]);
    }
    CheckDecayMap();
    updated = true;
  }

  inline void SetOccupancyFromWorldFrame(double x, double y, double z, double p = PROB_OCCUPIED)
  {
    // Find occupancy value to be set
    int value = 0;
    if (p == PROB_OCCUPIED) value = logOddOccupied;
    else if (p == PROB_FREE) value = logOddFree;
    else return;
    // Update occupancy value, return the column that have been changed
    int mx, my, mz;
    WorldFrameToMapFrame(x, y, z, mx, my, mz);
    ResizeMapBase(mx, my);
    if (!mapBase[my*mapX+mx]) 
      mapBase[my*mapX+mx] = new OccupancyGridList;
    mapBase[my*mapX+mx]->SetOccupancyValue(mz, value);
    // Also record the column that have been changed in another list, for publish incremental map
    if (mapBase[my*mapX+mx]->GetUpdateCounter() != updateCounter)
    {
      updateList.push_back(mapBase[my*mapX+mx]); 
      mapBase[my*mapX+mx]->SetUpdateCounterXY(updateCounter, x, y);
    }
    updated = true;
  }

  inline int GetOccupancyFromWorldFrame(double x, double y, double z)
  {
    int mx, my, mz;
    WorldFrameToMapFrame(x, y, z, mx, my, mz);
    if (mx < 0 || my < 0 || mx >= mapX || my >= mapY)
      return 0;
    if (!mapBase[my*mapX+mx])
      return 0;
    else
    {
      if (mapBase[my*mapX+mx]->GetOccupancyValue(mz) > logOddOccupiedThr)
        return 1;
      else if (mapBase[my*mapX+mx]->GetOccupancyValue(mz) < logOddFreeThr)
        return -1;
      else 
        return 0;
    }
  }

  inline void DeleteFromWorldFrame(double x, double y, double z)
  {
    int mx, my, mz;
    WorldFrameToMapFrame(x, y, z, mx, my, mz);
    if (mx < 0 || my < 0 || mx >= mapX || my >= mapY)
      return;
    if (mapBase[my*mapX+mx])
      mapBase[my*mapX+mx]->DeleteOccupancyGrid(mz);
  }

  vector<arma::colvec>& GetOccupancyWorldFrame(int type = OCCUPIED)
  {
    pts.clear();
    for (int mx = 0; mx < mapX; mx++)
    {
      for (int my = 0; my < mapY; my++)
      {
        if (mapBase[my*mapX+mx])
        {
          vector<OccupancyGrid> grids;
          mapBase[my*mapX+mx]->GetOccupancyGrids(grids);
          for (unsigned int k = 0; k < grids.size(); k++)
          {
            if  ( (grids[k].mass / (grids[k].upper - grids[k].lower + 1) > logOddOccupiedThr && type == OCCUPIED) || 
                  (grids[k].mass / (grids[k].upper - grids[k].lower + 1) < logOddFreeThr     && type == FREE) )
            {
              for (int mz = grids[k].lower; mz <= grids[k].upper; mz++)
              {
                double x, y, z;
                MapFrameToWorldFrame(mx, my, mz, x, y, z);
                arma::colvec pt(3);
                pt(0) = x;
                pt(1) = y;
                pt(2) = z;
                pts.push_back(pt);
              }
            }
          }
        }
      }
    }
    return pts;
  }

  // Do not allow setting parameters if at least one update received
  void SetResolution(double _resolution) 
  {
    if (!updated)
      resolution = _resolution;
  }
  
  void SetDecayInterval(double _decayInterval) 
  {
    if (!updated && _decayInterval > 0)
      decayInterval = _decayInterval;
  }  

  inline double GetResolution() { return resolution; }
  inline double GetMaxX() { return originX + mapX*resolution; }
  inline double GetMinX() { return originX; }
  inline double GetMaxY() { return originY + mapY*resolution; }
  inline double GetMinY() { return originY; }
  inline bool Updated() { return updated; }

private:

  inline void WorldFrameToMapFrame(double x, double y, double z, int& mx, int& my, int& mz)
  {
    mx = (x - originX) / resolution;
    my = (y - originY) / resolution;
    mz = (z - originZ) / resolution;
  }

  inline void MapFrameToWorldFrame(int mx, int my, int mz, double& x, double& y, double& z)
  {
    double r = 0.5*resolution;
    x = mx * resolution + r + originX;
    y = my * resolution + r + originY;
    z = mz * resolution + r + originZ;
  }

  inline void ResizeMapBase(int& mx, int& my)
  {
    if (mx < 0 || my < 0 || mx >= mapX || my >= mapY)
    {
      double prevOriginX = originX;
      double prevOriginY = originY;
      int    prevMapX    = mapX;
      int    prevMapY    = mapY;
      while(mx < 0)
      {
        mapX    += expandStep;
        mx      += expandStep;
        originX -= expandStep*resolution;
      }
      while(my < 0)
      {
        mapY    += expandStep;
        my      += expandStep;
        originY -= expandStep*resolution;
      }
      while(mx >= mapX)
      {
        mapX    += expandStep;
      } 
      while(my >= mapY)
      {
        mapY    += expandStep;
      } 
      vector<OccupancyGridList*> _mapBase = mapBase;
      mapBase.clear();
      mapBase.resize(mapX*mapY,NULL);
      for (int _x = 0; _x < prevMapX; _x++)
      {
        for(int _y = 0; _y < prevMapY; _y++)
        {
          int x = _x + round((prevOriginX - originX) / resolution);
          int y = _y + round((prevOriginY - originY) / resolution);
          mapBase[y*mapX+x] = _mapBase[_y*prevMapX+_x];
        }
      }
    }
  }

  void CheckDecayMap()
  {
    if (decayInterval < 0)
      return;
    // Check whether to decay
    static ros::Time prevDecayT = ros::Time::now();
    ros::Time t = ros::Time::now();
    double dt = (t - prevDecayT).toSec();
    if (dt > decayInterval)
    {
      double r = pow(LOG_ODD_DECAY_RATE, dt);
      for (int mx = 0; mx < mapX; mx++)
        for (int my = 0; my < mapY; my++)
          if (mapBase[my*mapX+mx])
            mapBase[my*mapX+mx]->Decay(logOddOccupiedFixedThr, logOddFreeFixedThr, r);
      prevDecayT = t;
    }
  }

  double resolution;
  double decayInterval;

  int logOddOccupied;
  int logOddFree;
  int logOddOccupiedThr;
  int logOddOccupiedFixedThr;
  int logOddFreeThr;
  int logOddFreeFixedThr;

  bool updated;
  int updateCounter;
  vector<OccupancyGridList*> updateList;

  double originX, originY, originZ;
  int mapX, mapY;
  int expandStep;
  vector<OccupancyGridList*> mapBase;

  vector<arma::colvec> pts;

};
#endif
