// Load's a 2D map.
#pragma once

#include <string>
#include <vector>
#include "common.h"

using namespace std;




class MapLoader 
{
 public:
  bool* my_map;
  int rows;
  int cols;

  //for kiva map
  int workpoint_num;
  int maxtime;



    int start_loc;
  int goal_loc;

    enum valid_moves_t { NORTH, EAST, SOUTH, WEST, WAIT_MOVE, INVALID, MOVE_COUNT };  // MOVE_COUNT is the enum's size
    int* moves_offset;

    enum valid_actions_t { WAIT_ACTION, MOVE, ROTATE_L, ROTATE_R, ACTIONS_COUNT};
    int* actions_offset;

    enum orientation_t { FACE_NORTH, FACE_EAST, FACE_SOUTH, FACE_WEST, ORIENTATION_COUNT};
  bool validMove(int curr, int next) const;

  MapLoader(std::string fname); // load map from file
  MapLoader(int rows, int cols); // initialize new [rows x cols] empty map
  MapLoader();

  vector<pair<int, int>> get_transitions(int loc, int heading, int noWait) const;
  bool getLoc(int loc) ;
  inline bool is_blocked (int row, int col) const { return my_map[row * this->cols + col]; }
  inline bool is_blocked (int loc) const { return my_map[loc]; }
  inline size_t map_size() const { return rows * cols; }
  void printMap ();
  void printMap (char* mapChar);
  char* mapToChar();
  bool* get_map () const; // return a deep-copy of my_map
  inline int linearize_coordinate(int row, int col) const { return ( this->cols * row + col); }
  inline int row_coordinate(int id) const { return id / this->cols; }
  inline int col_coordinate(int id) const { return id % this->cols; }
  void printPath (std::vector<int> path);
  void saveToFile(std::string fname);
  int getDegree(int loc);
  bool isFullyBlocked(int start, int end);//check does map block entire area from start to end;
  virtual int getDistance(int initial, int target, int heading = -1){};
  virtual void setCostMap(int location, int heading = -1){};
  ~MapLoader();
};