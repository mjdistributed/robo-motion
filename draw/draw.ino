
#include "structs.h"


void
setup(){
  
}

int**
get_mat() {
  /*
   * area_mat is a bit matrix corresponding to 1cm square areas.
   * If there's a "1" in area_mat, then the robot will draw there
   */
  int** area_mat;
  int i;
  int j;
 
  area_mat = (int**)malloc(100 * 100);
  // "calloc" or something inits to 0 but i'll fix this later
  for(i = 0; i < 100; i ++) {
    for(j = 0; j < 100; j ++) {
      area_mat[i][j] = 0;
    }
  }
  
  return area_mat;

}



/*
 * Fill in the desired pattern in the area matrix
 * TODO: GUI
 */
int**
fill_pattern(int** area_mat) {
  //for now, hard-code in a pattern
  return area_mat;
}

/*
 * Sends the robot from curr_pos to end_pos
 */
void
send_to_pos(pos start_pos, pos end_pos) {

  return;
}


/*
 * Returns the closest position yet to be filled in area_mat
 * 
 * Requires: area_mat valid bit matrix
 */
pos 
find_closest_pos(int** area_mat, pos curr_pos) {

  return curr_pos;
}

/*
 * Marks the current position as filled
 *
 * Requires: area_mat is valid bit matrix, curr_pos valid position in area_mat
 */
void
mark_filled(int **area_mat, pos to_mark) {
  area_mat[to_mark.x][to_mark.y] = 0;
}

void 
loop() {
  pos curr_pos;
  curr_pos.x = curr_pos.y = 0;
  int** area_mat = get_mat();
  pos next_pos = find_closest_pos(area_mat, curr_pos);
  send_to_pos(curr_pos, next_pos);
  curr_pos = next_pos;
  mark_filled(area_mat, curr_pos);
}

