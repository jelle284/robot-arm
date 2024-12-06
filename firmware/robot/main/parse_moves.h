#ifndef PARSE_MOVES_H
#define PARSE_MOVES_H

/* example json
{
    "axis_angles": [
        { "axis": 0, "angle": 25 },
        { "axis": 2, "angle": -15 },
        { "axis": 5, "angle": 10 }
    ],
    "type": "absolute",
    "duration": 10
}
*/

// Structure to hold the parsed robot move information
typedef struct {
    double move[6];    // Either absolute angles or delta angles for each joint
    double duration;   // Duration of the move in seconds
    int move_type;      // Flag to indicate if all angles are delta (1) or absolute (0)
} RobotMove;

int parse_robot_move(const char *json_str, RobotMove *move);

void print_move(RobotMove move);

#endif // PARSE_MOVES_H