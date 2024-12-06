#include "parse_moves.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "cJSON.h"

// Function to parse the JSON string and extract robot move information
int parse_robot_move(const char *json_str, RobotMove *move) {
    // Initialize the move structure
    for (int i = 0; i < 6; i++) {
        move->move[i] = NAN;  // Set default as invalid for all joints
    }
    move->move_type = 0;  // Default to absolute
    move->duration = -1; // Default invalid duration

    // Parse the JSON string
    cJSON *json = cJSON_Parse(json_str);
    if (json == NULL) {
        return 0; // Parsing failed
    }

    // Parse the axis_angles array
    cJSON *axis_angles = cJSON_GetObjectItemCaseSensitive(json, "axis_angles");
    if (cJSON_IsArray(axis_angles)) {
        cJSON *axis_angle = NULL;
        cJSON_ArrayForEach(axis_angle, axis_angles) {
            // Parse axis and angle for each entry
            cJSON *axis = cJSON_GetObjectItemCaseSensitive(axis_angle, "axis");
            cJSON *angle = cJSON_GetObjectItemCaseSensitive(axis_angle, "angle");

            if (cJSON_IsNumber(axis) && cJSON_IsNumber(angle)) {
                int axis_index = axis->valueint;
                if (axis_index >= 0 && axis_index < 6) {
                    // Store the angle in the corresponding axis
                    move->move[axis_index] = angle->valuedouble;
                }
            }
        }
    }

    // Parse the type ("absolute" or "delta")
    cJSON *type = cJSON_GetObjectItemCaseSensitive(json, "type");
    if (cJSON_IsString(type) && (strcmp(type->valuestring, "delta") == 0)) {
        move->move_type = 1; // Mark as delta if the type is "delta"
    }

    // Parse duration
    cJSON *duration = cJSON_GetObjectItemCaseSensitive(json, "duration");
    if (cJSON_IsNumber(duration)) {
        move->duration = duration->valuedouble;
    }

    // Cleanup JSON object
    cJSON_Delete(json);
    return 1; // Parsing succeeded
}

void print_move(RobotMove move)
{
    printf("Parsed Move Information:\n");
    printf("Joint Moves: ");
    for (int i = 0; i < 6; i++) {
        if (!isnan(move.move[i])) {
            if (move.move_type == 1) {
                printf("Delta(%.2f) ", move.move[i]);
            } else {
                printf("Absolute(%.2f) ", move.move[i]);
            }
        } else {
            printf("null ");
        }
    }
    printf("\nDuration: %.2f seconds\n", move.duration);
}
