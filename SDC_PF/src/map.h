//
// Created by nick on 17-8-9.
//

#ifndef SDC_PF_MAP_H
#define SDC_PF_MAP_H

class Map {
public:

    struct single_landmark_s{
        int id_i;  // Landkdmark ID
        float x_f; // Landmark x-position int the map (global coordinates)
        float y_f; // Landmark y-position in the map (global coordinates)
    };

    std::vector<single_landmark_s> landmark_list; // List the landmarks in the map
};


#endif //SDC_PF_MAP_H
