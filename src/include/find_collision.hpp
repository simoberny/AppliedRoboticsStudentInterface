//
// Created by osboxes on 11/22/19.
//

#ifndef STUDENT_PROJECT_FIND_COLLISION_HPP
#define STUDENT_PROJECT_FIND_COLLISION_HPP
#pragma once
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"



class Find_collision {

   public:
        Find_collision();
        bool robot_obstacles_intersection(const std::vector<Polygon>& obstacle_list, const float x, const float y, const float theta);
       //bool obstacles_line_intersection
       //bool obstcles_arc_intersection



};




#endif //STUDENT_PROJECT_FIND_COLLISION_HPP
