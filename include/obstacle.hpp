// Copyright 2023 watson.wang

#include <vector>
#include "geomtry.hpp"

#ifndef OBSTACLE_HPP_
#define OBSTACLE_HPP_

using namespace std;

namespace ob {

    class obstacle {
    public:
        explicit obstacle(geo_ring poly) :_poly(poly){}
        virtual ~obstacle() = default;
      
        obstacle(const obstacle&) = default;
        obstacle(obstacle&&) noexcept = default;

        obstacle & operator = (const obstacle&) = default;
        obstacle & operator = (obstacle&&) noexcept = default;

        geo_ring getPoly(){ return _poly; };

    protected:
        geo_ring _poly{};
    };


    class stationaryObj: public obstacle {

    public:
        explicit stationaryObj(geo_ring poly):obstacle(poly){}
        virtual ~stationaryObj() = default;

        stationaryObj(const stationaryObj&) = default;
        stationaryObj(stationaryObj&&) noexcept = default;

        stationaryObj& operator = (const stationaryObj&) = default;
        stationaryObj& operator = (stationaryObj&&) noexcept = default;
    };

   class movingObj : public obstacle {

   public:
      explicit movingObj(geo_ring poly) :obstacle(poly) {}
      virtual ~movingObj() = default;

      movingObj(const movingObj&) = default;
      movingObj(movingObj&&) noexcept = default;

      movingObj& operator = (const movingObj&) = default;
      movingObj& operator = (movingObj&&) noexcept = default;
   };



   struct Constraints
   {
      vector<stationaryObj> static_obstacles;
      vector<movingObj> moving_obstacles;
   };
}

#endif