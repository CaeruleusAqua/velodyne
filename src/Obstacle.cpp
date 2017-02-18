#include "Obstacle.h"

Obstacle::Obstacle(double x, double y) {
    x<<x,y,0,0;
    I.setIdentity();
    P<<1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1;
}
