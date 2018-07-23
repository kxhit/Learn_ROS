/// Obstacle Avoidance Algorithm Class
///
/// (C)2015-2017 CSC-105 Robotics Lab, ZJU
/// Author: Highlight Clark
/// e-mail: highlightz@163.com
///
/// Modified: Dec. 6, 2016

#ifndef OBS_AVOIDER_H
#define OBS_AVOIDER_H

#include <iostream>
#include <vector>

class obs_avoider
{
public:
    /**
     * Laser data conversion,
     * provided that only front -90 deg - 90 deg range considered
     */
    static int step2index( int step ){ return step + 360; }
    static int deg2step( double deg ){ return static_cast< int >( 4 * deg ); }
    static double index2rad(int index,int size=720, double angle=M_PI){ return ( index * angle / size - angle / 2. ); }
    static double deg2rad( double deg ){ return deg * M_PI / 180.; }

    /**
     * @param distance scanning: rays of range [-90 deg, 90 deg], with x axis pointing to front
     * @param interest_oa_radius: default obstalce avoidance radius is 1.5 meters, namely 1500 mm
     * @param strategy: 0 stands for virtual force method, 1 for evading method
     */
    static double laser_obstalce_avoid( const std::vector< long >& distance, const long& interest_oa_radius = 1500, int strategy = 0 );

};

#endif  // OBS_AVOIDER_H
