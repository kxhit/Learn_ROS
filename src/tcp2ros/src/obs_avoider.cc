#include <cmath>
#include <cstdlib>

#include "obs_avoider.h"

double obs_avoider::laser_obstalce_avoid( const std::vector< long >& distance, const long& interest_oa_radius, int strategy )
{
    double const EPS = 0.000001;

    // Check input laser data
    if ( distance.empty( ))// || 721 !=distance.size( ) )
    {
        return 0.0;
    }

    // Virtual force method
    if ( 0 == strategy )
    {
        double sum_force_x = 0.0;
        double sum_force_y = 0.0;

        // person following
#if 0
        for ( size_t index = 150; index < distance.size( ) - 150; ++index )
        {
            double radian = index2rad( index, distance.size()-1,M_PI/180.0*190.0);
            long limited_distance = ( distance[index] > interest_oa_radius ? interest_oa_radius : distance[index] );

            // Suppose that, F = 1/d^2
            // accumulate all the forces
            sum_force_x += 1.0 / ( limited_distance * limited_distance + 1 ) * cos( radian );
            sum_force_y += 1.0 / ( limited_distance * limited_distance + 1 ) * sin( radian );
        }
#endif

#if 1
        for ( size_t index = 0; index < distance.size( ); ++index )
        {
            double radian = index2rad( index, distance.size()-1,M_PI/180.0*190.0);
            long limited_distance = ( distance[index] > interest_oa_radius ? interest_oa_radius : distance[index] );

            // Suppose that, F = d^2
            // accumulate all the forces
            sum_force_x += limited_distance * limited_distance * cos( radian );
            sum_force_y += limited_distance * limited_distance * sin( radian );
        }
#endif

        return ( -atan2( sum_force_y, sum_force_x ) );
    }
    else
    /*
     * Evading method
     * Go straight forward, until when within the interest obstacle avoidance radius,
     * there exist obstalce point clouds, who generate repulse force to change
     * the car direction.
     */
    {
        double sum_force_x = 0.0;
        double sum_force_y = 0.0;
        //从右到左
        for ( size_t index = 0; index < distance.size( ); ++index )
        {
            if ( distance[index] < interest_oa_radius )
            {
                double radian = index2rad( index, distance.size()-1,M_PI/180.0*190.0);
                //std::cout<<radian<<" ";

		if(radian>=-M_PI/2.0 &&radian<=M_PI/2.0 && distance[index]!=0)
		{
			std::cout<<distance[index]<<" "<<radian<<std::endl;
		        // Suppose that, F = 1 / d^2
		        sum_force_x += 1. / ( distance[index] * distance[index] + EPS ) * cos( radian );
		        sum_force_y += 1. / ( distance[index] * distance[index] + EPS ) * sin( radian );
		}
#if 0
                // Suppose that, F = 1 / d
                sum_force_x += 1. / ( distance[index] + eps ) * cos( radian );
                sum_force_y += 1. / ( distance[index] + eps ) * sin( radian );
#endif
            }
        }

        if ( fabs( sum_force_x ) < EPS && fabs( sum_force_y ) < EPS )
        {
            return 0.0;
        }

        double x_norm = sum_force_x / sqrt( sum_force_x * sum_force_x + sum_force_y * sum_force_y );
        double y_norm = sum_force_y / sqrt( sum_force_x * sum_force_x + sum_force_y * sum_force_y );

        //这里改成<0,向左转，因为右边的为负的，负的累积的越多，说明右边障碍越多，则越往左转
        if ( y_norm < 0 )  // Control makes car turn left
        {
            return ( - atan2( y_norm, x_norm ) + M_PI / 2 );
        }
        else  // Control makes car turn right
        {
            return ( - M_PI / 2 - atan2( y_norm, x_norm ) );
        }
    }
}
