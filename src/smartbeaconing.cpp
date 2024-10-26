#include "smartbeaconing.h"
#include <math.h>
#include <Arduino.h>

uint64_t SmartBeaconing::update_pos(float latitute, float longitude, float speed_kmph, float course)
{
    average_speed[point_avg_speed] = speed_kmph;
    ++point_avg_speed;
    if (point_avg_speed >= SPEED_AVGS)
        point_avg_speed = 0;

    average_speed_final = (average_speed[0] + average_speed[1] + average_speed[2] + average_speed[3] + average_speed[4]) / 5;
    average_course[point_avg_course] = course;
    ++point_avg_course;

    next_tx = (sb_max_interval - sb_min_interval) / (sb_max_speed - sb_min_speed) * (sb_max_speed - average_speed_final) + sb_min_interval;
    if (next_tx < sb_min_interval)
        next_tx = sb_min_interval;
    if (next_tx > sb_max_interval)
        next_tx = sb_max_interval;

    if (point_avg_course >= ANGLE_AVGS)
    {
        point_avg_course = 0;
        avg_c_y = 0;
        avg_c_x = 0;
        for (int i = 0; i < ANGLE_AVGS; i++)
        {
            avg_c_y += sin(average_course[i] / 180 * M_PI);
            avg_c_x += cos(average_course[i] / 180 * M_PI);
        }
        new_course = atan2f(avg_c_y, avg_c_x) * 180 / M_PI;
        if (new_course < 0)
        {
            new_course = 360 + new_course;
        }
        if ((old_course < sb_angle) && (new_course > (360 - sb_angle)))
        {
            if (fabs(new_course - old_course - 360) >= sb_angle)
            {
                next_tx = 1; // give one second for turn to finish and then TX
            }
        }
        else
        {
            if ((old_course > (360 - sb_angle)) && (new_course < sb_angle))
            {
                if (fabs(new_course - old_course + 360) >= sb_angle)
                {
                    next_tx = 1;
                }
            }
            else
            {
                if (fabs(new_course - old_course) >= sb_angle)
                {
                    next_tx = 1;
                }
            }
        }
        old_course = new_course;
    }

    if ((millis() < sb_max_interval) && (last_tx == 0))
    {
        next_tx = 0;
    }

    return last_tx + next_tx;
}

void SmartBeaconing::update_tx()
{
    last_tx = millis();
}

void SmartBeaconing::set_min_interval(uint32_t min_interval)
{
    sb_min_interval = min_interval;
}

uint32_t SmartBeaconing::get_min_interval()
{
    return sb_min_interval;
}

void SmartBeaconing::set_max_interval(uint32_t max_interval)
{
    sb_max_interval = max_interval;
}

uint32_t SmartBeaconing::get_max_interval()
{
    return sb_max_interval;
}

void SmartBeaconing::set_min_speed(uint32_t min_speed)
{
    sb_min_speed = min_speed;
}

uint32_t SmartBeaconing::get_min_speed()
{
    return sb_min_speed;
}

void SmartBeaconing::set_max_speed(uint32_t max_speed)
{
    sb_max_speed = max_speed;
}

uint32_t SmartBeaconing::get_max_speed()
{
    return sb_max_speed;
}

void SmartBeaconing::set_angle(float angle)
{
    sb_angle = angle;
}

float SmartBeaconing::get_angle()
{
    return sb_angle;
}
