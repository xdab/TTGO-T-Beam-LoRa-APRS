#ifndef SMARTBEACONING_H
#define SMARTBEACONING_H

#include <stdint.h>

#define ANGLE_AVGS 3  // number of course averages to take
#define SPEED_AVGS 5 // number of speed averages to take

class SmartBeaconing
{
private:
    uint32_t sb_min_interval = 60000;
    uint32_t sb_max_interval = 360000;
    float sb_min_speed = 0;
    float sb_max_speed = 30;
    float sb_angle = 30;

    float average_speed[SPEED_AVGS] = {0};
    float average_speed_final = 0;
    float old_course = 0;
    float new_course = 0;
    int point_avg_speed = 0;
    int point_avg_course = 0;
    float average_course[ANGLE_AVGS] = {0};
    float avg_c_y;
    float avg_c_x;

    uint32_t last_tx = 0;
    uint32_t next_tx = 60000L;

public:
    uint64_t update_pos(float latitute, float longitude, float speed_kmph, float course);
    void update_tx();

    void set_min_interval(uint32_t min_interval);
    uint32_t get_min_interval();
    void set_max_interval(uint32_t max_interval);
    uint32_t get_max_interval();
    void set_min_speed(uint32_t min_speed);
    uint32_t get_min_speed();
    void set_max_speed(uint32_t max_speed);
    uint32_t get_max_speed();
    void set_angle(float angle);
    float get_angle();
};

#endif // SMARTBEACONING_H
