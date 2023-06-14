#include "controller.h"

class target_follow: public Controller{
    public:
        target_follow(std::vector<Eigen::Vector3f> waypoints);
        ~target_follow(){};

        void set_waypoints(std::vector<Eigen::Vector3f> waypoints){wp_list = waypoints;};
    private:
        Eigen::Vector3f current_wp; // current target
        Eigen::Vector3f centroid; // current centroid
        std::vector<Eigen::Vector3f> wp_list; // list of waypoints to follow. cyclical behaviour by default

        float target_follow_gain;
        float wp_radius;
        float dist_to_wp;
        float dist_to_wp_max;
        bool start;
        uint wp_counter; 

        bool current_wp_reached();
        virtual void animation(const uint16_t ID);
        virtual Eigen::Vector3f get_velocity_cmd(const uint16_t ID);

};