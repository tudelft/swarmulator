#include "controller.h"
#include "multiranger.h"
#include "draw.h"

class wall_avoidance: public Controller{
    private:
        MultiRanger multi_ranger;

    public:
        wall_avoidance();
        ~wall_avoidance(){};
        virtual Eigen::Vector3f get_velocity_cmd(const uint16_t ID);
        virtual void animation(const uint16_t ID);
};