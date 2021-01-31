#include <dynamic_reconfigure/server.h>
#include <nodelet/nodelet.h>
#include <rr_common/PathPlannerConfig.h>

namespace planner_dyn_reconf_ns {

    void dynamic_callback(rr_common::PathPlannerConfig&, uint32_t);
    class planner_node : public nodelet::Nodelet {
        private:
            virtual void onInit();
    };
}