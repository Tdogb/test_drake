#include <cstdlib>
#include <limits>
#include <memory>

#include <gflags/gflags.h>
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/examples/test_drake/simple_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/analysis/simulator.h"

//using drake::systems::DiagramBuilder;
//using drake::systems::Diagram;

namespace drake {
namespace examples {
namespace test_drake {
namespace {

class run_sim : public systems::Diagram<double>
{
private:
    /* data */
public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(run_sim);
    run_sim(/* args */);
    std::unique_ptr<systems::Context<double>> CreateContext(double value) const;
    ~run_sim();
};

run_sim::run_sim(/* args */)
{
    systems::DiagramBuilder<double> diagrambuilder;
    const double val = 1.5; //Value to be multiplied
    auto valueSource = diagrambuilder.AddSystem<systems::ConstantVectorSource<double>>(val);
    auto simple_sys = diagrambuilder.template AddSystem<simple_system<double>>(); //How is this legal syntax?
    diagrambuilder.Connect(*valueSource, *simple_sys);
    auto logger = LogOutput(simple_sys->get_output_port(0), &diagrambuilder);
    //logger->set_publish_period(simple_system<double>::kPeriod);
    auto diagram = diagrambuilder.Build();

    systems::Simulator<double> sim(*diagram);
    sim.AdvanceTo(10 * simple_system<double>::kPeriod);

    //Write out the logs
    for(int i = 0; i < logger->sample_times().size(); i++) {
        const double logOut = logger->sample_times()[i];
        std::cout << logger->data()(0,i) << "  ( " << logOut << " )  " << std::endl;
    }
    //auto lcm = diagrambuilder.AddSystem<systems::lcm::LcmInterfaceSystem>();
}

run_sim::~run_sim()
{
}

/*
Context is where the value is stored
This would usually be the position or velocity of a system
*/
std::unique_ptr<systems::Context<double>> run_sim::CreateContext(double value) const {
    auto context = this->AllocateContext();
    systems::VectorBase<double>& dstate = context->get_mutable_discrete_state_vector();
    dstate.SetAtIndex(0, value);
    return context;
}

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    run_sim a;
    //std::make_unique<run_sim>;
    //std::make_unique<simple_system<double>> ex_system;
    //auto lcm = diagrambuilder.AddSystem<systems::lcm::LcmInterfaceSystem>();
    //auto simple_sys = diagrambuilder.AddSystem(std::make_unique<drake::examples::test_drake::simple_system<double>>());
    //diagrambuilder.AddSystem<simple_system<double>>():
    return 0;
}
}   
}
}
}
int main(int argc, char **argv) {
    return drake::examples::test_drake::main(argc,argv);
}