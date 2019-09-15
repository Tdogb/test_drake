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
    std::unique_ptr<systems::Context<double>> CreateContext(double value) const; //Not sure why this has to be const
    systems::SignalLogger<double>* logger;
    ~run_sim();
};

run_sim::run_sim() {
    systems::DiagramBuilder<double> diagrambuilder;
    /*
    This will add the simple_system to the diagram
    */
    auto simple_sys = diagrambuilder.AddSystem<simple_system<double>>();
    /*
    Set up logging (in order to print out the result after the simulation)
    */
    logger = LogOutput(simple_sys->get_output_port(0), &diagrambuilder);
    diagrambuilder.BuildInto(this); 
}

run_sim::~run_sim() {}

/*
Sets the initial condition
*/
std::unique_ptr<systems::Context<double>> run_sim::CreateContext(double value) const {
    auto context = this->AllocateContext();
    systems::VectorBase<double>& dstate = context->get_mutable_discrete_state_vector();
    dstate.SetAtIndex(0, value);
    return context;
}

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    auto system = std::make_unique<run_sim>();

    /*
    This actually runs the simulation
    */
    systems::Simulator<double> sim(*system, system->CreateContext(1.5));
    sim.Initialize();
    sim.AdvanceTo(1);

    /*
    Prints out the logs
    */
    for(int i = 0; i < system->logger->sample_times().size(); i++) {
        const double logOut = system->logger->sample_times()[i];
        std::cout << system->logger->data()(0,i) << "  ( " << logOut << " )  " << std::endl;
    }
    return 0;
}
}   
}
}
}

/*
This just calls the main function inside of the namespaces
because the main inside of the namespaces will not be recognized by the compiler
*/
int main(int argc, char **argv) {
    return drake::examples::test_drake::main(argc,argv);
}