#include <cstdlib>
#include <limits>
#include <memory>

#include <gflags/gflags.h>
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/examples/test_drake/simple_system.h"
#include "drake/examples/test_drake/send_to_visualizer.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/examples/test_drake/utilities.h"

//using drake::systems::DiagramBuilder;
//using drake::systems::Diagram;

namespace drake {
namespace examples {
namespace test_drake {
namespace {

class run_sim : public systems::Diagram<double>
{
private:

public:
    std::unique_ptr<RigidBodyTree<double>> tree = std::make_unique<RigidBodyTree<double>>();
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(run_sim);
    run_sim(/* args */);
    std::unique_ptr<systems::Context<double>> CreateContext(double value) const; //Not sure why this has to be const
    systems::SignalLogger<double>* logger;
    ~run_sim();
};

run_sim::run_sim() {
    systems::DiagramBuilder<double> diagrambuilder;
    parsers::sdf::AddModelInstancesFromSdfFileToWorld("/Users/tdogb/drake/examples/test_drake/models/particle.sdf", multibody::joints::kRollPitchYaw, tree.get());
    tree->compile();
    std::cout << tree->get_num_positions() << "  " << tree->get_num_velocities() << std::endl;
    /*
    This will add the simple_system to the diagram
    */
    std::cout << "before covert_sys" << std::endl;
   //auto convert_sys = diagrambuilder.AddSystem<send_to_visualizer<double>>();
    MatrixX<double> translatingMatrix(6,1);
    translatingMatrix.setZero();
    translatingMatrix(0,0) = 1.0;
    auto convert_sys = diagrambuilder.AddSystem(MakeDegenerateEulerJoint(translatingMatrix));
    std::cout << "created convert_sys" << std::endl;
    auto simple_sys = diagrambuilder.AddSystem<simple_system<double>>();
    auto lcm = diagrambuilder.AddSystem<systems::lcm::LcmInterfaceSystem>();
    auto visualizer = diagrambuilder.AddSystem<systems::DrakeVisualizer>(*tree, lcm);
    visualizer->get_name();
    simple_sys->get_name();
    convert_sys->get_name();
    //auto joint = diagrambuilder.AddSystem(MakeDegenerate);
    /*
    Set up logging (in order to print out the result after the simulation)
    */

    diagrambuilder.Connect(*simple_sys, *convert_sys);
    diagrambuilder.Connect(*convert_sys, *visualizer);
    logger = LogOutput(simple_sys->get_output_port(0), &diagrambuilder);
    diagrambuilder.BuildInto(this);
}

run_sim::~run_sim() {}

/*
Sets the initial condition
*/
std::unique_ptr<systems::Context<double>> run_sim::CreateContext(double value) const {
    auto context = this->AllocateContext();
    std::cout << context->to_string() << std::endl;
    systems::VectorBase<double>& cstate = context->get_mutable_continuous_state_vector();
    cstate.SetAtIndex(0, 0.0); //Velocity
    cstate.SetAtIndex(1, value); //Acceleration
    return context;
}

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    auto system = std::make_unique<run_sim>();
    std::cout << "finished" << std::endl;
    /*
    This actually runs the simulation
    */
    systems::Simulator<double> sim(*system, system->CreateContext(1.5));
    std::cout << "sim created" << std::endl;
    sim.Initialize();
    std::cout << "sim init finished" << std::endl;
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