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
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"

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

    auto plnt = multibody::AddMultibodyPlantSceneGraph(&diagrambuilder, std::make_unique<multibody::MultibodyPlant<double>>());
    multibody::MultibodyPlant<double>& plant = plnt.plant;

    //multibody::Parser(&plant).AddModelFromFile("/Users/tdogb/drake/examples/test_drake/models/particle.sdf");
    plant.RegisterVisualGeometry(plant.world_body(), math::RigidTransformd(), geometry::Box(1,1,1), "Box");
    plant.RegisterVisualGeometry(plant.world_body(), math::RigidTransformd(), geometry::HalfSpace(), "Floor");
    const multibody::CoulombFriction<double> friction(1.0,1.0);
    plant.RegisterCollisionGeometry(plant.world_body(), math::RigidTransformd(), geometry::Box(1,1,1), "BoxCollision", friction);
    plant.RegisterCollisionGeometry(plant.world_body(), math::RigidTransformd(), geometry::HalfSpace(), "GroundCollision", friction);
    plant.Finalize();
    plant.set_penetration_allowance(0.001);
    plant.set_stiction_tolerance(0.001);
    
    //parsers::sdf::AddModelInstancesFromSdfFileToWorld("/Users/tdogb/drake/examples/test_drake/models/particle.sdf", multibody::joints::kRollPitchYaw, tree.get());
    //tree->compile();
    auto lcm = diagrambuilder.AddSystem<systems::lcm::LcmInterfaceSystem>();
    const geometry::SourceId sid = plnt.scene_graph.RegisterSource("Pose0");
    auto simple_sys = diagrambuilder.AddSystem<simple_system<double>>(sid, &plnt.scene_graph);
    diagrambuilder.Connect(plnt.scene_graph.get_query_output_port(), simple_sys->get_input_port());
    diagrambuilder.Connect(simple_sys->get_output_port(), plnt.scene_graph.get_source_pose_port(sid));
    geometry::ConnectDrakeVisualizer(&diagrambuilder, plnt.scene_graph, lcm);
    multibody::ConnectContactResultsToDrakeVisualizer(&diagrambuilder, plant, lcm);
    //auto visualizer = diagrambuilder.AddSystem<systems::DrakeVisualizer>(*tree, lcm);
    //visualizer->get_name();
    simple_sys->get_name();

    //diagrambuilder.Connect(simple_sys->get_, plnt.scene_graph.get_source_pose_port());
    logger = LogOutput(simple_sys->get_output_port(0), &diagrambuilder);
    diagrambuilder.BuildInto(this);
}

run_sim::~run_sim() {}

/*
Sets the initial condition
*/
std::unique_ptr<systems::Context<double>> run_sim::CreateContext(double value) const {
    auto context = this->AllocateContext();
    systems::VectorBase<double>& cstate = context->get_mutable_continuous_state_vector();
    //std::cout << "cstate size: " << cstate.size() << std::endl;
    cstate.SetAtIndex(0, 0.0); //Velocity
    cstate.SetAtIndex(1, value); //Acceleration
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
    sim.set_target_realtime_rate(1);
    sim.AdvanceTo(10);

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