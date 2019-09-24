#include "drake/examples/test_drake/simple_system.h"

namespace drake {
namespace examples {
namespace test_drake {
    #define NUM_STATES 12 //Need to also change DeclareContinuousState

    template<typename T>
    simple_system<T>::simple_system(geometry::SourceId sid, geometry::SceneGraph<T>* scene_graph) {
        /* Query is asking for the geometry */
        this->DeclareAbstractInputPort(systems::kUseDefaultName, Value<geometry::QueryObject<T>>());
        this->DeclareContinuousState(6,6,0); //continuous state
        this->DeclareVectorOutputPort("outputPort", systems::BasicVector<T>(NUM_STATES), &simple_system::setStateOutput);

        auto simple_system_frame = scene_graph->RegisterFrame(sid, geometry::GeometryFrame("simple_system_geo_frame"));
        auto simple_system_geo = scene_graph->RegisterGeometry(sid, simple_system_frame);
        this->DeclareAbstractOutputPort();
    }

    template<typename T>
    simple_system<T>::~simple_system() {}
    
    /*
    This is used for the output port (which will be read by the Logger)
    */
    template<typename T>
    void simple_system<T>::setStateOutput(const systems::Context<T>& context, systems::BasicVector<T>* output) const {
        const systems::VectorBase<T>& currentVector = context.get_continuous_state_vector();
        //std::cout << currentVector.CopyToVector() << std::endl;
        output->set_value(currentVector.CopyToVector());
    }

    template<typename T>
    void simple_system<T>::DoCalcTimeDerivatives(const systems::Context<T>& context, systems::ContinuousState<T>* derrivatives) const {
        const systems::VectorBase<T>& state_vector = context.get_continuous_state_vector();
        systems::VectorBase<T>& derivsVelocity = derrivatives->get_mutable_generalized_velocity();
        systems::VectorBase<T>& derivsPosition = derrivatives->get_mutable_generalized_position();
        const auto& query_object = 
        
        derivsPosition.SetAtIndex(0, state_vector.GetAtIndex(1)); //Velocity
        derivsVelocity.SetAtIndex(1, 9.8); //Acceleration
    }

    template class simple_system<double>; //Forces the instantiation of the template with a double
}
}
}