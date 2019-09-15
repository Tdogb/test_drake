#include "drake/examples/test_drake/simple_system.h"

namespace drake {
namespace examples {
namespace test_drake {

    template<typename T>
    simple_system<T>::simple_system() {
        this->DeclareVectorInputPort("inputPort", systems::BasicVector<T>(1));
        this->DeclareDiscreteState(1); //1D discrete state
        this->DeclarePeriodicDiscreteUpdateEvent(0.02, 0, &simple_system::update);
        this->DeclareVectorOutputPort("outputPort", systems::BasicVector<T>(1), &simple_system::setStateOutput);
    }

    template<typename T>
    simple_system<T>::~simple_system() {}
    
    /*
    This is used for the output port (which will be read by the Logger)
    */
    template<typename T>
    void simple_system<T>::setStateOutput(const systems::Context<T>& context, systems::BasicVector<T>* output) const {
        const systems::BasicVector<T>& currentVector = context.get_discrete_state_vector();
        output->set_value(currentVector.CopyToVector());
    }

    /*
    This will set the state vector at time = t
    @param context: The state vector at time t = t - 1
    @param values: pointer to where the 
    */
    template<typename T>
    void simple_system<T>::update(const systems::Context<T>& context, systems::DiscreteValues<T>* values) const {
        const double discrete_state = context.get_discrete_state()[0];
        (*values)[0] = discrete_state * 2;
    }

    template class simple_system<double>; //Forces the instantiation of the template with a double
}
}
}