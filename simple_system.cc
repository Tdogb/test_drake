#include "drake/examples/test_drake/simple_system.h"

namespace drake {
namespace examples {
namespace test_drake {

    template<typename T>
    simple_system<T>::simple_system(/* args */)
    {
        this->DeclareVectorInputPort("inputP", systems::BasicVector<T>(1));
        this->DeclareDiscreteState(1);
        this->DeclarePeriodicDiscreteUpdateEvent(kPeriod, kOffset, &simple_system::update);
        this->DeclareVectorOutputPort("outputP", systems::BasicVector<T>(1), &simple_system::setStateOutput);
    }

    template<typename T>
    simple_system<T>::~simple_system()
    {
    }
    
    /*
    This is in reference to the output port
    */
    template<typename T>
    void simple_system<T>::setStateOutput(const systems::Context<T>& context, systems::BasicVector<T>* output) const {
        const systems::BasicVector<T>& currentVector = context.get_discrete_state_vector();
        //const systems::BasicVector<T> newVector = currentVector * 2;
        output->set_value(currentVector.CopyToVector());
    }

    template<typename T>
    void simple_system<T>::update(const systems::Context<T>& context, systems::DiscreteValues<T>* values) const {
        //const systems::BasicVector<T>& currentVector = context.get_discrete_state();
        const double discrete_state = context.get_discrete_state()[0];
        //systems::BasicVector<T> newVector;
        //newVector.CopyToVector() = systems::BasicVector<T>(currentVector.CopyToVector() * 2);
        //(*values).get_mutable_vector(1) = newVector.CopyToVector();
        (*values)[0] = discrete_state + 2;
    }

    template class simple_system<double>; //Forces the instantiation of the template with a double
}
}
}