#include "drake/examples/test_drake/send_to_visualizer.h"

namespace drake {
namespace examples {
namespace test_drake {

template<typename T>
send_to_visualizer<T>::send_to_visualizer(/* args */)
{
    this->DeclareVectorInputPort("visInput", systems::BasicVector<T>(2));
    this->DeclareDiscreteState(6);
    this->DeclarePeriodicDiscreteUpdateEvent(0.02, 0, &send_to_visualizer::update);
    this->DeclareVectorOutputPort("visOutput", systems::BasicVector<T>(12), &send_to_visualizer::setStateOutput);
}

template<typename T>
send_to_visualizer<T>::~send_to_visualizer()
{
}

template<typename T>
void send_to_visualizer<T>::update(const systems::Context<T>& context, systems::DiscreteValues<T>* values) const {

}

template<typename T>
void send_to_visualizer<T>::setStateOutput(const systems::Context<T>& context, systems::BasicVector<T>* output) const {
    const systems::BasicVector<double>* input = this->EvalVectorInput(context,0);
    input->size();
    MatrixX<double> m(12,1);
    m.topLeftCorner(6,1) = input->CopyToVector();
    m.bottomLeftCorner(6,1) = input->CopyToVector();
    output->set_value(m);
}

template class send_to_visualizer<double>;

}
}
}