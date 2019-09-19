#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace test_drake {

template<typename T>
class send_to_visualizer : public systems::LeafSystem<T>
{
private:
    /* data */
public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(send_to_visualizer);
    send_to_visualizer(/* args */);
    void setStateOutput(const systems::Context<T>& context, systems::BasicVector<T>* output) const;
    void update(const systems::Context<T>& context, systems::DiscreteValues<T>* values) const;
    ~send_to_visualizer();
};

}
}
}



