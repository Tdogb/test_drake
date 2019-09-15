// #include "drake/systems/framework/system_base.h"
//#include "drake/systems/framework/system.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace test_drake {

    template<typename T>
    class simple_system : public systems::LeafSystem<T>
    {
    private:
        void update(const systems::Context<T>& context, systems::DiscreteValues<T>* values) const;
        void setStateOutput(const systems::Context<T>& context, systems::BasicVector<T>* output) const;
    public:
        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(simple_system); 
        simple_system();
        static constexpr double kPeriod = 1 / 50; //10hz
        static constexpr double kOffset = 0;
        ~simple_system();
    };

}
}
}
