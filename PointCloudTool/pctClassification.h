#pragma once
#include <string>
namespace pct
{
    class Classification
    {
    public:
        Classification();
        ~Classification();

        void compute_features(std::size_t nb_scales); //��������
        void train(std::string traindir, unsigned int nb_trials);
    };


}
