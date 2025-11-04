#define EIGEN_USE_BLAS
#define EIGEN_USE_LAPACK
#include "model_SFDI.hpp"
int main(void) {
    SFDI::model_SFDI model("reference_670", "sample_670","ROfRhoAndTime");
    SFDI::SFDI_Model mc_model = model.mc_model_for_SFDI((SFDI::Optical_prop() << 0.0059).finished(),
                         (SFDI::Optical_prop() << 0.9748).finished(),
                         (SFDI::Optical_prop() << 1.37).finished(),
                         (SFDI::Freq() << 0, 0.2).finished());
    
    std::cout<<mc_model<<std::endl;
    return 0;
}