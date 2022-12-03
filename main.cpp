#include <stdio.h>
#include "mtxmath.h"

template <size_t NUM_STATE_VARIABLES, size_t NUM_MEASUREMENT_VARIABLES>
class KalmanFilter
{
public:
    KalmanFilter()
    {

    }

    void Predict()
    {
        // Update State and covariance based on simulation
        m_state = m_stateUpdate * m_state;
        m_covariance = m_stateUpdate * m_covariance * Transpose(m_covariance);
    }

    void Update()
    {

    }

    Vec<NUM_STATE_VARIABLES> m_state = {};
    Mtx<NUM_STATE_VARIABLES, NUM_STATE_VARIABLES> m_covariance = {};
    Mtx<NUM_STATE_VARIABLES, NUM_STATE_VARIABLES> m_stateUpdate = {};
};

int main(int argc, char** argv)
{
    /*
    Vec<2> testV = { 1.0f, 0.2f };
    Mtx<2, 2> testM = {
        0.0f, 1.0f,
        1.0f, 0.0f,
    };
    auto result = testV * testM;
    int ijkl = 0;
    */

    // 
    {
        static const float c_deltaT = 1.0f / 60.0f;

        KalmanFilter<2, 1> filter;
        filter.m_state = { 0.0f, 0.0f };
        filter.m_covariance = {
            1.0f, 1.0f,
            1.0f, 1.0f
        };
        filter.m_stateUpdate = {
            1.0f, c_deltaT,
            0.0f, 1.0f
        };

        filter.Predict();
        int ijkl = 0;
    }

    return 0;
}

/*
TODO:
- rename functions? predict to simulate? update may be ok but is what adapts to the simulation data and the measurements
- output the details of each step to be able to graph it etc.

*/