#include <stdio.h>
#include "mtxmath.h"

template <size_t NUM_STATE_VARIABLES, size_t NUM_CONTROL_VARIABLES, size_t NUM_MEASUREMENT_VARIABLES>
class KalmanFilter
{
public:
    KalmanFilter()
    {

    }

    void Predict()
    {
        // TODO: need process noise, but that is a matrix so... ???
        m_stateVector = m_stateUpdateMatrix * m_stateVector + m_controlMatrix * m_controlVector;// +m_processNoiseMatrix;
        // TODO: why no process noise? is that only on first iteration?
        m_stateUncertainty = m_stateUpdateMatrix * m_stateUncertainty * Transpose(m_stateUpdateMatrix);// +m_processNoiseMatrix;
    }

    void Iterate(const Vec<NUM_MEASUREMENT_VARIABLES>& measurement)
    {

        // H is N_z x N_x = NUM_MEASUREMENT_VARIABLES x NUM_STATE_VARIABLES
        // P is N_x x N_x = NUM_STATE_VARIABLES x NUM_STATE_VARIABLES = Estimate uncertainty

        // m_observationMatrix is NUM_MEASUREMENT_VARIABLES x NUM_STATE_VARIABLES
        // m_stateVector is NUM_STATE_VARIABLES x 1;
        // m_measurementUncertainty is NUM_MEASUREMENT_VARIABLES x NUM_MEASUREMENT_VARIABLES

        // m_observationMatrix is 2 x 6
        // m_stateVector is 6 x 1;
        // m_measurementUncertainty is 2x2

        // m_observationMatrix * m_stateVector is 2 x 1;

        // ----- 1. Measure -----

        // ----- 2. Update -----

        // Calculate kalman gain
        Mtx<NUM_STATE_VARIABLES, NUM_MEASUREMENT_VARIABLES> kalmanGain = m_stateUncertainty * Transpose(m_observationMatrix) *
            Inverse(m_observationMatrix * m_stateUncertainty * Transpose(m_observationMatrix) + m_measurementUncertainty);

        // estimate current state
        m_stateVector = m_stateVector + kalmanGain * (measurement - m_observationMatrix * m_stateVector);


        // K * H
        // K is NUM_STATE_VARIABLES x NUM_MEASUREMENT_VARIABLES
        // H is NUM_MEASUREMENT_VARIABLES x NUM_STATE_VARIABLES

        // update estimate uncertainty
        const Mtx<NUM_STATE_VARIABLES, NUM_STATE_VARIABLES> I = Identity<NUM_STATE_VARIABLES>();
        m_stateUncertainty = (I - kalmanGain * m_observationMatrix) * m_stateUncertainty * Transpose(I - kalmanGain * m_observationMatrix) +
            kalmanGain * m_measurementUncertainty * Transpose(kalmanGain);

        // ----- 3. Predict -----

        m_stateVector = m_stateUpdateMatrix * m_stateVector + m_controlMatrix * m_controlVector;

        m_stateUncertainty = m_stateUpdateMatrix * m_stateUncertainty * Transpose(m_stateUpdateMatrix);// +m_processNoiseMatrix;

        // TODO: seems like the process noise matrix is never actually added in?!

        int ijkl = 0;
    }

    Vec<NUM_STATE_VARIABLES> m_stateVector = {};
    Mtx<NUM_STATE_VARIABLES, NUM_STATE_VARIABLES> m_stateUpdateMatrix = {};
    Mtx<NUM_STATE_VARIABLES, NUM_STATE_VARIABLES> m_processNoiseMatrix = {};

    Mtx<NUM_STATE_VARIABLES, NUM_STATE_VARIABLES> m_stateUncertainty = {};
    Mtx<NUM_MEASUREMENT_VARIABLES, NUM_MEASUREMENT_VARIABLES> m_measurementUncertainty = {};

    Vec<NUM_CONTROL_VARIABLES> m_controlVector = {};
    Mtx<NUM_STATE_VARIABLES, NUM_CONTROL_VARIABLES> m_controlMatrix = {};

    // To match measurement values with state vector values
    Mtx<NUM_MEASUREMENT_VARIABLES, NUM_STATE_VARIABLES> m_observationMatrix = {};
};

int main(int argc, char** argv)
{
    /*
    Vec<2> testV = { 1.0f, 0.2f };
    Mtx<2, 3> testM = {
        0.0f, 1.0f, 0.5f,
        1.0f, 0.0f, 0.5f,
    };
    auto result = testV * testM;
    int ijkl = 0;
    */
 
    // Example 9
    // https://www.kalmanfilter.net/multiExamples.html
    {
        static const float c_deltaT = 1.0f;
        static const float c_randomAccelerationStdDev = 0.2f; // Meters / Seconds^2
        static const float c_measurementErrorStdDevX = 3.0f; // Meters
        static const float c_measurementErrorStdDevY = 3.0f; // Meters

        KalmanFilter<6, 0, 2> filter;

        filter.m_stateUpdateMatrix = {
            1.0f, c_deltaT, 0.5f * c_deltaT * c_deltaT, 0.0f,     0.0f,                       0.0f,
            0.0f,     1.0f,                   c_deltaT, 0.0f,     0.0f,                       0.0f,
            0.0f,     0.0f,                       1.0f, 0.0f,     0.0f,                       0.0f,
            0.0f,     0.0f,                       0.0f, 1.0f, c_deltaT, 0.5f * c_deltaT * c_deltaT,
            0.0f,     0.0f,                       0.0f, 0.0f,     1.0f,                   c_deltaT,
            0.0f,     0.0f,                       0.0f, 0.0f,     0.0f,                       1.0f,
        };

        // TODO: how was this derived?
        filter.m_processNoiseMatrix = {
            powf(c_deltaT, 4.0f) / 4.0f, powf(c_deltaT, 3.0f) / 2.0f, powf(c_deltaT, 2.0f) / 2.0f,                        0.0f,                        0.0f,                        0.0f,
            powf(c_deltaT, 3.0f) / 2.0f,         c_deltaT * c_deltaT,                    c_deltaT,                        0.0f,                        0.0f,                        0.0f,
            powf(c_deltaT, 2.0f) / 2.0f,                    c_deltaT,                        1.0f,                        0.0f,                        0.0f,                        0.0f,
                                   0.0f,                        0.0f,                        0.0f, powf(c_deltaT, 4.0f) / 4.0f, powf(c_deltaT, 3.0f) / 2.0f, powf(c_deltaT, 2.0f) / 2.0f,
                                   0.0f,                        0.0f,                        0.0f, powf(c_deltaT, 3.0f) / 2.0f,          c_deltaT * c_deltaT,                    c_deltaT,
                                   0.0f,                        0.0f,                        0.0f, powf(c_deltaT, 2.0f) / 2.0f,                    c_deltaT,                        1.0f,
        };
        filter.m_processNoiseMatrix = filter.m_processNoiseMatrix * (c_randomAccelerationStdDev * c_randomAccelerationStdDev);

        filter.m_measurementUncertainty = {
            c_measurementErrorStdDevX * c_measurementErrorStdDevX,                                                  0.0f,
                                                             0.0f, c_measurementErrorStdDevY * c_measurementErrorStdDevY,
        };

        filter.m_observationMatrix = {
            1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
        };

        // Initial state
        filter.m_stateVector = {};
        filter.m_stateUncertainty = {};

        filter.m_stateUncertainty[0][0] =
            filter.m_stateUncertainty[1][1] =
            filter.m_stateUncertainty[2][2] =
            filter.m_stateUncertainty[3][3] =
            filter.m_stateUncertainty[4][4] = 
            filter.m_stateUncertainty[5][5] = 500.0f;

        filter.Predict();

        // TODO: put this into the class, when you understand things better

        float measureX[] =
        {
            -393.66f,
            -375.93f,
            -351.04f,
            -328.96f,
            -299.35f,
            -273.36f,
            -245.89f,
            -222.58f,
            -198.03f,
            -174.17f,
            -146.32f,
            -123.72f,
            -103.47f,
            -78.23f,
            -52.63f,
            -23.34f,
            25.96f,
            49.72f,
            76.94f,
            95.38f,
            119.83f,
            144.01f,
            161.84f,
            180.56f,
            201.42f,
            222.62f,
            239.4f,
            252.51f,
            266.26f,
            271.75f,
            277.4f,
            294.12f,
            301.23f,
            291.8f,
            299.89f
        };

        float measureY[] =
        {
            300.4f,
            301.78f,
            295.1f,
            305.19f,
            301.06f,
            302.05f,
            300.0f,
            303.57f,
            296.33f,
            297.65f,
            297.41f,
            299.61f,
            299.6f,
            302.39f,
            295.04f,
            300.09f,
            294.72f,
            298.61f,
            294.64f,
            284.88f,
            272.82f,
            264.93f,
            251.46f,
            241.27f,
            222.98f,
            203.73f,
            184.1f,
            166.12f,
            138.71f,
            119.71f,
            100.41f,
            79.76f,
            50.62f,
            32.99f,
            2.14f
        };

        filter.Iterate({ measureX[0], measureY[0] });

        int ijkl = 0;
    }

    return 0;
}

/*
TODO:
- rename functions? predict to simulate? update may be ok but is what adapts to the simulation data and the measurements
- output the details of each step to be able to graph it etc.

*/