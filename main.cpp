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
        // Note: the formula shows this plus some noise. That noise is implied (this formula != reality), not actually added in!
        m_stateVector = m_stateUpdateMatrix * m_stateVector + m_controlMatrix * m_controlVector;
        m_stateUncertainty = m_stateUpdateMatrix * m_stateUncertainty * Transpose(m_stateUpdateMatrix) + m_processNoiseMatrix;
    }

    void Iterate(const Vec<NUM_MEASUREMENT_VARIABLES>& measurement)
    {
        // There is a predict as the first step, per https://www.kalmanfilter.net/multiExamples.html
        if (m_firstIterate)
        {
            Predict();
            m_firstIterate = false;
        }

        // ----- 1. Measure -----

        // This was already done and provided to us

        // ----- 2. Update -----

        // Calculate kalman gain
        Mtx<NUM_STATE_VARIABLES, NUM_MEASUREMENT_VARIABLES> kalmanGain = m_stateUncertainty * Transpose(m_observationMatrix) *
            Inverse(m_observationMatrix * m_stateUncertainty * Transpose(m_observationMatrix) + m_measurementUncertainty);

        // estimate current state
        m_stateVector = m_stateVector + kalmanGain * (measurement - m_observationMatrix * m_stateVector);

        // update estimate uncertainty
        const Mtx<NUM_STATE_VARIABLES, NUM_STATE_VARIABLES> I = Identity<NUM_STATE_VARIABLES>();
        m_stateUncertainty = (I - kalmanGain * m_observationMatrix) * m_stateUncertainty * Transpose(I - kalmanGain * m_observationMatrix) +
            kalmanGain * m_measurementUncertainty * Transpose(kalmanGain);

        // ----- 3. Predict -----

        m_stateVector = m_stateUpdateMatrix * m_stateVector + m_controlMatrix * m_controlVector;
        m_stateUncertainty = m_stateUpdateMatrix * m_stateUncertainty * Transpose(m_stateUpdateMatrix) + m_processNoiseMatrix;
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

    bool m_firstIterate = true;
};

int main(int argc, char** argv)
{ 
    // Example 9
    // https://www.kalmanfilter.net/multiExamples.html
    {
        static const float c_deltaT = 1.0f; // Seconds
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
        filter.m_stateUncertainty = Scale<6>(500.0f);

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

        for (int i = 0; i < _countof(measureX); ++i)
            filter.Iterate({ measureX[i], measureY[i] });
    }

    return 0;
}

/*
TODO:
- rename functions? predict to simulate? update may be ok but is what adapts to the simulation data and the measurements
- output the details of each step to be able to graph it etc.

*/