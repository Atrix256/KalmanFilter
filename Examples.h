#pragma once

#include "Kalman.h"

inline void TestExamples()
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
}