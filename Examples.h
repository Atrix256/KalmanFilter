#pragma once

#include "Kalman.h"

// Example 9
// https://www.kalmanfilter.net/multiExamples.html
inline void Example9()
{
    printf(__FUNCTION__ "...\n");
    const char* fileName = "out/" __FUNCTION__ ".csv";

    static const float c_deltaT = 1.0f; // Seconds
    static const float c_randomAccelerationStdDev = 0.2f; // Meters / Seconds^2
    static const float c_measurementErrorStdDevX = 3.0f; // Meters
    static const float c_measurementErrorStdDevY = 3.0f; // Meters

    KalmanFilter<6, 0, 2> filter;

    filter.m_stateNames = {"pred. x", "pred. x'", "pred. x''", "pred. y", "pred. y'", "pred. y''"};
    filter.m_measurementNames = { "measured x", "measured y" };

    filter.m_stateUpdateMatrix = {
        1.0f, c_deltaT, 0.5f * c_deltaT * c_deltaT, 0.0f,     0.0f,                       0.0f,
        0.0f,     1.0f,                   c_deltaT, 0.0f,     0.0f,                       0.0f,
        0.0f,     0.0f,                       1.0f, 0.0f,     0.0f,                       0.0f,
        0.0f,     0.0f,                       0.0f, 1.0f, c_deltaT, 0.5f * c_deltaT * c_deltaT,
        0.0f,     0.0f,                       0.0f, 0.0f,     1.0f,                   c_deltaT,
        0.0f,     0.0f,                       0.0f, 0.0f,     0.0f,                       1.0f,
    };

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
    {
        filter.Iterate({ measureX[i], measureY[i] });
        filter.OutputState(fileName, i, { measureX[i], measureY[i] });
    }
}

// Example 10
// https://www.kalmanfilter.net/multiExamples.html
inline void Example10()
{
    printf(__FUNCTION__ "...\n");
    const char* fileName = "out/" __FUNCTION__ ".csv";

    static const float c_deltaT = 0.25f; // Seconds
    static const float c_altimeterMeasurementErrorStdDev = 20.0f; // Meters
    static const float c_accelerometerMeasurementErrorStdDev = 0.1f; // Meters / Seconds^2
    static const float c_gravity = -9.8f; // Meters / Seconds^2

    KalmanFilter<2, 1, 1> filter;

    filter.m_stateNames = { "pred. y", "pred. y'" };
    filter.m_measurementNames = { "altimeter" };

    filter.m_stateUpdateMatrix = {
        1.0f, c_deltaT,
        0.0f,     1.0f,
    };

    filter.m_controlMatrix = {
        0.5 * c_deltaT * c_deltaT,
        c_deltaT,
    };

    filter.m_processNoiseMatrix = {
        pow(c_deltaT, 4.0f) / 4.0f, pow(c_deltaT, 3.0f) / 2.0f,
        pow(c_deltaT, 3.0f) / 2.0f, c_deltaT * c_deltaT

    };
    filter.m_processNoiseMatrix = filter.m_processNoiseMatrix * c_accelerometerMeasurementErrorStdDev * c_accelerometerMeasurementErrorStdDev;

    filter.m_measurementUncertainty = {
        c_altimeterMeasurementErrorStdDev * c_altimeterMeasurementErrorStdDev
    };

    // The accelerometer isn't measuring anything that is part of the state, so it isn't
    // part of the observation matrix, but we use it to set the control variable.
    filter.m_observationMatrix = {
        1.0f,
        0.0f,
    };

    // Initial state
    filter.m_stateVector = {};
    filter.m_stateUncertainty = Scale<2>(500.0f);

    float altimeter[] =
    {
        -32.4f,
        -11.1f,
        18.0f,
        22.9f,
        19.5f,
        28.5f,
        46.5f,
        68.9f,
        48.2f,
        56.1f,
        90.5f,
        104.9f,
        140.9f,
        148.0f,
        187.6f,
        209.2f,
        244.6f,
        276.4f,
        323.5f,
        357.3f,
        357.4f,
        398.3f,
        446.7f,
        465.1f,
        529.4f,
        570.4f,
        636.8f,
        693.3f,
        707.3f,
        748.5f
    };

    float accelerometer[] =
    {
        39.72f,
        40.02f,
        39.97f,
        39.81f,
        39.75f,
        39.6f,
        39.77f,
        39.83f,
        39.73f,
        39.87f,
        39.81f,
        39.92f,
        39.78f,
        39.98f,
        39.76f,
        39.86f,
        39.61f,
        39.86f,
        39.74f,
        39.87f,
        39.63f,
        39.67f,
        39.96f,
        39.8f,
        39.89f,
        39.85f,
        39.9f,
        39.81f,
        39.81f,
        39.68f
    };

    filter.m_controlVector = {
        -c_gravity
    };
    filter.DoFirstPrediction();

    for (int i = 0; i < _countof(altimeter); ++i)
    {
        filter.m_controlVector = {
            accelerometer[i] + c_gravity
        };

        filter.Iterate({ altimeter[i] });
        filter.OutputState(fileName, i, { altimeter[i] }, { {accelerometer[i], "accelerometer"} });
    }
}

inline void TestExamples()
{
    Example9();
    Example10();
}
