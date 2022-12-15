#pragma once

#include "mtxmath.h"
#include <string>
#include <vector>

template <size_t NUM_STATE_VARIABLES, size_t NUM_CONTROL_VARIABLES, size_t NUM_MEASUREMENT_VARIABLES>
class KalmanFilter
{
public:
    KalmanFilter()
    {
        char buffer[256];

        for (int i = 0; i < NUM_STATE_VARIABLES; ++i)
        {
            sprintf_s(buffer, "State%i", i);
            m_stateNames[i] = buffer;
        }

        for (int i = 0; i < NUM_MEASUREMENT_VARIABLES; ++i)
        {
            sprintf_s(buffer, "Measurement%i", i);
            m_measurementNames[i] = buffer;
        }
    }

    void Predict()
    {
        // Note: the formula shows this plus some noise. That noise is implied (this formula != reality), not actually added in!
        m_stateVector = m_stateUpdateMatrix * m_stateVector + m_controlMatrix * m_controlVector;
        m_stateUncertainty = m_stateUpdateMatrix * m_stateUncertainty * Transpose(m_stateUpdateMatrix) + m_processNoiseMatrix;
    }

    void DoFirstPrediction()
    {
        Predict();
        m_firstIterate = false;
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
        m_kalmanGain = m_stateUncertainty * Transpose(m_observationMatrix) *
            Inverse(m_observationMatrix * m_stateUncertainty * Transpose(m_observationMatrix) + m_measurementUncertainty);

        // estimate current state
        m_stateVector = m_stateVector + m_kalmanGain * (measurement - m_observationMatrix * m_stateVector);

        // update estimate uncertainty
        const Mtx<NUM_STATE_VARIABLES, NUM_STATE_VARIABLES> I = Identity<NUM_STATE_VARIABLES>();
        m_stateUncertainty = (I - m_kalmanGain * m_observationMatrix) * m_stateUncertainty * Transpose(I - m_kalmanGain * m_observationMatrix) +
            m_kalmanGain * m_measurementUncertainty * Transpose(m_kalmanGain);

        // ----- 3. Predict -----
        Predict();
    }

    struct ExtraValues
    {
        float value;
        const char* label;
    };

    // TODO: output the extra state too!

    void OutputState(const char* fileName, int index, const Vec<NUM_MEASUREMENT_VARIABLES>& measurement, std::vector<ExtraValues> extraValues = {})
    {
        FILE* file;
        if (index == 0)
        {
            fopen_s(&file, fileName, "wb");
            fprintf(file, "\"index\"");
            for (int i = 0; i < NUM_MEASUREMENT_VARIABLES; ++i)
                fprintf(file, ",\"%s\"", m_measurementNames[i].c_str());
            for (int i = 0; i < extraValues.size(); ++i)
                fprintf(file, ",\"%s\"", extraValues[i].label);
            for (int i = 0; i < NUM_STATE_VARIABLES; ++i)
                fprintf(file, ",\"%s\"", m_stateNames[i].c_str());
            for (int iy = 0; iy < NUM_STATE_VARIABLES; ++iy)
                for (int ix = 0; ix < NUM_MEASUREMENT_VARIABLES; ++ix)
                    fprintf(file, ",\"K_%i,%i\"", iy, ix);
            for (int iy = 0; iy < NUM_STATE_VARIABLES; ++iy)
                for (int ix = 0; ix < NUM_STATE_VARIABLES; ++ix)
                    fprintf(file, ",\"Cov(%s,%s)\"", m_stateNames[iy].c_str(), m_stateNames[ix].c_str());
            fprintf(file, "\n");
        }
        else
        {
            fopen_s(&file, fileName, "ab");
        }
        fprintf(file, "\"%i\"", index);
        for (int i = 0; i < NUM_MEASUREMENT_VARIABLES; ++i)
            fprintf(file, ",\"%f\"", measurement[i][0]);
        for (int i = 0; i < extraValues.size(); ++i)
            fprintf(file, ",\"%f\"", extraValues[i].value);
        for (int i = 0; i < NUM_STATE_VARIABLES; ++i)
            fprintf(file, ",\"%f\"", m_stateVector[i][0]);
        for (int iy = 0; iy < NUM_STATE_VARIABLES; ++iy)
            for (int ix = 0; ix < NUM_MEASUREMENT_VARIABLES; ++ix)
                fprintf(file, ",\"%f\"", m_kalmanGain[iy][ix]);
        for (int iy = 0; iy < NUM_STATE_VARIABLES; ++iy)
            for (int ix = 0; ix < NUM_STATE_VARIABLES; ++ix)
                fprintf(file, ",\"%f\"", m_stateUncertainty[iy][ix]);
        fprintf(file, "\n");
        fclose(file);
    }

    // Stored state
    Vec<NUM_STATE_VARIABLES> m_stateVector = {};
    Mtx<NUM_STATE_VARIABLES, NUM_STATE_VARIABLES> m_stateUncertainty = {};

    // Provided from the outside
    Mtx<NUM_STATE_VARIABLES, NUM_STATE_VARIABLES> m_stateUpdateMatrix = {};
    Mtx<NUM_STATE_VARIABLES, NUM_STATE_VARIABLES> m_processNoiseMatrix = {};
    Mtx<NUM_MEASUREMENT_VARIABLES, NUM_MEASUREMENT_VARIABLES> m_measurementUncertainty = {};
    Vec<NUM_CONTROL_VARIABLES> m_controlVector = {};
    Mtx<NUM_STATE_VARIABLES, NUM_CONTROL_VARIABLES> m_controlMatrix = {};
    Mtx<NUM_MEASUREMENT_VARIABLES, NUM_STATE_VARIABLES> m_observationMatrix = {}; // To match measurement values with state vector values

    // Other junk
    std::array<std::string, NUM_STATE_VARIABLES> m_stateNames;
    std::array<std::string, NUM_MEASUREMENT_VARIABLES> m_measurementNames;
    Mtx<NUM_STATE_VARIABLES, NUM_MEASUREMENT_VARIABLES> m_kalmanGain; // just so debug output can show it

    bool m_firstIterate = true;
};