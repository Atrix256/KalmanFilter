#pragma once

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