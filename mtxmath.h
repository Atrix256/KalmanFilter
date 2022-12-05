#pragma once

#include <array>

// Matrix notion of an MxN matrix means it is M values high and N values wide
template <size_t M, size_t N>
using Mtx = std::array<std::array<float, N>, M>;

template <size_t N>
using Vec = Mtx<N, 1>;

#include "GaussJordan.h"

// ============================= VECTOR / VECTOR =============================

template <size_t N>
float Dot(const Vec<N>& A, const Vec<N>& B)
{
    float ret = 0.0f;
    for (int i = 0; i < N; ++i)
        ret += A[i][0] * B[i][0];
    return ret;
}

// ============================= MATRIX =============================

template <size_t M, size_t N>
Vec<N> Row(const Mtx<M, N>& mtx, int index)
{
    Vec<N> ret = {};
    for (int i = 0; i < N; ++i)
        ret[i][0] = mtx[index][i];
    return ret;
}

template <size_t M, size_t N>
Vec<M> Col(const Mtx<M, N>& mtx, int index)
{
    Vec<M> ret = {};
    for (int i = 0; i < M; ++i)
        ret[i][0] = mtx[i][index];
    return ret;
}

template <size_t M, size_t N>
Mtx<N, M> Transpose(const Mtx<M, N>& A)
{
    Mtx<N, M> ret = {};
    for (int iy = 0; iy < M; ++iy)
        for (int ix = 0; ix < N; ++ix)
            ret[ix][iy] = A[iy][ix];
    return ret;
}

template <size_t N>
Mtx<N, N> Inverse(const Mtx<N, N>& A)
{
    // Make the augmented matrix
    Mtx<N, N * 2> augmentedMtx;
    for (int iy = 0; iy < N; ++iy)
    {
        for (int ix = 0; ix < N; ++ix)
        {
            augmentedMtx[iy][ix] = A[iy][ix];
            augmentedMtx[iy][ix + N] = (ix == iy) ? 1.0f : 0.0f;
        }
    }

    // Invert
    GaussJordanElimination(augmentedMtx);

    // Get inverted matrix
    Mtx<N, N> ret;
    for (int iy = 0; iy < N; ++iy)
        for (int ix = 0; ix < N; ++ix)
            ret[iy][ix] = augmentedMtx[iy][ix + N];
    return ret;
}

template <size_t N>
Mtx<N, N> Identity()
{
    Mtx<N, N> ret = {};
    for (int i = 0; i < N; ++i)
        ret[i][i] = 1.0f;
    return ret;
}

// ============================= MATRIX / MATRIX =============================

template <size_t M, size_t N, size_t O>
Mtx<M, O> operator *(const Mtx<M, N>& A, const Mtx<N, O>& B)
{
    Mtx<M, O> ret = {};
    for (int iy = 0; iy < M; ++iy)
        for (int ix = 0; ix < O; ++ix)
            ret[iy][ix] = Dot(Row(A, iy), Col(B, ix));
    return ret;
}

template <size_t M, size_t N>
Mtx<M, N> operator +(const Mtx<M, N>& A, const Mtx<M, N>& B)
{
    Mtx<M, N> ret;
    for (int iy = 0; iy < M; ++iy)
        for (int ix = 0; ix < N; ++ix)
            ret[iy][ix] = A[iy][ix] + B[iy][ix];
    return ret;
}

template <size_t M, size_t N>
Mtx<M, N> operator -(const Mtx<M, N>& A, const Mtx<M, N>& B)
{
    Mtx<M, N> ret;
    for (int iy = 0; iy < M; ++iy)
        for (int ix = 0; ix < N; ++ix)
            ret[iy][ix] = A[iy][ix] - B[iy][ix];
    return ret;
}

// ============================= MATRIX / SCALAR =============================

template <size_t M, size_t N>
Mtx<M, N> operator *(const Mtx<M, N>& A, float B)
{
    Mtx<M, N> ret;
    for (int iy = 0; iy < M; ++iy)
        for (int ix = 0; ix < N; ++ix)
            ret[iy][ix] = A[iy][ix] * B;
    return ret;
}

template <size_t M, size_t N>
Mtx<M, N> operator *(float A, const Mtx<M, N>& B)
{
    return B * A;
}

// ============================= MATRIX / VECTOR =============================
