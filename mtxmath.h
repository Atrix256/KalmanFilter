#pragma once

#include <array>

template <size_t N>
using Vec = std::array<float, N>;

// Matrix notion of an MxN matrix means it is M values high and N values wide
template <size_t M, size_t N>
using Mtx = std::array<std::array<float, N>, M>;

template <size_t M, size_t N>
Vec<N> Row(const Mtx<M, N>& mtx, int index)
{
    return mtx[index];
}

template <size_t M, size_t N>
Vec<M> Col(const Mtx<M, N>& mtx, int index)
{
    Vec<M> ret;
    for (int i = 0; i < M; ++i)
        ret[i] = mtx[i][index];
    return ret;
}

template <size_t N>
float Dot(const Vec<N>& A, const Vec<N>& B)
{
    float ret = 0.0f;
    for (int i = 0; i < N; ++i)
        ret += A[i] * B[i];
    return ret;
}

template <size_t N, size_t M>
Vec<M> operator *(const Vec<N>& A, const Mtx<N, M>& B)
{
    Vec<M> ret = {};
    for (int i = 0; i < M; ++i)
        ret[i] = Dot(A, Col(B, i));
    return ret;
}

template <size_t M, size_t N>
Vec<M> operator *(const Mtx<M, N>& A, const Vec<N>& B)
{
    Vec<M> ret = {};
    for (int i = 0; i < M; ++i)
        ret[i] = Dot(Row(A, i), B);
    return ret;
}

template <size_t M, size_t N, size_t O>
Mtx<M, O> operator *(const Mtx<M, N>& A, const Mtx<N, O>& B)
{
    Mtx<M, O> ret = {};
    for (int iy = 0; iy < M; ++iy)
        for (int ix = 0; ix < N; ++ix)
            ret[iy][ix] = Dot(Row(A, ix), Col(B, iy));
    return ret;
}

template <size_t M, size_t N>
Mtx<N, M> Transpose(const Mtx<M, N>& A)
{
    Mtx<N, M> ret = {};
    for (int iy = 0; iy < M; ++iy)
        ret[iy] = Col(A, iy);
    return ret;
}
