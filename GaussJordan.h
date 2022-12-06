#pragma once

template <size_t M, size_t N>
void GaussJordanElimination(Mtx<M, N>& augmentedMtx)
{
	// augmentedMtx is assumed to be an NxN matrix possibly with some "extra bits" of size (M-N, N) on the right side.

	// make each column in the matrix have only a single row with a value in it, and have that value be 1.
	for (int column = 0; column < M; ++column)
	{
		// find the row that has the maximum absolute value for this column
		int maxValueRowIndex = column;
		double maxValue = augmentedMtx[column][column];
		for (int row = column + 1; row < M; ++row)
		{
			if (abs(augmentedMtx[row][column]) > abs(maxValue))
			{
				maxValue = augmentedMtx[row][column];
				maxValueRowIndex = row;
			}
		}

		// swap rows if we need to
		if (column != maxValueRowIndex)
		{
			for (size_t ix = 0; ix < N; ++ix)
				std::swap(augmentedMtx[column][ix], augmentedMtx[maxValueRowIndex][ix]);
		}

		// scale this row by the value
		{
			double scale = augmentedMtx[column][column];
			for (size_t ix = 0; ix < N; ++ix)
				augmentedMtx[column][ix] /= scale;
		}

		// make only this row have a value in it, by adding or subtracting multiples of it from the other rows
		for (size_t iy = 0; iy < M; ++iy)
		{
			if (iy == column)
				continue;

			double scale = augmentedMtx[iy][column];
			if (scale == 0.0)
				continue;

			for (size_t ix = 0; ix < N; ++ix)
				augmentedMtx[iy][ix] -= augmentedMtx[column][ix] * scale;
		}
	}
}