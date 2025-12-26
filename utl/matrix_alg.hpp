#ifndef MATALG_HPP
#define MATALG_HPP
#include <iostream>
#include <cstdio>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <algorithm>

constexpr double LA_EPSILON = 1e-12;
constexpr double LA_SQRT_EPSILON = 1e-6;

void transpose(const double *__restrict__ matrix, size_t width, size_t height, double* __restrict__ out) {
    for (size_t i = 0; i < width; i++) {
        for (size_t j = 0; j < height; j++) {
            out[i * height + j] = matrix[j * width + i];
        }
    }
}

/* Seems hard to do inplace transpose for nonsquare matrices*/
void transpose_inplace_square(double *matrix, size_t N) {
    for (size_t i = 0; i < N; i++) {
        for (size_t j = i + 1; j < N; j++) {
            double temp = matrix[i * N + j];
            matrix[i * N + j] = matrix[j * N + i];
            matrix[j * N + i] = temp;
        }
    }
}

void zeros(double *matrix, size_t n, size_t m) {
    // std::fill(matrix, matrix + n * m, 0.0);
    std::memset(matrix, 0, n * m * sizeof(double)); // More efficient, sets to all 0 bytes
}

void ones(double *matrix, size_t n, size_t m) {
    std::fill(matrix, matrix + n * m, 1.0);
}

void eye(double* matrix, size_t n, size_t m) {
    // Create identity matrix
    zeros(matrix, n, m);
    size_t min_dim = (n < m) ? n : m;
    for (size_t i = 0; i < min_dim; i++) {
        matrix[i * m + i] = 1.0;
    }
}

void copy(const double* __restrict__ src, size_t n, size_t m, double* __restrict__ dst) {
    std::memcpy(dst, src, n * m * sizeof(double));
}

/** Scale matrix: out = A * scalar */
void scale(const double *__restrict__ A, size_t n, size_t m, double scalar, double *__restrict__ out) {
    for (size_t i = 0; i < n * m; i++) {
        out[i] = A[i] * scalar;
    }
}
void scale_inplace(double *matrix, size_t n, size_t m, double scalar) {
    const size_t total = n * m;
    for (size_t i = 0; i < total; i++) {
        matrix[i] *= scalar;
    }
}

/* AB */
void mat_mat(const double *__restrict__ A, size_t a_width, size_t a_height, const double *__restrict__ B, double *__restrict__ workspace, size_t b_width, size_t b_height, double *__restrict__ out)
{
    // Standard matrix multiplication, with transposed B for cache efficiency
    double* BT = workspace;
    transpose(B, b_width, b_height, BT);

    for (size_t i = 0; i < a_height; i++) {
        for (size_t j = 0; j < b_width; j++) {
            double sum = 0.0;
            for (size_t k = 0; k < a_width; k++) {
                sum += A[i * a_width + k] * BT[j * b_height + k];
            }
            out[i * b_width + j] = sum;
        }
    }
}

/**
 * Matrix-vector multiplication: out = A * x
 * A: m x n, x: n x 1, out: m x 1
 * Faster than mat_mat since it removes allocating BT
 */
void mat_vec(const double *__restrict__ A, size_t m, size_t n, const double *__restrict__ x, double *__restrict__ out) {
    for (size_t i = 0; i < m; ++i) {
        double sum = 0.0;
        for (size_t j = 0; j < n; ++j) {
            sum += A[i * n + j] * x[j];
        }
        out[i] = sum;
    }
}

/**
 * Transposed matrix-vector multiplication: out = A^T * x
 * A: m x n, x: m x 1, out: n x 1
 * more efficient than transposing first and matvec
 */
void transposed_mat_vec(const double *__restrict__ A, size_t m, size_t n, const double *__restrict__ x, double *__restrict__ out) {
    std::memset(out, 0, n * sizeof(double));
    for (size_t i = 0; i < m; ++i) {
        const double xi = x[i];
        for (size_t j = 0; j < n; ++j) {
            out[j] += A[i * n + j] * xi;
        }
    }
}

// vectors a and b
double inner_product(const double *__restrict__ a, const double *__restrict__ b, size_t n) {
    double sum = 0.0;
    for (size_t i = 0; i < n; i++) {
        sum += a[i] * b[i];
    }
    return sum;
}

void abs(double* matrix, size_t n, size_t m) {
    // In place modify each element to its absolute value
    for (size_t i = 0; i < n * m; i++) {
        matrix[i] = std::fabs(matrix[i]); // avoids branching
    }
}

void add(const double *__restrict__ A, const double *__restrict__ B, size_t n, size_t m, double *__restrict__ out) {
    for (size_t i = 0; i < n * m; ++i) {
        out[i] = A[i] + B[i];
    }
}

void sub(const double *__restrict__ A, const double *__restrict__ B, size_t n, size_t m, double *__restrict__ out) {
    for (size_t i = 0; i < n * m; ++i) {
        out[i] = A[i] - B[i];
    }
}

// double inv

double l2_norm(const double *vec, size_t n) {
    double sum = 0.0;
    for (size_t i = 0; i < n; i++) {
        sum += vec[i] * vec[i];
    }
    return std::sqrt(sum);
}

/** Frobenius norm (L2 norm of matrix as vector) */
double frobenius_norm(const double *matrix, size_t n, size_t m) {
    return l2_norm(matrix, n * m);
}

double matrix_sum(const double *matrix, size_t n, size_t m) {
    double sum = 0.0;
    for (size_t i = 0; i < n * m; i++) {
        sum += matrix[i];
    }
    return sum;
}

double trace(const double *matrix, size_t n, size_t m) {
    double sum = 0.0;
    const size_t min_dim = (n < m) ? n : m;
    for (size_t i = 0; i < min_dim; i++) {
        sum += matrix[i * m + i];
    }
    return sum;
}

/* Cholesky Decomp, returns false if not positive definite (thus it fails) 
A -> LL^T
L gets zeroed out first
*/
bool cholesky_decompose(const double *__restrict__ A, double *__restrict__ L, size_t N)
{
    // Zero out L
    zeros(L, N, N);

    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j <= i; ++j) {
            double sum = A[i * N + j];
            for (size_t k = 0; k < j; ++k)
                sum -= L[i * N + k] * L[j * N + k];

            if (i == j) {
                if (sum <= LA_EPSILON)
                    return false; // not symmetric PD
                L[i * N + i] = std::sqrt(sum);
            }
            else {
                L[i * N + j] = sum / L[j * N + j];
            }
        }
    }
    return true;
}

void svd_decompose(const double *__restrict__ A, double *__restrict__ U, double *__restrict__ S, double *__restrict__ Vt, size_t N, size_t M)
{
}

void plu_decompose(const double *__restrict__ A, double *__restrict__ L, double *__restrict__ U, size_t *P, size_t N)
{
    // Placeholder implementation
}

void qr_decompose(const double *__restrict__ A, double *__restrict__ Q, double *__restrict__ R, size_t N, size_t M)
{
    // Placeholder implementation
}

/*
FOR PD SYMMETRIC MATRICES (such as covariance matrices)
For solving for X in AX = B
Instead of full inverse, solve AX = B using Cholesky factorization A = LL^T
LL^TX = B

X NxM is unknown matrix of size N x M, pass it in to allow saving allocations
L NxN is cholesky_decompose(A), mark constant to indicate won't change
B NxM
*/
void cholesky_solve(const double *__restrict__ L, const double *__restrict__ B, double *__restrict__ X, size_t N, size_t M) {
    // Y = L^T X
    // Solve LY = B  (forward)
    // store in X since we don't need it after
    for (size_t i = 0; i < N; ++i) {
        const double Lii = L[i * N + i];
        for (size_t c = 0; c < M; ++c)
        {
            double sum = B[i * M + c];
            for (size_t k = 0; k < i; ++k)
                sum -= L[i * N + k] * X[k * M + c];
            X[i * M + c] = sum / Lii;
        }
    }
    // Solve L^T X = Y (backward)
    for (size_t i = N; i > 0; --i) {
        const double Lii = L[(i - 1) * N + i - 1];
        for (size_t c = 0; c < M; ++c)
        {
            double sum = X[(i - 1) * M + c];
            for (size_t k = i; k < N; ++k)
                sum -= L[k * N + i - 1] * X[k * M + c];
            X[(i - 1) * M + c] = sum / Lii;
        }
    }
}

/* For solving for Z in Z = BA^-1 -> ZA = B -> AX = B
L NxN, A = LL^T
B MxN
Z MxN
Bt is buffer of size N x M
Zt is buffer of size N x M
*/
void cholesky_solve_right(const double *__restrict__ L, const double *__restrict__ B, double *__restrict__ Z, double *__restrict__ Bt, double *__restrict__ Zt, size_t N, size_t M)
{
    // Transpose B -> Bt (N x R)
    transpose(B, N, M, Bt);
    // Solve AX = Bt -> X = Z^T
    cholesky_solve(L, Bt, Zt, N, M);
    // Transpose back Z^T -> Z
    transpose(Zt, M, N, Z);
}

/*
CHECK THIS IS RIGHT
*/
double determinant(const double *__restrict__ A, size_t n, double *__restrict__ workspace) {
    // Copy A to workspace for LU decomposition in place
    std::memcpy(workspace, A, n * n * sizeof(double));

    double det = 1.0;
    int sign = 1;

    for (size_t k = 0; k < n; ++k) {
        // Find pivot
        size_t max_row = k;
        double max_val = std::fabs(workspace[k * n + k]);
        for (size_t i = k + 1; i < n; ++i) {
            double val = std::fabs(workspace[i * n + k]);
            if (val > max_val) {
                max_val = val;
                max_row = i;
            }
        }

        if (max_val < LA_EPSILON)
            return 0.0; // Singular

        // Swap rows if needed
        if (max_row != k) {
            for (size_t j = 0; j < n; ++j) {
                double tmp = workspace[k * n + j];
                workspace[k * n + j] = workspace[max_row * n + j];
                workspace[max_row * n + j] = tmp;
            }
            sign = -sign;
        }

        det *= workspace[k * n + k];

        // Eliminate below
        for (size_t i = k + 1; i < n; ++i) {
            double factor = workspace[i * n + k] / workspace[k * n + k];
            for (size_t j = k + 1; j < n; ++j) {
                workspace[i * n + j] -= factor * workspace[k * n + j];
            }
        }
    }

    return sign * det;
}

/* The easiest test of PD is to try a Cholesky factorization */
bool is_PD(const double *A, size_t N, double *workspace) {
    return cholesky_decompose(A, workspace, N);
}

/* Uses workspace to shift A by tolerance */
bool is_PSD(const double *A, size_t n, double *workspace) {
    // Try is_PD on A + tol*I
    double *shifted = workspace;
    std::memcpy(shifted, A, n * n * sizeof(double));
    for (size_t i = 0; i < n; i++) {
        shifted[i * n + i] += LA_SQRT_EPSILON; // LA_EPSILON is too strict
    }

    return is_PD(shifted, n, workspace + n * n);
}

/*Test symmetry within tolerance tol (allows for small floating-point differences) 
make sure to check dims right before calling */
bool is_symmetric(const double *A, size_t N, double tol = LA_EPSILON) {
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = i + 1; j < N; ++j) {
            double aij = A[i * N + j];
            double aji = A[j * N + i];
            if (std::fabs(aij - aji) > tol * (1.0 + std::fabs(aij) + std::fabs(aji)))
                return false;
        }
    }
    return true;
}

/** Uses determinant function to check */
bool is_singular(const double *A, size_t n, double *workspace, double tol = LA_EPSILON) {
    return std::fabs(determinant(A, n, workspace)) < tol;
}

bool is_diagonal(const double *A, size_t n, size_t m, double tol = LA_EPSILON) {
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < m; ++j) {
            if (i != j && std::fabs(A[i * m + j]) > tol) {
                return false;
            }
        }
    }
    return true;
}

// double matrix_derivative

size_t _printed_length(double value) {
    char buffer[64]; // big enough for any double representation
    int len = std::snprintf(buffer, sizeof(buffer), "%g", value);
    return static_cast<size_t>(len);
}

void print_matrix(double* matrix, size_t n, size_t m) {
    size_t longest_num_in_col[n];
    for (size_t col = 0; col < n; col++) {
        longest_num_in_col[col] = 0;
        for (size_t row = 0; row < m; row++) {
            size_t len = _printed_length(matrix[row * n + col]);
            if (len > longest_num_in_col[col]) {
                longest_num_in_col[col] = len;
            }
        }
    }

    std::cout << "[";
    for (size_t row = 0; row < m; row++) {
        if (row > 0) std::cout << " ";
        std::cout << "[";

        for (size_t col = 0; col < n; col++) {
            size_t pad_size = longest_num_in_col[col] - _printed_length(matrix[row * n + col]);
            std::cout << std::string(pad_size + 1, ' ') << matrix[row * n + col];
            if (col < n - 1) std::cout << ",";
        }

        std::cout << " ";
        if (row == m - 1) std::cout << "]";
        std::cout << "]" << std::endl;
    }
}

#endif // MATALG_HPP