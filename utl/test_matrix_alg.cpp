#include "matrix_alg.hpp"
#include <chrono>
#include <cstdio>
#include <cmath>
#include <cstring>

constexpr size_t N = 35;
constexpr size_t M = 17;
constexpr size_t BENCH_ITERS = 10000;
constexpr double TEST_TOL = 1e-9;

// Set to true to run individual tests, false to skip
struct TestConfig {
    bool transpose = true;
    bool transpose_inplace = true;
    bool zeros_ones_eye = true;
    bool copy = true;
    bool scale = true;
    bool mat_mat = true;
    bool mat_vec = true;
    bool transposed_mat_vec = true;
    bool inner_product = true;
    bool abs_test = true;
    bool add_sub = true;
    bool norms = true;
    bool trace_sum = true;
    bool cholesky = true;
    bool cholesky_solve = true;
    bool cholesky_solve_right = true;
    bool determinant = true;
    bool is_pd_psd = true;
    bool is_symmetric = true;
    bool is_singular = true;
    bool is_diagonal = true;
    bool near_zero = true; // Test matrices with near-zero values
};

static TestConfig config;
static int tests_passed = 0;
static int tests_failed = 0;

// ============================================================================
// Helpers
// ============================================================================
#define ASSERT_NEAR(a, b, tol) do { \
    if (std::fabs((a) - (b)) > (tol)) { \
        printf("    FAIL: expected %.10g, got %.10g\n", (double)(b), (double)(a)); \
        tests_failed++; return; \
    } \
} while(0)

#define ASSERT_TRUE(cond) do { \
    if (!(cond)) { \
        printf("    FAIL: condition false\n"); \
        tests_failed++; return; \
    } \
} while(0)

#define TEST_PASS() do { tests_passed++; } while(0)

inline double now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}

// Fill matrix with deterministic values
void fill_matrix(double* A, size_t rows, size_t cols, double seed = 1.0) {
    for (size_t i = 0; i < rows * cols; i++) {
        A[i] = seed * (1.0 + std::sin(i * 0.1) * 0.5);
    }
}

// Fill SPD matrix: A = B * B^T + diag
void fill_spd(double* A, size_t n, double* workspace) {
    double* B = workspace;
    fill_matrix(B, n, n, 0.3);
    // A = B * B^T
    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j <= i; j++) {
            double sum = 0;
            for (size_t k = 0; k < n; k++) sum += B[i*n+k] * B[j*n+k];
            A[i*n+j] = A[j*n+i] = sum;
        }
        A[i*n+i] += n;  // Ensure positive definiteness
    }
}

// ============================================================================
// Tests
// ============================================================================
void test_transpose() {
    if (!config.transpose) return;
    printf("  transpose (35x17 -> 17x35)... ");
    
    alignas(64) double A[N * M], AT[M * N], ATT[N * M];
    fill_matrix(A, N, M);
    
    // Correctness
    transpose(A, M, N, AT);
    for (size_t i = 0; i < N; i++)
        for (size_t j = 0; j < M; j++)
            ASSERT_NEAR(AT[j * N + i], A[i * M + j], TEST_TOL);
    
    transpose(AT, N, M, ATT);
    for (size_t i = 0; i < N * M; i++) ASSERT_NEAR(ATT[i], A[i], TEST_TOL);
    
    // Benchmark
    double t0 = now_ns();
    for (size_t it = 0; it < BENCH_ITERS; it++) transpose(A, M, N, AT);
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (%.1f ns)\n", ns);
    TEST_PASS();
}

void test_transpose_inplace() {
    if (!config.transpose_inplace) return;
    printf("  transpose_inplace_square (35x35)... ");
    
    alignas(64) double A[N * N], A_orig[N * N];
    fill_matrix(A, N, N);
    std::memcpy(A_orig, A, sizeof(A));
    
    transpose_inplace_square(A, N);
    for (size_t i = 0; i < N; i++)
        for (size_t j = 0; j < N; j++)
            ASSERT_NEAR(A[i * N + j], A_orig[j * N + i], TEST_TOL);
    
    double t0 = now_ns();
    for (size_t it = 0; it < BENCH_ITERS; it++) {
        transpose_inplace_square(A, N);
    }
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (%.1f ns)\n", ns);
    TEST_PASS();
}

void test_zeros_ones_eye() {
    if (!config.zeros_ones_eye) return;
    printf("  zeros/ones/eye (35x35)... ");
    
    alignas(64) double A[N * N];
    
    zeros(A, N, N);
    for (size_t i = 0; i < N * N; i++) ASSERT_NEAR(A[i], 0.0, TEST_TOL);
    
    ones(A, N, N);
    for (size_t i = 0; i < N * N; i++) ASSERT_NEAR(A[i], 1.0, TEST_TOL);
    
    eye(A, N, N);
    for (size_t i = 0; i < N; i++)
        for (size_t j = 0; j < N; j++)
            ASSERT_NEAR(A[i*N+j], (i == j) ? 1.0 : 0.0, TEST_TOL);
    
    double t0 = now_ns();
    for (size_t it = 0; it < BENCH_ITERS; it++) eye(A, N, N);
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (eye: %.1f ns)\n", ns);
    TEST_PASS();
}

void test_copy() {
    if (!config.copy) return;
    printf("  copy (35x35)... ");
    
    alignas(64) double A[N * N], B[N * N];
    fill_matrix(A, N, N);
    
    copy(A, N, N, B);
    for (size_t i = 0; i < N * N; i++) ASSERT_NEAR(B[i], A[i], TEST_TOL);
    
    double t0 = now_ns();
    for (size_t it = 0; it < BENCH_ITERS; it++) copy(A, N, N, B);
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (%.1f ns)\n", ns);
    TEST_PASS();
}

void test_scale() {
    if (!config.scale) return;
    printf("  scale/scale_inplace (35x35)... ");
    
    alignas(64) double A[N * N], B[N * N], A_orig[N * N];
    fill_matrix(A, N, N);
    std::memcpy(A_orig, A, sizeof(A));
    
    scale(A, N, N, 2.5, B);
    for (size_t i = 0; i < N * N; i++) ASSERT_NEAR(B[i], A[i] * 2.5, TEST_TOL);
    
    scale_inplace(A, N, N, 2.5);
    for (size_t i = 0; i < N * N; i++) ASSERT_NEAR(A[i], A_orig[i] * 2.5, TEST_TOL);
    
    double t0 = now_ns();
    for (size_t it = 0; it < BENCH_ITERS; it++) scale(A_orig, N, N, 2.5, B);
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (%.1f ns)\n", ns);
    TEST_PASS();
}

void test_mat_mat() {
    if (!config.mat_mat) return;
    printf("  mat_mat (35x35 * 35x35)... ");
    
    alignas(64) double A[N * N], B[N * N], C[N * N], workspace[N * N];
    fill_matrix(A, N, N, 1.0);
    fill_matrix(B, N, N, 0.5);
    
    mat_mat(A, N, N, B, workspace, N, N, C);
    
    // Verify a few elements manually
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            double expected = 0;
            for (size_t k = 0; k < N; k++) expected += A[i*N+k] * B[k*N+j];
            ASSERT_NEAR(C[i*N+j], expected, TEST_TOL);
        }
    }
    
    double t0 = now_ns();
    for (size_t it = 0; it < BENCH_ITERS; it++) mat_mat(A, N, N, B, workspace, N, N, C);
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (%.1f ns)\n", ns);
    TEST_PASS();
}

void test_mat_vec() {
    if (!config.mat_vec) return;
    printf("  mat_vec (35x35 * 35x1)... ");
    
    alignas(64) double A[N * N], x[N], y[N];
    fill_matrix(A, N, N);
    fill_matrix(x, N, 1, 0.7);
    
    mat_vec(A, N, N, x, y);
    
    for (size_t i = 0; i < N; i++) {
        double expected = 0;
        for (size_t j = 0; j < N; j++) expected += A[i*N+j] * x[j];
        ASSERT_NEAR(y[i], expected, TEST_TOL);
    }
    
    double t0 = now_ns();
    for (size_t it = 0; it < BENCH_ITERS; it++) mat_vec(A, N, N, x, y);
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (%.1f ns)\n", ns);
    TEST_PASS();
}

void test_transposed_mat_vec() {
    if (!config.transposed_mat_vec) return;
    printf("  transposed_mat_vec (35x17)^T * 35x1... ");
    
    alignas(64) double A[N * M], x[N], y[M], AT[M * N], y_check[M];
    fill_matrix(A, N, M);
    fill_matrix(x, N, 1, 0.3);
    
    transposed_mat_vec(A, N, M, x, y);
    
    // Verify against explicit transpose
    transpose(A, M, N, AT);
    mat_vec(AT, M, N, x, y_check);
    for (size_t i = 0; i < M; i++) ASSERT_NEAR(y[i], y_check[i], TEST_TOL);
    
    double t0 = now_ns();
    for (size_t it = 0; it < BENCH_ITERS; it++) transposed_mat_vec(A, N, M, x, y);
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (%.1f ns)\n", ns);
    TEST_PASS();
}

void test_inner_product() {
    if (!config.inner_product) return;
    printf("  inner_product (35)... ");
    
    alignas(64) double a[N], b[N];
    fill_matrix(a, N, 1, 1.0);
    fill_matrix(b, N, 1, 0.5);
    
    double result = inner_product(a, b, N);
    double expected = 0;
    for (size_t i = 0; i < N; i++) expected += a[i] * b[i];
    ASSERT_NEAR(result, expected, TEST_TOL);
    
    double t0 = now_ns();
    volatile double r;
    for (size_t it = 0; it < BENCH_ITERS; it++) r = inner_product(a, b, N);
    (void)r;
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (%.1f ns)\n", ns);
    TEST_PASS();
}

void test_abs() {
    if (!config.abs_test) return;
    printf("  abs (35x35)... ");
    
    alignas(64) double A[N * N];
    for (size_t i = 0; i < N * N; i++) A[i] = (i % 2 == 0) ? -(double)i : (double)i;
    
    abs(A, N, N);
    for (size_t i = 0; i < N * N; i++) ASSERT_TRUE(A[i] >= 0);
    
    double t0 = now_ns();
    for (size_t it = 0; it < BENCH_ITERS; it++) abs(A, N, N);
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (%.1f ns)\n", ns);
    TEST_PASS();
}

void test_add_sub() {
    if (!config.add_sub) return;
    printf("  add/sub (35x35)... ");
    
    alignas(64) double A[N * N], B[N * N], C[N * N];
    fill_matrix(A, N, N, 1.0);
    fill_matrix(B, N, N, 0.5);
    
    add(A, B, N, N, C);
    for (size_t i = 0; i < N * N; i++) ASSERT_NEAR(C[i], A[i] + B[i], TEST_TOL);
    
    sub(A, B, N, N, C);
    for (size_t i = 0; i < N * N; i++) ASSERT_NEAR(C[i], A[i] - B[i], TEST_TOL);
    
    double t0 = now_ns();
    for (size_t it = 0; it < BENCH_ITERS; it++) add(A, B, N, N, C);
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (add: %.1f ns)\n", ns);
    TEST_PASS();
}

void test_norms() {
    if (!config.norms) return;
    printf("  l2_norm/frobenius_norm (35x35)... ");
    
    alignas(64) double A[N * N];
    fill_matrix(A, N, N);
    
    double l2 = l2_norm(A, N * N);
    double frob = frobenius_norm(A, N, N);
    ASSERT_NEAR(l2, frob, TEST_TOL);
    
    double expected = 0;
    for (size_t i = 0; i < N * N; i++) expected += A[i] * A[i];
    ASSERT_NEAR(l2, std::sqrt(expected), TEST_TOL);
    
    double t0 = now_ns();
    volatile double r;
    for (size_t it = 0; it < BENCH_ITERS; it++) r = frobenius_norm(A, N, N);
    (void)r;
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (%.1f ns)\n", ns);
    TEST_PASS();
}

void test_trace_sum() {
    if (!config.trace_sum) return;
    printf("  trace/matrix_sum (35x35)... ");
    
    alignas(64) double A[N * N];
    fill_matrix(A, N, N);
    
    double tr = trace(A, N, N);
    double expected_tr = 0;
    for (size_t i = 0; i < N; i++) expected_tr += A[i * N + i];
    ASSERT_NEAR(tr, expected_tr, TEST_TOL);
    
    double sum = matrix_sum(A, N, N);
    double expected_sum = 0;
    for (size_t i = 0; i < N * N; i++) expected_sum += A[i];
    ASSERT_NEAR(sum, expected_sum, TEST_TOL);
    
    double t0 = now_ns();
    volatile double r;
    for (size_t it = 0; it < BENCH_ITERS; it++) r = trace(A, N, N);
    (void)r;
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (trace: %.1f ns)\n", ns);
    TEST_PASS();
}

void test_cholesky() {
    if (!config.cholesky) return;
    printf("  cholesky_decompose (35x35 SPD)... ");
    
    alignas(64) double A[N * N], L[N * N], LLT[N * N], workspace[N * N];
    fill_spd(A, N, workspace);
    
    ASSERT_TRUE(cholesky_decompose(A, L, N));
    
    // Verify L is lower triangular
    for (size_t i = 0; i < N; i++)
        for (size_t j = i + 1; j < N; j++)
            ASSERT_NEAR(L[i * N + j], 0.0, TEST_TOL);
    
    // Verify L * L^T = A
    for (size_t i = 0; i < N; i++) {
        for (size_t j = 0; j <= i; j++) {
            double sum = 0;
            for (size_t k = 0; k <= std::min(i, j); k++) sum += L[i*N+k] * L[j*N+k];
            ASSERT_NEAR(sum, A[i*N+j], 1e-8);
        }
    }
    
    double t0 = now_ns();
    for (size_t it = 0; it < BENCH_ITERS; it++) cholesky_decompose(A, L, N);
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (%.1f ns)\n", ns);
    TEST_PASS();
}

void test_cholesky_solve() {
    if (!config.cholesky_solve) return;
    printf("  cholesky_solve (35x35 * X = 35x17)... ");
    
    alignas(64) double A[N * N], L[N * N], B[N * M], X[N * M], AX[N * M], workspace[N * N];
    fill_spd(A, N, workspace);
    fill_matrix(B, N, M, 0.7);
    
    ASSERT_TRUE(cholesky_decompose(A, L, N));
    cholesky_solve(L, B, X, N, M);
    
    // Verify AX = B
    mat_mat(A, N, N, X, workspace, M, N, AX);
    for (size_t i = 0; i < N * M; i++) ASSERT_NEAR(AX[i], B[i], 1e-7);
    
    double t0 = now_ns();
    for (size_t it = 0; it < BENCH_ITERS; it++) cholesky_solve(L, B, X, N, M);
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (%.1f ns)\n", ns);
    TEST_PASS();
}

void test_cholesky_solve_right() {
    if (!config.cholesky_solve_right) return;
    printf("  cholesky_solve_right (17x35 * A^-1)... ");
    
    alignas(64) double A[N * N], L[N * N], B[M * N], Z[M * N], ZA[M * N];
    alignas(64) double Bt[N * M], Zt[N * M], workspace[N * N];
    fill_spd(A, N, workspace);
    fill_matrix(B, M, N, 0.3);
    
    ASSERT_TRUE(cholesky_decompose(A, L, N));
    cholesky_solve_right(L, B, Z, Bt, Zt, N, M);
    
    // Verify Z * A = B
    mat_mat(Z, N, M, A, workspace, N, N, ZA);
    for (size_t i = 0; i < M * N; i++) ASSERT_NEAR(ZA[i], B[i], 1e-6);
    
    double t0 = now_ns();
    for (size_t it = 0; it < BENCH_ITERS; it++) cholesky_solve_right(L, B, Z, Bt, Zt, N, M);
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (%.1f ns)\n", ns);
    TEST_PASS();
}

void test_determinant() {
    if (!config.determinant) return;
    printf("  determinant (35x35)... ");
    
    alignas(64) double A[N * N], workspace[N * N];
    
    // Test with identity (det = 1)
    eye(A, N, N);
    ASSERT_NEAR(determinant(A, N, workspace), 1.0, TEST_TOL);
    
    // Test with scaled identity (det = 2^N)
    scale_inplace(A, N, N, 2.0);
    double expected = std::pow(2.0, N);
    ASSERT_NEAR(determinant(A, N, workspace), expected, expected * 1e-10);
    
    // Test singular matrix (det = 0)
    fill_matrix(A, N, N);
    for (size_t j = 0; j < N; j++) A[1 * N + j] = A[0 * N + j];  // Row 1 = Row 0
    ASSERT_NEAR(determinant(A, N, workspace), 0.0, TEST_TOL);
    
    fill_matrix(A, N, N);
    double t0 = now_ns();
    volatile double r;
    for (size_t it = 0; it < BENCH_ITERS; it++) r = determinant(A, N, workspace);
    (void)r;
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (%.1f ns)\n", ns);
    TEST_PASS();
}

void test_is_pd_psd() {
    if (!config.is_pd_psd) return;
    printf("  is_PD/is_PSD (35x35)... ");
    
    alignas(64) double A[N * N], workspace[N * N], shifted[N * N];
    
    // SPD matrix
    fill_spd(A, N, workspace);
    ASSERT_TRUE(is_PD(A, N, workspace));
    ASSERT_TRUE(is_PSD(A, N, shifted, workspace));
    
    // Not PD (has negative eigenvalue)
    eye(A, N, N);
    A[0] = -1.0;
    ASSERT_TRUE(!is_PD(A, N, workspace));
    
    double t0 = now_ns();
    fill_spd(A, N, workspace);
    for (size_t it = 0; it < BENCH_ITERS; it++) is_PD(A, N, workspace);
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (is_PD: %.1f ns)\n", ns);
    TEST_PASS();
}

void test_is_symmetric() {
    if (!config.is_symmetric) return;
    printf("  is_symmetric (35x35)... ");
    
    alignas(64) double A[N * N], workspace[N * N];
    
    // Symmetric
    fill_spd(A, N, workspace);
    ASSERT_TRUE(is_symmetric(A, N));
    
    // Not symmetric
    fill_matrix(A, N, N);
    ASSERT_TRUE(!is_symmetric(A, N));
    
    double t0 = now_ns();
    fill_spd(A, N, workspace);
    for (size_t it = 0; it < BENCH_ITERS; it++) is_symmetric(A, N);
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (%.1f ns)\n", ns);
    TEST_PASS();
}

void test_is_singular() {
    if (!config.is_singular) return;
    printf("  is_singular (35x35)... ");
    
    alignas(64) double A[N * N], workspace[N * N];
    
    // Non-singular (identity)
    eye(A, N, N);
    ASSERT_TRUE(!is_singular(A, N, workspace));
    
    // Singular (duplicate row)
    fill_matrix(A, N, N);
    for (size_t j = 0; j < N; j++) A[1 * N + j] = A[0 * N + j];
    ASSERT_TRUE(is_singular(A, N, workspace));
    
    double t0 = now_ns();
    eye(A, N, N);
    for (size_t it = 0; it < BENCH_ITERS; it++) is_singular(A, N, workspace);
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (%.1f ns)\n", ns);
    TEST_PASS();
}

void test_is_diagonal() {
    if (!config.is_diagonal) return;
    printf("  is_diagonal (35x35, 35x17)... ");
    
    alignas(64) double A[N * N], B[N * M];
    
    // Diagonal
    zeros(A, N, N);
    for (size_t i = 0; i < N; i++) A[i * N + i] = i + 1.0;
    ASSERT_TRUE(is_diagonal(A, N, N));
    
    // Not diagonal
    A[0 * N + 1] = 1.0;
    ASSERT_TRUE(!is_diagonal(A, N, N));
    
    // Rectangular diagonal
    zeros(B, N, M);
    for (size_t i = 0; i < M; i++) B[i * M + i] = i + 1.0;
    ASSERT_TRUE(is_diagonal(B, N, M));
    
    double t0 = now_ns();
    zeros(A, N, N);
    for (size_t i = 0; i < N; i++) A[i * N + i] = i + 1.0;
    for (size_t it = 0; it < BENCH_ITERS; it++) is_diagonal(A, N, N);
    double ns = (now_ns() - t0) / BENCH_ITERS;
    
    printf("PASS  (%.1f ns)\n", ns);
    TEST_PASS();
}

void test_near_zero_values() {
    if (!config.near_zero) return;
    printf("  near-zero value handling... ");
    
    alignas(64) double A[N * N], L[N * N], workspace[N * N];
    
    // Matrix with values near LA_EPSILON
    eye(A, N, N);
    for (size_t i = 0; i < N; i++) {
        for (size_t j = 0; j < N; j++) {
            if (i != j) A[i * N + j] = LA_EPSILON * 0.1;  // Below tolerance
        }
    }
    ASSERT_TRUE(is_diagonal(A, N, N, LA_EPSILON));
    
    // SPD with small off-diagonal
    fill_spd(A, N, workspace);
    scale_inplace(A, N, N, LA_EPSILON * 10);  // Scale down
    for (size_t i = 0; i < N; i++) A[i * N + i] += 1.0;  // Ensure PD
    
    bool result = cholesky_decompose(A, L, N);
    // Should still work or fail gracefully
    (void)result;
    
    // Test determinant of near-singular matrix
    eye(A, N, N);
    A[0] = LA_EPSILON * 0.5;  // Very small but not zero
    double det = determinant(A, N, workspace);
    ASSERT_NEAR(det, LA_EPSILON * 0.5, LA_EPSILON);
    
    printf("PASS\n");
    TEST_PASS();
}

// ============================================================================
// Main
// ============================================================================
void run_all_tests() {
    printf("\n=== matalg.hpp Test Suite ===\n");
    printf("Matrix sizes: %zux%zu (square), %zux%zu (rect)\n", N, N, N, M);
    printf("Benchmark iterations: %zu\n\n", BENCH_ITERS);
    
    test_transpose();
    test_transpose_inplace();
    test_zeros_ones_eye();
    test_copy();
    test_scale();
    test_mat_mat();
    test_mat_vec();
    test_transposed_mat_vec();
    test_inner_product();
    test_abs();
    test_add_sub();
    test_norms();
    test_trace_sum();
    test_cholesky();
    test_cholesky_solve();
    test_cholesky_solve_right();
    test_determinant();
    test_is_pd_psd();
    test_is_symmetric();
    test_is_singular();
    test_is_diagonal();
    test_near_zero_values();
    
    printf("\n=== Results ===\n");
    printf("Passed: %d\n", tests_passed);
    printf("Failed: %d\n", tests_failed);
}

int main() {
    // Edit config here to enable/disable individual tests:
    // config.mat_mat = false;  // Example: disable mat_mat test
    
    run_all_tests();
    return tests_failed > 0 ? 1 : 0;
}