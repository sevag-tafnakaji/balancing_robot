#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "cholesky.c"
#include "esp_log.h"

float** initialise_matrix(int dim) {
  // initialise dim x dim matrix with zeros
  float** mat = (float**)malloc(sizeof(float*) * dim);
  for (int i = 0; i < dim; i++) {
    mat[i] = (float*)malloc(sizeof(float) * dim);
  }

  mat[0][0] = 0.0f;
  mat[0][1] = 0.0f;
  mat[0][2] = 0.0f;
  mat[1][0] = 0.0f;
  mat[1][1] = 0.0f;
  mat[1][2] = 0.0f;
  mat[2][0] = 0.0f;
  mat[2][1] = 0.0f;
  mat[2][2] = 0.0f;

  return mat;
}

static void transpose_matrix(float** mat, float** trans, int dim) {
  // assume mat is sized dim x dim

  for (int i = 0; i < dim; i++) {
    for (int j = 0; j < dim; j++) {
      trans[j][i] = mat[i][j];
    }
  }
}

static float sample_standard_gaussian() {
  float x = (float)random() / RAND_MAX;
  float y = (float)random() / RAND_MAX;
  float z = sqrt(-2 * log(x)) * cos(2 * M_PI * y);

  return z;
}

void mult_mat_w_vec(float** mat, float* vec, float* dest, int dim) {
  for (int i = 0; i < dim; i++) {
    float sum = 0;
    for (int j = 0; j < dim; j++) {
      sum += mat[i][j] * vec[i];
    }
    dest[i] = sum;
  }
}

void add_vec(float* vec1, float* vec2, float* dest, int dim) {
  for (int i = 0; i < dim; i++) {
    dest[i] = vec1[i] + vec2[i];
  }
}

static void test_chol(float** cov, int dim) {
  float** covT = initialise_matrix(dim);
  float** chol = initialise_matrix(dim);
  float** cholT = initialise_matrix(dim);

  ESP_LOGI("TEST_CHOL",
           "[[%d.%d, %d.%d, %d.%d]\n[%d.%d, %d.%d, %d.%d]\n[%d.%d, %d.%d, "
           "%d.%d]]",
           (int)cov[0][0], (int)(fabs(cov[0][0]) * 100) % 100, (int)cov[0][1],
           (int)(fabs(cov[0][1]) * 100) % 100, (int)cov[0][2],
           (int)(fabs(cov[0][2]) * 100) % 100, (int)cov[1][0],
           (int)(fabs(cov[1][0]) * 100) % 100, (int)cov[1][1],
           (int)(fabs(cov[1][1]) * 100) % 100, (int)cov[1][2],
           (int)(fabs(cov[1][2]) * 100) % 100, (int)cov[2][0],
           (int)(fabs(cov[2][0]) * 100) % 100, (int)cov[2][1],
           (int)(fabs(cov[2][1]) * 100) % 100, (int)cov[2][2],
           (int)(fabs(cov[2][2]) * 100) % 100);

  transpose_matrix(cov, covT, dim);

  cholesky(cov, dim, covT, dim, chol, cholT, 0);

  ESP_LOGI(
      "TEST_CHOL",
      "[[%d.%d, %d.%d, %d.%d]\n[%d.%d, %d.%d, %d.%d]\n[%d.%d, %d.%d, %d.%d]]",
      (int)chol[0][0], (int)(fabs(chol[0][0]) * 100) % 100, (int)chol[0][1],
      (int)(fabs(chol[0][1]) * 100) % 100, (int)chol[0][2],
      (int)(fabs(chol[0][2]) * 100) % 100, (int)chol[1][0],
      (int)(fabs(chol[1][0]) * 100) % 100, (int)chol[1][1],
      (int)(fabs(chol[1][1]) * 100) % 100, (int)chol[1][2],
      (int)(fabs(chol[1][2]) * 100) % 100, (int)chol[2][0],
      (int)(fabs(chol[2][0]) * 100) % 100, (int)chol[2][1],
      (int)(fabs(chol[2][1]) * 100) % 100, (int)chol[2][2],
      (int)(fabs(chol[2][2]) * 100) % 100);

  // Cleanup of float pointer not used anymore
  for (int i = 0; i < dim; i++) {
    free(chol[i]);
    free(cholT[i]);
    free(covT[i]);
  }
  free(chol);
  free(cholT);
  free(covT);
}

static void sample_multivariate_gaussian(float* mean, float** cov, float* dest,
                                         int dim) {
  // Initialising
  float** covT = initialise_matrix(dim);
  float** chol = initialise_matrix(dim);
  float** cholT = initialise_matrix(dim);

  transpose_matrix(cov, covT, dim);

  cholesky(cov, dim, covT, dim, chol, cholT, 0);

  float* z;

  // Generate independent samples
  for (int i = 0; i < dim; i++) {
    z[i] = sample_standard_gaussian();
  }

  // Transform into multivariate gaussian using the cholesky decomposition
  mult_mat_w_vec(chol, z, z, dim);  // store result in z
  add_vec(mean, z, dest, dim);

  // Cleanup of float pointer not used anymore
  for (int i = 0; i < dim; i++) {
    free(chol[i]);
    free(cholT[i]);
    free(covT[i]);
  }
  free(chol);
  free(cholT);
  free(covT);
}
