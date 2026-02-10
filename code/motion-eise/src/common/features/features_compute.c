#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <omp.h>



#include "motion/macros.h"
#include "vec.h"

#include "motion/features/features_compute.h"

RoI_t* features_alloc_RoIs(const size_t max_size) {
    RoI_t* RoIs = (RoI_t*)malloc(max_size * sizeof(RoI_t));
    return RoIs;
}

void features_init_RoIs(RoI_t* RoIs, const size_t max_size) {
    memset(RoIs, 0, max_size * sizeof(RoI_t));
}

void features_free_RoIs(RoI_t* RoIs) {
    free(RoIs);
}

void features_extract(const uint32_t** labels, const int i0, const int i1, const int j0, const int j1,
                      RoI_t* RoIs, const size_t n_RoIs) {

    // Global init
    #pragma omp parallel for schedule(static)
    for (size_t r = 0; r < n_RoIs; r++) {
        RoIs[r].xmin = j1;
        RoIs[r].xmax = j0;
        RoIs[r].ymin = i1;
        RoIs[r].ymax = i0;
        RoIs[r].S = 0;
        RoIs[r].id = 0;
        RoIs[r].x = 0.f;
        RoIs[r].y = 0.f;
    }

    int nthreads = 1;
    #pragma omp parallel
    {
        #pragma omp single
        nthreads = omp_get_num_threads();
    }

    // Thread-local accumulators (one per thread)
    uint32_t* S_loc  = (uint32_t*)calloc((size_t)nthreads * n_RoIs, sizeof(uint32_t));
    uint32_t* Sx_loc = (uint32_t*)calloc((size_t)nthreads * n_RoIs, sizeof(uint32_t));
    uint32_t* Sy_loc = (uint32_t*)calloc((size_t)nthreads * n_RoIs, sizeof(uint32_t));
    uint32_t* xmin_loc = (uint32_t*)malloc((size_t)nthreads * n_RoIs * sizeof(uint32_t));
    uint32_t* xmax_loc = (uint32_t*)malloc((size_t)nthreads * n_RoIs * sizeof(uint32_t));
    uint32_t* ymin_loc = (uint32_t*)malloc((size_t)nthreads * n_RoIs * sizeof(uint32_t));
    uint32_t* ymax_loc = (uint32_t*)malloc((size_t)nthreads * n_RoIs * sizeof(uint32_t));

    if (!S_loc || !Sx_loc || !Sy_loc || !xmin_loc || !xmax_loc || !ymin_loc || !ymax_loc) {
        perror("features_extract alloc");
        exit(1);
    }

    // init bbox locals
    #pragma omp parallel for schedule(static)
    for (size_t t = 0; t < (size_t)nthreads; t++) {
        size_t base = t * n_RoIs;
        for (size_t r = 0; r < n_RoIs; r++) {
            xmin_loc[base + r] = (uint32_t)j1;
            xmax_loc[base + r] = (uint32_t)j0;
            ymin_loc[base + r] = (uint32_t)i1;
            ymax_loc[base + r] = (uint32_t)i0;
        }
    }

    // Pixel accumulation (no atomics)
    // Pixel accumulation (run-length, no atomics)
    #pragma omp parallel
    {
        int tid = omp_get_thread_num();
        size_t base = (size_t)tid * n_RoIs;

        #pragma omp for schedule(static)
        for (int i = i0; i <= i1; i++) {
            const uint32_t* row = labels[i];
            int j = j0;

            while (j <= j1) {
                // 1) skip zeros
                while (j <= j1 && row[j] == 0) j++;
                if (j > j1) break;

                // 2) non-zero segment [j, k)
                uint32_t e = row[j];
                int k = j + 1;
                while (k <= j1 && row[k] == e) k++;

                // 3) accumulate this segment
                size_t r = (size_t)(e - 1);
                uint32_t len = (uint32_t)(k - j);

                S_loc [base + r] += len;

                // sum(j..k-1) = (j + (k-1)) * len / 2
                uint32_t sumx = (uint32_t)(((long long)j + (long long)(k - 1)) * (long long)len / 2);
                Sx_loc[base + r] += sumx;
                Sy_loc[base + r] += (uint32_t)i * len;

                uint32_t uj0 = (uint32_t)j;
                uint32_t uj1 = (uint32_t)(k - 1);
                uint32_t ui  = (uint32_t)i;

                if (uj0 < xmin_loc[base + r]) xmin_loc[base + r] = uj0;
                if (uj1 > xmax_loc[base + r]) xmax_loc[base + r] = uj1;
                if (ui  < ymin_loc[base + r]) ymin_loc[base + r] = ui;
                if (ui  > ymax_loc[base + r]) ymax_loc[base + r] = ui;

                // 4) continue after segment
                j = k;
            }
        }
    }


    // Merge locals into global (parallel over r)
    #pragma omp parallel for schedule(static)
    for (size_t r = 0; r < n_RoIs; r++) {
        uint32_t S = 0, Sx = 0, Sy = 0;
        uint32_t xmin = (uint32_t)j1, xmax = (uint32_t)j0, ymin = (uint32_t)i1, ymax = (uint32_t)i0;

        for (int t = 0; t < nthreads; t++) {
            size_t base = (size_t)t * n_RoIs + r;
            uint32_t St = S_loc[base];
            if (St) {
                S  += St;
                Sx += Sx_loc[base];
                Sy += Sy_loc[base];
                if (xmin_loc[base] < xmin) xmin = xmin_loc[base];
                if (xmax_loc[base] > xmax) xmax = xmax_loc[base];
                if (ymin_loc[base] < ymin) ymin = ymin_loc[base];
                if (ymax_loc[base] > ymax) ymax = ymax_loc[base];
            }
        }

        if (S > 0) {
            RoIs[r].id = (uint32_t)(r + 1);
            RoIs[r].S = S;
            RoIs[r].xmin = xmin;
            RoIs[r].xmax = xmax;
            RoIs[r].ymin = ymin;
            RoIs[r].ymax = ymax;
            RoIs[r].x = (float)Sx / (float)S;
            RoIs[r].y = (float)Sy / (float)S;
        } else {
            RoIs[r].id = 0;
            RoIs[r].S = 0;
        }
    }

    free(S_loc); free(Sx_loc); free(Sy_loc);
    free(xmin_loc); free(xmax_loc); free(ymin_loc); free(ymax_loc);
}


uint32_t features_filter_surface(const uint32_t** in_labels, uint32_t** out_labels, const int i0, const int i1,
                                 const int j0, const int j1, RoI_t* RoIs, const size_t n_RoIs, const uint32_t S_min,
                                 const uint32_t S_max) {
    if (out_labels != NULL && (void*)in_labels != (void*)out_labels)
        for (int i = i0; i <= i1; i++)
            memset(out_labels[i], 0, (j1 - j0 + 1) * sizeof(uint32_t));

    uint32_t x0, x1, y0, y1, id;
    uint32_t cur_label = 1;
    for (size_t i = 0; i < n_RoIs; i++) {
        if (RoIs[i].id) {
            id = RoIs[i].id;
            x0 = RoIs[i].ymin;
            x1 = RoIs[i].ymax;
            y0 = RoIs[i].xmin;
            y1 = RoIs[i].xmax;
            if (S_min > RoIs[i].S || RoIs[i].S > S_max) {
                RoIs[i].id = 0;
                if (out_labels != NULL && ((void*)in_labels == (void*)out_labels)) {
                    for (uint32_t k = x0; k <= x1; k++) {
                        for (uint32_t l = y0; l <= y1; l++) {
                            if (in_labels[k][l] == id)
                                out_labels[k][l] = 0;
                        }
                    }
                }
                continue;
            }
            if (out_labels != NULL) {
                for (uint32_t k = x0; k <= x1; k++) {
                    for (uint32_t l = y0; l <= y1; l++) {
                        if (in_labels[k][l] == id) {
                            out_labels[k][l] = cur_label;
                        }
                    }
                }
            }
            cur_label++;
        }
    }

    return cur_label - 1;
}

void features_shrink_basic(const RoI_t* RoIs_src, const size_t n_RoIs_src, RoI_t* RoIs_dst) {
    size_t cpt = 0;
    for (size_t i = 0; i < n_RoIs_src; i++) {
        if (RoIs_src[i].id) {
            RoIs_dst[cpt].id = cpt + 1;
            RoIs_dst[cpt].xmin = RoIs_src[i].xmin;
            RoIs_dst[cpt].xmax = RoIs_src[i].xmax;
            RoIs_dst[cpt].ymin = RoIs_src[i].ymin;
            RoIs_dst[cpt].ymax = RoIs_src[i].ymax;
            RoIs_dst[cpt].S = RoIs_src[i].S;
            RoIs_dst[cpt].x = RoIs_src[i].x;
            RoIs_dst[cpt].y = RoIs_src[i].y;
            cpt++;
        }
    }
}

void features_labels_zero_init(const RoI_t* RoIs, const size_t n_RoIs, uint32_t** labels) {
        for (size_t i = 0; i < n_RoIs; i++) {
        uint32_t y0 = RoIs[i].ymin;
        uint32_t y1 = RoIs[i].ymax;
        uint32_t x0 = RoIs[i].xmin;
        uint32_t x1 = RoIs[i].xmax;
        for (uint32_t k = y0; k <= y1; k++)
            for (uint32_t l = x0; l <= x1; l++)
                labels[k][l] = 0;
    }
}
