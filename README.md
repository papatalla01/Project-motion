# Motion Project
## Optimization of Moving Object Detection and Tracking

## Description

This project focuses on the complete optimization of a **moving object detection and tracking pipeline** from video streams.  
The main objective is to significantly improve performance (FPS) while preserving detection accuracy.

The work is based on a classical pipeline using the **Sigma-Delta** method, enhanced with morphological operations, **Connected Component Labeling (CCL)**, and **Connected Component Analysis (CCA)**. The pipeline is then optimized at multiple levels: algorithmic design, memory access, CPU parallelism, and SIMD vectorization.

---

## Processing Pipeline

The global pipeline consists of the following stages:

- Grayscale image acquisition  
- Motion detection (Sigma-Delta)  
- Morphological filtering (erosion, dilation)  
- Connected Component Labeling (CCL)  
- Connected Component Analysis (CCA)  
- Temporal tracking of detected objects  

A simplification of the initial pipeline eliminated redundant processing between consecutive frames, significantly reducing computation time.

---

## Implemented Optimizations

### 1. Algorithmic Simplification

- Removal of explicit recomputation of the *t-1* frame  
- Use of buffer pointer swapping  
- Immediate performance gain of approximately **2×** compared to the initial version  

### 2. OpenMP Parallelization

- Sigma-Delta parallelized using `#pragma omp for`  
- Loop fusion into single parallel regions  
- Targeted parallelization of morphological operators  
- Full parallelization of CCL and CCA using per-thread local buffers  

### 3. Memory Optimizations

- Use of **row pointers** instead of 2D array indexing  
- Reduction of address computation overhead  
- Improved cache locality  
- Easier and more efficient vectorization  

### 4. SIMD Vectorization

- SIMD (AVX2) implementation of the Sigma-Delta algorithm  
- Elimination of conditional branches  
- Maximized intra-core data-level parallelism  

### 5. CPU Allocation

- Explicit tuning of the number of threads  
- Optimized CPU core allocation to maximize throughput  

---

## Performance

| Stage | Performance |
|------|------------|
| Initial version (motion2) | ~17 FPS |
| Optimized baseline | ~34 FPS |
| OpenMP Sigma-Delta | ~77 FPS |
| Optimized morphology | >220 FPS |
| Optimized CCL | ~310 FPS |
| Optimized CCA | ~249 FPS |
| SIMD Sigma-Delta | ~702 FPS |
| Fully optimized pipeline | ~1820 FPS |

Memory and SIMD optimizations proved to be the most impactful improvements in this project.

---

## Technologies Used

- **Language:** C / C++  
- **Parallelism:** OpenMP  
- **Vectorization:** SIMD (AVX2)  
- **Architecture:** Multi-core CPU  
- **Environment:** Linux & Dalek LIP6  

---

## Author and Supervisor

- **Author:** Papa Talla Dioum  
- **Supervisor:** Adrien Cassagne  
