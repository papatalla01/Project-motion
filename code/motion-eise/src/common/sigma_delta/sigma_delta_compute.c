#include <math.h>
#include <stdlib.h>
#include <nrc2.h>
#include <immintrin.h>
#include <stdint.h>


#include "motion/macros.h"
#include "motion/sigma_delta/sigma_delta_compute.h"

sigma_delta_data_t* sigma_delta_alloc_data(const int i0, const int i1, const int j0, const int j1, const uint8_t vmin,
                                           const uint8_t vmax) {
    sigma_delta_data_t* sd_data = (sigma_delta_data_t*)malloc(sizeof(sigma_delta_data_t));
    sd_data->i0 = i0;
    sd_data->i1 = i1;
    sd_data->j0 = j0;
    sd_data->j1 = j1;
    sd_data->vmin = vmin;
    sd_data->vmax = vmax;
    sd_data->M = ui8matrix(sd_data->i0, sd_data->i1, sd_data->j0, sd_data->j1);
    sd_data->O = ui8matrix(sd_data->i0, sd_data->i1, sd_data->j0, sd_data->j1);
    sd_data->V = ui8matrix(sd_data->i0, sd_data->i1, sd_data->j0, sd_data->j1);
    return sd_data;
}

void sigma_delta_init_data(sigma_delta_data_t* sd_data, const uint8_t** img_in, const int i0, const int i1,
                           const int j0, const int j1) {
    for (int i = i0; i <= i1; i++) {
        for (int j = j0; j <= j1; j++) {
            sd_data->M[i][j] = img_in != NULL ? img_in[i][j] : sd_data->vmax;
            sd_data->V[i][j] = sd_data->vmin;
        }
    }
}

void sigma_delta_free_data(sigma_delta_data_t* sd_data) {
    free_ui8matrix(sd_data->M, sd_data->i0, sd_data->i1, sd_data->j0, sd_data->j1);
    free_ui8matrix(sd_data->O, sd_data->i0, sd_data->i1, sd_data->j0, sd_data->j1);
    free_ui8matrix(sd_data->V, sd_data->i0, sd_data->i1, sd_data->j0, sd_data->j1);
    free(sd_data);
}

void sigma_delta_compute(sigma_delta_data_t *sd_data, const uint8_t** img_in, uint8_t** img_out,
                         const int i0, const int i1, const int j0, const int j1, const uint8_t N)
{
    const int W = 32; // AVX2 = 32 bytes

    // Constantes vectorielles
    const __m256i vone  = _mm256_set1_epi8((char)1);
    const __m256i vvmin = _mm256_set1_epi8((char)sd_data->vmin);
    const __m256i vvmax = _mm256_set1_epi8((char)sd_data->vmax);
    const __m256i v0    = _mm256_setzero_si256();
    const __m256i v255  = _mm256_set1_epi8((char)0xFF);

    // Astuce : si N=2 (cas de votre pipeline), thr = 2*O => shift (en u16) au lieu de mul
    const int N_is_2 = (N == 2);

    #pragma omp parallel for schedule(static)
    for (int i = i0; i <= i1; i++) {
        uint8_t*       Mi   = sd_data->M[i];
        uint8_t*       Oi   = sd_data->O[i];
        uint8_t*       Vi   = sd_data->V[i];
        const uint8_t* Ini  = img_in[i];
        uint8_t*       Outi = img_out[i];

        int j = j0;

        // ---------------------------
        // Boucle AVX2 (32 pixels)
        // ---------------------------
        for (; j <= j1 - (W - 1); j += W) {
            __m256i rM = _mm256_loadu_si256((const __m256i*)(Mi  + j));
            __m256i rI = _mm256_loadu_si256((const __m256i*)(Ini + j));
            __m256i rV = _mm256_loadu_si256((const __m256i*)(Vi  + j));

            // ---- Step 1: M <- M +/- 1 vers I
            // masks signed compare ok ici car valeurs [0..255] mais compare signed casse au delà de 127.
            // => on fait du compare unsigned en transformant via xor 0x80.
            const __m256i bias = _mm256_set1_epi8((char)0x80);
            __m256i rM_b = _mm256_xor_si256(rM, bias);
            __m256i rI_b = _mm256_xor_si256(rI, bias);

            __m256i m_lt_i = _mm256_cmpgt_epi8(rI_b, rM_b); // I > M
            __m256i m_gt_i = _mm256_cmpgt_epi8(rM_b, rI_b); // M > I

            // masked add/sub
            __m256i inc = _mm256_and_si256(m_lt_i, vone);
            __m256i dec = _mm256_and_si256(m_gt_i, vone);
            rM = _mm256_add_epi8(rM, inc);
            rM = _mm256_sub_epi8(rM, dec);

            // ---- Step 2: O = |M - I| en uint8
            // abs diff unsigned : max - min (unsigned)
            __m256i rMax = _mm256_max_epu8(rM, rI);
            __m256i rMin = _mm256_min_epu8(rM, rI);
            __m256i rO   = _mm256_sub_epi8(rMax, rMin);

            // ---- Step 3: update V vers thr = N*O, clamp [vmin,vmax]
            // On travaille en u16 : split low/high 16 bytes

            __m128i Olo_128 = _mm256_castsi256_si128(rO);
            __m128i Ohi_128 = _mm256_extracti128_si256(rO, 1);

            __m128i Vlo_128 = _mm256_castsi256_si128(rV);
            __m128i Vhi_128 = _mm256_extracti128_si256(rV, 1);

            __m256i Olo = _mm256_cvtepu8_epi16(Olo_128);
            __m256i Ohi = _mm256_cvtepu8_epi16(Ohi_128);
            __m256i Vlo = _mm256_cvtepu8_epi16(Vlo_128);
            __m256i Vhi = _mm256_cvtepu8_epi16(Vhi_128);

            __m256i Thr_lo, Thr_hi;
            if (N_is_2) {
                Thr_lo = _mm256_add_epi16(Olo, Olo);
                Thr_hi = _mm256_add_epi16(Ohi, Ohi);
            } else {
                __m256i vN = _mm256_set1_epi16((short)N);
                Thr_lo = _mm256_mullo_epi16(Olo, vN);
                Thr_hi = _mm256_mullo_epi16(Ohi, vN);
            }

            __m256i v_lt_thr_lo = _mm256_cmpgt_epi16(Thr_lo, Vlo); // Thr > V  <=> V < Thr
            __m256i v_gt_thr_lo = _mm256_cmpgt_epi16(Vlo, Thr_lo); // V > Thr
            __m256i v_lt_thr_hi = _mm256_cmpgt_epi16(Thr_hi, Vhi);
            __m256i v_gt_thr_hi = _mm256_cmpgt_epi16(Vhi, Thr_hi);

            // V += 1 si V < Thr
            Vlo = _mm256_add_epi16(Vlo, _mm256_and_si256(v_lt_thr_lo, _mm256_set1_epi16(1)));
            Vhi = _mm256_add_epi16(Vhi, _mm256_and_si256(v_lt_thr_hi, _mm256_set1_epi16(1)));

            // V -= 1 si V > Thr
            Vlo = _mm256_sub_epi16(Vlo, _mm256_and_si256(v_gt_thr_lo, _mm256_set1_epi16(1)));
            Vhi = _mm256_sub_epi16(Vhi, _mm256_and_si256(v_gt_thr_hi, _mm256_set1_epi16(1)));

            // clamp [vmin, vmax] en u16
            __m256i vmin16 = _mm256_set1_epi16((short)sd_data->vmin);
            __m256i vmax16 = _mm256_set1_epi16((short)sd_data->vmax);

            Vlo = _mm256_max_epu16(Vlo, vmin16);
            Vlo = _mm256_min_epu16(Vlo, vmax16);
            Vhi = _mm256_max_epu16(Vhi, vmin16);
            Vhi = _mm256_min_epu16(Vhi, vmax16);

            // repack u16->u8
            __m256i rV_new = _mm256_packus_epi16(Vlo, Vhi);
            // packus met dans chaque 128-bit : [lo..] puis [hi..] mais avec permutation
            // On corrige l’ordre
            rV_new = _mm256_permute4x64_epi64(rV_new, 0xD8);

            // ---- Step 4: out = (O < V) ? 0 : 255 (unsigned)
            __m256i rO_b = _mm256_xor_si256(rO, bias);
            __m256i rVb  = _mm256_xor_si256(rV_new, bias);
            __m256i o_lt_v = _mm256_cmpgt_epi8(rVb, rO_b); // V > O <=> O < V

            // out = 0 si o_lt_v, sinon 255
            __m256i rOut = _mm256_blendv_epi8(v255, v0, o_lt_v);

            // store
            _mm256_storeu_si256((__m256i*)(Mi   + j), rM);
            _mm256_storeu_si256((__m256i*)(Oi   + j), rO);
            _mm256_storeu_si256((__m256i*)(Vi   + j), rV_new);
            _mm256_storeu_si256((__m256i*)(Outi + j), rOut);
        }

        // ---------------------------
        // Tail scalaire
        // ---------------------------
        for (; j <= j1; j++) {
            uint8_t m  = Mi[j];
            uint8_t in = Ini[j];
            if (m < in) m++;
            else if (m > in) m--;
            Mi[j] = m;

            int d = (int)m - (int)in;
            uint8_t o = (uint8_t)(d < 0 ? -d : d);
            Oi[j] = o;

            uint8_t v = Vi[j];
            uint16_t thr = (uint16_t)N * (uint16_t)o;
            if (v < thr) v++;
            else if (v > thr) v--;
            if (v < sd_data->vmin) v = sd_data->vmin;
            if (v > sd_data->vmax) v = sd_data->vmax;
            Vi[j] = v;

            Outi[j] = (o < v) ? 0 : 255;
        }
    }
}

