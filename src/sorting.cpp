/*
 * sorting.cpp
 *
 *  Created on: 30 janv. 2017
 *      Author: Arnaud ROSAY
 */

#include "sorting.h"

void merge (uint32_t *a, uint32_t n, uint32_t m) {
	uint32_t i, j, k;
    uint32_t *x = (uint32_t*) malloc(n * sizeof (uint32_t));
    for (i = 0, j = m, k = 0; k < n; k++) {
        x[k] = j == n      ? a[i++]
             : i == m      ? a[j++]
             : a[j] < a[i] ? a[j++]
             :               a[i++];
    }
    for (i = 0; i < n; i++) {
        a[i] = x[i];
    }
    free(x);
}
 
void merge_sort (uint32_t *a, uint32_t n) {
    if (n < 2)
        return;
    uint32_t m = n / 2;
    merge_sort(a, m);
    merge_sort(a + m, n - m);
    merge(a, n, m);
}
