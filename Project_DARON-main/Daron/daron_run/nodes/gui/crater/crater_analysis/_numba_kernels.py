"""Optional JIT kernels. Preserve NumPy NaN semantics; no fastmath or parallel reductions."""
import numpy as np
from numba import njit


@njit(cache=True, fastmath=False, nogil=True)
def median_numpy(values):
    # Numba's median does not always propagate NaN like NumPy's median.
    # Preserve the original invalid-cell semantics explicitly.
    for value in values:
        if np.isnan(value):
            return np.nan
    return np.median(values)


@njit(cache=True, fastmath=False, nogil=True)
def grouped_medians(sorted_values, breaks):
    result = np.empty((len(breaks)-1, sorted_values.shape[1]), np.float64)
    for i in range(len(breaks)-1):
        a, b = breaks[i], breaks[i+1]
        for j in range(sorted_values.shape[1]):
            result[i, j] = median_numpy(sorted_values[a:b, j])
    return result


@njit(cache=True, fastmath=False, nogil=True)
def cell_statistics(points, residual, breaks, minimum):
    n = len(breaks)-1
    xy = np.empty((n, 2), np.float64)
    delta = np.empty(n, np.float64)
    spread = np.empty(n, np.float64)
    count = np.empty(n, np.int64)
    used = 0
    for i in range(n):
        a, b = breaks[i], breaks[i+1]
        if b-a < minimum:
            continue
        med = median_numpy(residual[a:b])
        xy[used, 0] = median_numpy(points[a:b, 0])
        xy[used, 1] = median_numpy(points[a:b, 1])
        delta[used] = med
        spread[used] = 1.4826*median_numpy(np.abs(residual[a:b]-med))
        count[used] = b-a
        used += 1
    return xy[:used], delta[:used], spread[:used], count[:used]


@njit(cache=True, fastmath=False, nogil=True)
def profile_cells(rho, sectors, residual, background, centers, width, nsector, minimum, has_background):
    shape = (nsector, len(centers))
    heights = np.full(shape, np.nan)
    positions = np.full(shape, np.nan)
    backgrounds = np.full(shape, np.nan)
    counts = np.zeros(shape, np.int64)
    selected = np.empty(len(rho), np.int64)
    for s in range(nsector):
        for j in range(len(centers)):
            lower, upper = centers[j]-width/2, centers[j]+width/2
            n = 0
            for k in range(len(rho)):
                if sectors[k] == s and rho[k] >= lower and rho[k] < upper:
                    selected[n] = k
                    n += 1
            counts[s, j] = n
            if n >= minimum:
                take = selected[:n]
                heights[s, j] = median_numpy(residual[take])
                positions[s, j] = median_numpy(rho[take])
                if has_background:
                    backgrounds[s, j] = median_numpy(background[take])
    return heights, counts, positions, backgrounds


@njit(cache=True, fastmath=False, nogil=True)
def component_count(nvertices, edges, active):
    parent = np.arange(nvertices)
    for i in range(len(edges)):
        a, b = edges[i, 0], edges[i, 1]
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        while parent[b] != b:
            parent[b] = parent[parent[b]]
            b = parent[b]
        if a != b:
            parent[a] = b
    seen = np.zeros(nvertices, np.bool_)
    n = 0
    for i in active:
        a = i
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        if not seen[a]:
            seen[a] = True
            n += 1
    return n
