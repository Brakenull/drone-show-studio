// Neighbour grid, rebuilt every step for each run (docs/3-phase-3.md §3.2, §3.4).
//
// Cubic cells of 1 / INV_CELL metres, wrapped modulo GRID_G per axis so each
// run owns a fixed table of T_CELLS cells. simulator.py checks that a query
// box never spans more than GRID_G cells, so a drone is never visited twice.
// Build: grid_count (cell + atomic slot) -> grid_scan (cell starts) ->
// grid_scatter -> grid_sort_cells. Atomic slots depend on scheduling, so each
// cell is sorted by drone index: neighbour sums then run in a fixed order and
// results are bit-for-bit reproducible.

#define G_MASK (GRID_G - 1)
#define T_CELLS (GRID_G * GRID_G * GRID_G)

inline int cell_of(float x) { return (int)floor(x * INV_CELL); }
inline int cell_key(int x, int y, int z) { return ((x & G_MASK) * GRID_G + (y & G_MASK)) * GRID_G + (z & G_MASK); }

// counts[] must be zero on entry (grid_scan re-zeroes it).
__kernel void grid_count(__global const float4* pos, __global int* counts, __global int* keys, __global int* slots) {
    int gid = get_global_id(0);
    if (gid >= RN) return;
    int r = gid / N_DRONES;
    float3 p = pos[gid].xyz;
    int key = cell_key(cell_of(p.x), cell_of(p.y), cell_of(p.z));
    keys[gid] = key;
    slots[gid] = atomic_inc(&counts[r * T_CELLS + key]);
}

// One work-group of SCAN_WG work-items per run: exclusive scan of counts into
// starts (T_CELLS + 1 entries per run), then zero counts for the next step.
__kernel void grid_scan(__global int* counts, __global int* starts) {
    __local int partial[SCAN_WG];
    int r = get_group_id(0);
    int lid = get_local_id(0);
    const int per = T_CELLS / SCAN_WG;
    __global int* c = counts + r * T_CELLS + lid * per;
    int sum = 0;
    for (int k = 0; k < per; ++k) sum += c[k];
    partial[lid] = sum;
    barrier(CLK_LOCAL_MEM_FENCE);
    for (int off = 1; off < SCAN_WG; off <<= 1) {
        int v = lid >= off ? partial[lid - off] : 0;
        barrier(CLK_LOCAL_MEM_FENCE);
        partial[lid] += v;
        barrier(CLK_LOCAL_MEM_FENCE);
    }
    int run = partial[lid] - sum;
    __global int* s = starts + r * (T_CELLS + 1) + lid * per;
    for (int k = 0; k < per; ++k) {
        int n = c[k];
        s[k] = run;
        run += n;
        c[k] = 0;
    }
    if (lid == SCAN_WG - 1) starts[r * (T_CELLS + 1) + T_CELLS] = run;
}

__kernel void grid_scatter(__global const int* keys, __global const int* slots, __global const int* starts,
                           __global int* sorted) {
    int gid = get_global_id(0);
    if (gid >= RN) return;
    int r = gid / N_DRONES;
    sorted[r * N_DRONES + starts[r * (T_CELLS + 1) + keys[gid]] + slots[gid]] = gid - r * N_DRONES;
}

__kernel void grid_sort_cells(__global const int* starts, __global int* sorted) {
    int gid = get_global_id(0);
    if (gid >= N_RUNS * T_CELLS) return;
    int r = gid / T_CELLS;
    int cell = gid - r * T_CELLS;
    __global int* list = sorted + r * N_DRONES;
    int s = starts[r * (T_CELLS + 1) + cell];
    int e = starts[r * (T_CELLS + 1) + cell + 1];
    for (int a = s + 1; a < e; ++a) {
        int v = list[a];
        int b = a - 1;
        while (b >= s && list[b] > v) { list[b + 1] = list[b]; --b; }
        list[b + 1] = v;
    }
}
