// Camera frame by hardware ray tracing: Vulkan ray queries on NVIDIA RTX, AMD
// RDNA2+, Intel Arc, or Mesa lavapipe on the CPU.
//
// Positions are metres east, north and up of the origin (the home point).
// Textures and facades follow render/shading.py, render/materials.py and
// kernels/render.cl. Change them together. rtconst is generated from the
// Python constants (rt/device.py).

import rtconst;

// ---- hashes and noise (shading.py pcg_hash, unit_hash, value_noise) ----

uint pcg(uint x)
{
    x = x * 747796405u + 2891336453u;
    x = ((x >> ((x >> 28u) + 4u)) ^ x) * 277803737u;
    return (x >> 22u) ^ x;
}

float unit_hash(uint x)
{
    return float(pcg(x) >> 8) * (1.0 / 16777216.0);
}

float gaussian(uint key)
{
    float u1 = float((pcg(key) >> 8) + 1) * (1.0 / 16777217.0);
    float u2 = float(pcg(key ^ 0x9e3779b9u) >> 8) * (1.0 / 16777216.0);
    return sqrt(-2.0 * log(u1)) * cos(6.2831853 * u2);
}

uint cell(float v)
{
    return uint(int(floor(v)));
}

float band_limit(float value, float mean, float pixels)
{
    return mean + (value - mean) * smoothstep(1.0, 2.0, pixels);
}

float value_noise(float u, float v, uint seed)
{
    float iu = floor(u), iv = floor(v);
    float wu = smoothstep(0.0, 1.0, u - iu), wv = smoothstep(0.0, 1.0, v - iv);
    uint cu = uint(int(iu)), cv = uint(int(iv));
    float c00 = unit_hash((cu * 0x8DA6B343u ^ cv * 0xD8163841u) ^ seed);
    float c10 = unit_hash(((cu + 1u) * 0x8DA6B343u ^ cv * 0xD8163841u) ^ seed);
    float c01 = unit_hash((cu * 0x8DA6B343u ^ (cv + 1u) * 0xD8163841u) ^ seed);
    float c11 = unit_hash(((cu + 1u) * 0x8DA6B343u ^ (cv + 1u) * 0xD8163841u) ^ seed);
    return lerp(lerp(c00, c10, wu), lerp(c01, c11, wu), wv);
}

// Value noise in 3D: two 2D slices, blended along z.
float value_noise3(float3 q, uint seed)
{
    float iz = floor(q.z);
    float wz = smoothstep(0.0, 1.0, q.z - iz);
    uint cz = uint(int(iz));
    return lerp(value_noise(q.x, q.y, seed ^ (cz * 0x2C1B3C6Du)),
                value_noise(q.x, q.y, seed ^ ((cz + 1u) * 0x2C1B3C6Du)), wz);
}

// Brightness factor of a wall at world position p: large soft patches, so that
// walls are not one flat colour from a distance.
float macro_variation(float3 p)
{
    float n = 0.65 * value_noise3(p / MACRO_M, 0x5BD1E995u)
              + 0.35 * value_noise3(p / MACRO_FINE_M, 0x1B873593u);
    return 1.0 + MACRO_STRENGTH * (2.0 * n - 1.0);
}

// Rain streaks run down from the wall top. Each streak column has a random
// position, width, length and strength, so no regular bands show.
float weathering(float along, float above, float top, float footprint, uint seed)
{
    float u = along / STREAK_M;
    float iu = floor(u);
    uint c = cell(u) ^ seed;
    float s = unit_hash(c);
    float strength = s * s;
    float half_w = lerp(STREAK_HALF_MIN, STREAK_HALF_MAX, unit_hash(c ^ 0x85EBCA6Bu));
    float wobble = 0.03 * (value_noise(above / 1.5, iu, seed ^ 0x27D4EB2Fu) - 0.5);
    float centre = 0.5 + (0.47 - half_w) * (2.0 * unit_hash(c ^ 0x9E3779B9u) - 1.0) + wobble;
    float below = max(top - above, 0.0);
    float run = top * lerp(STREAK_LENGTH_MIN, 1.0, unit_hash(c ^ 0xC2B2AE35u));
    float fall = 1.0 - smoothstep(0.0, run, below);
    // The streak narrows as it fades downward.
    half_w *= 0.4 + 0.6 * fall;
    float profile = 1.0 - smoothstep(0.0, 1.0, abs(u - iu - centre) / half_w);
    float fall_mean = 1.0 - smoothstep(0.0, top * STREAK_LENGTH_MEAN, below);
    float grain = 0.6 + 0.8 * value_noise(iu * 3.0, above / 0.8, seed ^ 0x165667B1u);
    // A streak narrower than two pixels becomes its column average. A column
    // narrower than two pixels becomes the mean of all columns.
    float column = strength * half_w * fall;
    float streak = lerp(column, strength * profile * fall,
                        smoothstep(1.0, 2.0, 2.0 * half_w * STREAK_M / footprint));
    streak = lerp(STREAK_MEAN * fall_mean, streak, smoothstep(1.0, 2.0, STREAK_M / footprint));
    float stain = band_limit(value_noise(along / BLOTCH_M, above / BLOTCH_M, seed), BLOTCH_MEAN,
                             BLOTCH_M / footprint);
    float splash = clamp(1.0 - above / SPLASH_M, 0.0, 1.0);
    return STREAK_DARK * streak * grain + BLOTCH_DARK * stain + SPLASH_DARK * splash;
}

float3 unpack(uint x)
{
    return float3(float(x & 255u), float((x >> 8) & 255u), float((x >> 16) & 255u));
}

// ---- photo materials (materials.py sample) ----

float3 mat_texel(StructuredBuffer<uint> atlas, int base, int n, int ix, int iy)
{
    ix = ((ix % n) + n) % n;
    iy = ((iy % n) + n) % n;
    return unpack(atlas[base + iy * n + ix]);
}

float3 mat_level(StructuredBuffer<uint> atlas, int m, int k, float fu, float fv)
{
    int n = MAT_SIZE >> k;
    int base = m * MAT_BLOCK;
    for (int j = 0; j < k; j++) base += (MAT_SIZE >> j) * (MAT_SIZE >> j);
    float x = (fu - floor(fu)) * n - 0.5, y = (fv - floor(fv)) * n - 0.5;
    float x0 = floor(x), y0 = floor(y);
    float wx = x - x0, wy = y - y0;
    int ix = int(x0), iy = int(y0);
    float3 top = lerp(mat_texel(atlas, base, n, ix, iy), mat_texel(atlas, base, n, ix + 1, iy), wx);
    float3 bottom = lerp(mat_texel(atlas, base, n, ix, iy + 1),
                         mat_texel(atlas, base, n, ix + 1, iy + 1), wx);
    return lerp(top, bottom, wy);
}

struct HexSample
{
    float3 colour;
    float3 normal;
};

// One hex-tiling sample at texture coordinates (fu, fv), turned by angle about
// centre c and moved by off. The colour is a factor about 1. The normal comes back
// in the (u, v) frame.
HexSample hex_sample(StructuredBuffer<uint> atlas, StructuredBuffer<uint> normals, int m, int k0,
                     int k1, float t, float fu, float fv, float2 c, float2 off, float angle)
{
    float cs = cos(angle), sn = sin(angle);
    float2 d = float2(fu, fv) - c;
    float2 s = float2(cs * d.x - sn * d.y, sn * d.x + cs * d.y) + c + off;
    HexSample h;
    // Divide by a coarse level at the same place: only detail finer than a cell
    // remains. Large stains differ from cell to cell and showed the cells.
    float3 local = max(mat_level(atlas, m, HEX_LOWPASS_LEVEL, s.x, s.y), float3(1.0));
    float3 fine = lerp(mat_level(atlas, m, k0, s.x, s.y), mat_level(atlas, m, k1, s.x, s.y), t);
    h.colour = k0 >= HEX_LOWPASS_LEVEL ? float3(1.0) : fine / local;
    float3 n = lerp(mat_level(normals, m, k0, s.x, s.y), mat_level(normals, m, k1, s.x, s.y), t)
               / 127.5 - 1.0;
    // Image y is up, texture y is down. Turn back by the inverse rotation.
    float2 g = float2(n.x, -n.y);
    float2 back = float2(cs * g.x + sn * g.y, -sn * g.x + cs * g.y);
    h.normal = float3(back.x, -back.y, n.z);
    return h;
}

// Hex-tiling (Mikkelsen 2022, "Practical Real-Time Hex-Tiling"). Each vertex of a
// triangle grid smaller than one tile gets a random offset, and a random angle
// when spin is set. A pixel blends the samples of its triangle, so no repeat grid shows.
float3 hex_material(StructuredBuffer<uint> atlas, StructuredBuffer<uint> normals, int m, int k0,
                    int k1, float t, float fu, float fv, bool spin, out float3 tangent_normal)
{
    float x = fu / HEX_CELL, y = fv / HEX_CELL;
    float2 skew = float2(x - y * 0.57735027, y * 1.15470054);
    float2 base = floor(skew);
    float2 f = skew - base;
    float2 v0, v1, v2;
    float3 w;
    if (f.x + f.y < 1.0) {
        v0 = base; v1 = base + float2(1.0, 0.0); v2 = base + float2(0.0, 1.0);
        w = float3(1.0 - f.x - f.y, f.x, f.y);
    } else {
        v0 = base + float2(1.0, 1.0); v1 = base + float2(0.0, 1.0); v2 = base + float2(1.0, 0.0);
        w = float3(f.x + f.y - 1.0, 1.0 - f.x, 1.0 - f.y);
    }
    float2 corners[3] = { v0, v1, v2 };
    HexSample s[3];
    uint seed = uint(m) * 0x27D4EB2Du;
    for (int i = 0; i < 3; i++) {
        float2 vi = corners[i];
        uint key = pcg((uint(int(vi.x)) * 0x8DA6B343u ^ uint(int(vi.y)) * 0xD8163841u) ^ seed);
        float2 off = float2(unit_hash(key), unit_hash(key ^ 0x68E31DA4u));
        float angle = spin ? 6.2831853 * unit_hash(key ^ 0xB5297A4Du) : 0.0;
        float2 c = float2(vi.x + 0.5 * vi.y, 0.8660254 * vi.y) * HEX_CELL;
        s[i] = hex_sample(atlas, normals, m, k0, k1, t, fu, fv, c, off, angle);
    }
    float3 luma = float3(dot(s[0].colour, float3(0.3, 0.59, 0.11)),
                         dot(s[1].colour, float3(0.3, 0.59, 0.11)),
                         dot(s[2].colour, float3(0.3, 0.59, 0.11))) * 0.5;
    float3 weight = lerp(float3(1.0), luma, HEX_CONTRAST) * pow(w, float3(HEX_SHARPNESS));
    weight /= max(weight.x + weight.y + weight.z, 1e-6);
    tangent_normal = normalize(s[0].normal * weight.x + s[1].normal * weight.y
                               + s[2].normal * weight.z);
    return s[0].colour * weight.x + s[1].colour * weight.y + s[2].colour * weight.z;
}

bool isotropic(int m)
{
    return m == MAT_CONCRETE || m == MAT_PLASTER || m == MAT_PLASTER_OLD;
}

// Colour factor and tangent-space normal (x along u, y along v up the image, z out)
// of material m, sampled with the same tiling offsets so that they line up.
// Isotropic materials use hex-tiling; spin also turns them, for surfaces without
// an up direction. Materials with courses or rows keep region tiling, because
// free offsets would break their courses.
float3 material(StructuredBuffer<uint> atlas, StructuredBuffer<uint> normals,
                StructuredBuffer<float> info, int m, float u, float v, float footprint, bool spin,
                out float3 tangent_normal)
{
    tangent_normal = float3(0.0, 0.0, 1.0);
    if (m < 0) return float3(1.0);
    float size_m = info[4 * m];
    float lod = clamp(log2(max(footprint * MAT_SIZE / size_m, 1e-6)), 0.0, MAT_LEVELS - 1.0);
    int k0 = int(floor(lod));
    int k1 = min(k0 + 1, MAT_LEVELS - 1);
    float fu = u / size_m, fv = -v / size_m;
    if (isotropic(m)) {
        return hex_material(atlas, normals, m, k0, k1, lod - k0, fu, fv, spin, tangent_normal);
    }
    // Stochastic tiling: a smooth noise picks one of TILE_VARIANTS random offsets per
    // region and blends two of them at region borders, so the texture does not repeat
    // as a grid. The blend follows the texture's own detail, which hides the seam.
    float k = value_noise(fu * TILE_NOISE, fv * TILE_NOISE, uint(m) * 0x27D4EB2Du) * TILE_VARIANTS;
    float ia = floor(k);
    float2 off_a = frac(sin(float2(3.0, 7.0) * ia) * 43758.5453);
    float2 off_b = frac(sin(float2(3.0, 7.0) * (ia + 1.0)) * 43758.5453);
    float3 a = lerp(mat_level(atlas, m, k0, fu + off_a.x, fv + off_a.y),
                    mat_level(atlas, m, k1, fu + off_a.x, fv + off_a.y), lod - k0);
    float3 b = lerp(mat_level(atlas, m, k0, fu + off_b.x, fv + off_b.y),
                    mat_level(atlas, m, k1, fu + off_b.x, fv + off_b.y), lod - k0);
    float edge = (k - ia) - 0.1 * dot(a - b, float3(1.0 / 255.0));
    float blend = smoothstep(0.2, 0.8, edge);
    float3 c = lerp(a, b, blend);
    float3 na = lerp(mat_level(normals, m, k0, fu + off_a.x, fv + off_a.y),
                     mat_level(normals, m, k1, fu + off_a.x, fv + off_a.y), lod - k0);
    float3 nb = lerp(mat_level(normals, m, k0, fu + off_b.x, fv + off_b.y),
                     mat_level(normals, m, k1, fu + off_b.x, fv + off_b.y), lod - k0);
    tangent_normal = normalize(lerp(na, nb, blend) / 127.5 - 1.0);
    return c / float3(info[4 * m + 1], info[4 * m + 2], info[4 * m + 3]);
}

// World normal of a bumpy surface from its tangent frame; fades to n with distance.
float3 bump_normal(float3 tn, float3 t, float3 b, float3 n, float footprint)
{
    float w = 1.0 - smoothstep(BUMP_NEAR_M, BUMP_FAR_M, footprint);
    if (w <= 0.0) return n;
    float3 strong = normalize(float3(tn.xy * BUMP_STRENGTH, tn.z));
    return normalize(lerp(n, normalize(t * strong.x + b * strong.y + n * strong.z), w));
}

// ---- facades (shading.py facade, window_glass, classic_detail) ----

// Glass colour: room or curtain, sky reflection. The recess shadow comes from window_recess.
float3 window_glass(float along, float above, float floor_m, float bay_m, uint seed,
                    float cos_view, float3 sky)
{
    uint key = cell(along / bay_m) * 0x8DA6B343u ^ cell(above / floor_m) * 0xD8163841u;
    float room = unit_hash(key ^ seed);
    float curtain = unit_hash(key ^ seed ^ 0x5BD1E995u) > CURTAIN_SHARE ? CURTAIN_MIX : 0.0;
    float3 g = GLASS * (ROOM_DARK + ROOM_SPREAD * room);
    g = lerp(g, CURTAIN, curtain);
    float grazing = 1.0 - clamp(cos_view, 0.0, 1.0);
    grazing = grazing * grazing * grazing * grazing * grazing;
    return lerp(g, sky, REFLECT_MIN + (1.0 - REFLECT_MIN) * grazing);
}

// A window opening is a box of the layout depth behind the wall plane. The view ray
// goes on into the box and meets the glass or a reveal; the sun lights that point only
// if its way back out passes through the opening. Returns false outside an opening.
struct Recess
{
    bool glass;
    float3 normal;
    float lit;
    float along, above;
};

Recess window_recess(float3 r, float3 n, float3 sun, float along, float above, float s_lo,
                     float s_hi, float t_lo, float t_hi, float depth)
{
    float3 t_dir = float3(-n.y, n.x, 0.0);
    float ds = dot(r, t_dir), dt = r.z, dw = -dot(r, n);
    float lg = dw > 1e-4 ? depth / dw : 1e9;
    float ls = ds > 1e-6 ? (s_hi - along) / ds : (ds < -1e-6 ? (s_lo - along) / ds : 1e9);
    float lt = dt > 1e-6 ? (t_hi - above) / dt : (dt < -1e-6 ? (t_lo - above) / dt : 1e9);
    Recess hit;
    float l;
    if (lg <= min(ls, lt)) {
        hit.glass = true;
        hit.normal = n;
        l = lg;
    } else if (ls < lt) {
        hit.glass = false;
        hit.normal = ds > 0.0 ? -t_dir : t_dir;
        l = ls;
    } else {
        hit.glass = false;
        hit.normal = dt > 0.0 ? float3(0.0, 0.0, -1.0) : float3(0.0, 0.0, 1.0);
        l = lt;
    }
    hit.along = along + ds * l;
    hit.above = above + dt * l;
    float w = dw * l;
    float sw = -dot(sun, n);
    hit.lit = 0.0;
    if (sw < -1e-4) {
        float out_l = w / -sw;
        float se = hit.along + dot(sun, t_dir) * out_l, te = hit.above + sun.z * out_l;
        hit.lit = se > s_lo && se < s_hi && te > t_lo && te < t_hi ? 1.0 : 0.0;
    }
    return hit;
}

float trim_fade(float width, float footprint)
{
    return smoothstep(TRIM_PIXELS, 2.0 * TRIM_PIXELS, width / footprint);
}

void classic_detail(inout float3 wall, inout float3 glass, bool window, bool zone, float floor_,
                    float bay, float above, float footprint, float floor_m, float bay_m, float f0,
                    float f1, float b0, float b1, float top)
{
    bool ground = above < floor_m;
    float3 w = wall * (ground ? GROUND_SHADE : 1.0);
    float groove = fmod(above, GROOVE_M) < GROOVE_LINE_M ? 1.0 : 0.0;
    groove = band_limit(groove, GROOVE_LINE_M / GROOVE_M, GROOVE_LINE_M / footprint);
    w *= 1.0 - GROOVE_DARK * groove * (ground ? 1.0 : 0.0);
    float cornice = floor_ > 1.0 - CORNICE_M / floor_m ? 1.0 : 0.0;
    cornice = band_limit(cornice, CORNICE_M / floor_m, CORNICE_M / footprint) * (ground ? 0.0 : 1.0);
    w = lerp(w, TRIM, CORNICE_MIX * cornice);
    // No painted ledge shadow: cornice_light casts the real one.
    float fu = FRAME_M / floor_m, fb = FRAME_M / bay_m;
    bool frame = zone && !window && floor_ > f0 - fu && floor_ < f1 + fu && bay > b0 - fb && bay < b1 + fb;
    w = lerp(w, TRIM, (frame ? 1.0 : 0.0) * trim_fade(FRAME_M, footprint));
    float middle = above - (floor_ - 0.5 * (f0 + f1)) * floor_m;
    bool row = middle > WINDOW_BOTTOM_M && middle < top - WINDOW_TOP_GAP_M;
    float over = (FRAME_M + OVER_M) / bay_m;
    bool wide = row && bay > b0 - over && bay < b1 + over;
    float sill_top = f0 - fu, sill_low = sill_top - SILL_M / floor_m;
    bool sill = wide && floor_ > sill_low && floor_ <= sill_top;
    bool drop = wide && floor_ > sill_low - SILL_SHADOW_M / floor_m && floor_ <= sill_low;
    bool hood = wide && floor_ >= f1 + fu && floor_ < f1 + fu + HOOD_M / floor_m;
    w = lerp(w, TRIM, (sill ? 1.0 : 0.0) * trim_fade(SILL_M, footprint));
    w = lerp(w, TRIM, (hood ? 1.0 : 0.0) * trim_fade(HOOD_M, footprint));
    w *= 1.0 - SILL_DARK * (drop ? 1.0 : 0.0) * trim_fade(SILL_SHADOW_M, footprint);
    float transom = f0 + TRANSOM * (f1 - f0);
    bool cross = abs(bay - 0.5 * (b0 + b1)) * bay_m < 0.5 * MULLION_M
                 || abs(floor_ - transom) * floor_m < 0.5 * MULLION_M;
    glass = lerp(glass, TRIM, (cross ? 1.0 : 0.0) * trim_fade(MULLION_M, footprint));
    wall = w;
}

// Cornices stick out of classic facades: a band below every floor line and the
// eaves cornice under the roof. They are boxes in front of the wall plane, traced
// here from the wall hit back along the view ray. above and top are metres over
// the wall foot; the boxes span [line - height, line] and depth metres out.
struct Band
{
    bool hit;
    float3 normal;
    float lit;
};

float band_lit(float3 normal, float3 sun)
{
    return dot(normal, sun) > 0.0 ? 1.0 : 0.0;
}

// The ray part in front of the wall, from w = -depth to the wall, against one box.
bool band_box(float t_front, float rz, float dw, float slab, float tb, float tt, float depth,
              out float mu, out bool front)
{
    mu = 0.0;
    front = false;
    float end = slab / dw;
    float mu_w = (slab - depth) / dw;
    float lo = 0.0, hi = end;
    if (abs(rz) < 1e-6) {
        if (t_front < tb || t_front > tt) return false;
    } else {
        float a = (tb - t_front) / rz, b = (tt - t_front) / rz;
        lo = max(lo, min(a, b));
        hi = min(hi, max(a, b));
    }
    mu = max(lo, mu_w);
    front = mu_w >= lo;
    return mu <= hi;
}

Band cornice_hit(float3 r, float3 n, float3 sun, float above, float top, float floor_m)
{
    Band band;
    band.hit = false;
    band.normal = n;
    band.lit = 1.0;
    float dw = -dot(r, n);
    if (dw < 1e-4) return band;
    float slab = EAVES_DEPTH_M;
    float lam = slab / dw;
    float t_front = above - r.z * lam;
    float best = 1e9;
    bool best_front = false, found = false;
    float mu;
    bool front;
    if (band_box(t_front, r.z, dw, slab, top - EAVES_M, top, EAVES_DEPTH_M, mu, front)) {
        best = mu; best_front = front; found = true;
    }
    float t_lo = min(above, t_front), t_hi = max(above, t_front);
    for (float line = max(1.0, floor(t_lo / floor_m)) * floor_m; line <= t_hi + CORNICE_M; line += floor_m) {
        if (line > top - EAVES_M) break;
        if (band_box(t_front, r.z, dw, slab, line - CORNICE_M, line, CORNICE_DEPTH_M, mu, front)
            && mu < best) {
            best = mu; best_front = front; found = true;
        }
    }
    if (!found) return band;
    band.hit = true;
    band.normal = best_front ? n : (r.z < 0.0 ? float3(0.0, 0.0, 1.0) : float3(0.0, 0.0, -1.0));
    band.lit = band_lit(band.normal, sun);
    return band;
}

// Sun on a wall point below the cornices: 0 where a band shades it.
float cornice_light(float3 n, float3 sun, float above, float top, float floor_m)
{
    float sw = -dot(sun, n);
    if (sw >= -1e-4 || sun.z <= 1e-4) return 1.0;
    // Height the sun ray climbs while it crosses a band of this depth.
    float rise_eaves = sun.z * EAVES_DEPTH_M / -sw;
    if (above < top - EAVES_M && above + rise_eaves > top - EAVES_M) return 0.0;
    float line = ceil(above / floor_m) * floor_m;
    if (line < floor_m || line > top - EAVES_M) return 1.0;
    float rise = sun.z * CORNICE_DEPTH_M / -sw;
    return above < line - CORNICE_M && above + rise > line - CORNICE_M ? 0.0 : 1.0;
}

// Where a point falls in the window grid of its style.
struct WindowCell
{
    float floor_m, bay_m, f0, f1, b0, b1;
    float floor_, bay;
    bool zone, window;
    // The opening at this point: along and above range, depth behind the wall.
    float s_lo, s_hi, t_lo, t_hi, depth;
    // A door leaf instead of glass; a shop sign band.
    bool door, sign;
    // Window detail weight: 0 when a floor covers too few pixels, then only the average shows.
    float fade;
};

WindowCell window_cell(StructuredBuffer<float> styles, int style_count, StructuredBuffer<float> layouts,
                       float style, float along, float above, float top, float footprint, float storey,
                       float3 feature)
{
    WindowCell c;
    int index = clamp(int(style) - 1, 0, style_count - 1);
    int layout = LAYOUT_COLUMNS * int(styles[6 * index + 3]);
    // The tagged storey height wins; the window fractions scale with it.
    c.floor_m = storey > 0.0 ? storey : layouts[layout];
    c.bay_m = layouts[layout + 1];
    c.f0 = layouts[layout + 2];
    c.f1 = layouts[layout + 3];
    c.b0 = layouts[layout + 4];
    c.b1 = layouts[layout + 5];
    c.floor_ = fmod(above, c.floor_m) / c.floor_m;
    c.bay = fmod(along, c.bay_m) / c.bay_m;
    if (c.floor_ < 0.0) c.floor_ += 1.0;
    if (c.bay < 0.0) c.bay += 1.0;
    c.zone = above > WINDOW_BOTTOM_M && above < top - WINDOW_TOP_GAP_M;
    c.window = c.floor_ > c.f0 && c.floor_ < c.f1 && c.bay > c.b0 && c.bay < c.b1 && c.zone;
    c.fade = smoothstep(WINDOW_FADE_LO, WINDOW_FADE_HI, c.floor_m / footprint);
    float s0 = floor(along / c.bay_m) * c.bay_m, t0 = floor(above / c.floor_m) * c.floor_m;
    c.s_lo = s0 + c.b0 * c.bay_m;
    c.s_hi = s0 + c.b1 * c.bay_m;
    c.t_lo = t0 + c.f0 * c.floor_m;
    c.t_hi = t0 + c.f1 * c.floor_m;
    c.depth = layouts[layout + 6];
    c.door = false;
    c.sign = false;
    // OSM features replace the ground floor windows; feature = (kind, from, to) along the wall.
    bool ground = above >= 0.0 && above < c.floor_m;
    if (ground && feature.x == FEATURE_DOOR) {
        float door_top = min(DOOR_HEIGHT_M, c.floor_m - WINDOW_TOP_GAP_M);
        c.zone = false;
        c.window = false;
        c.door = above < door_top;
        c.s_lo = feature.y;
        c.s_hi = feature.z;
        c.t_lo = 0.0;
        c.t_hi = door_top;
        c.depth = DOOR_DEPTH_M;
    } else if (ground && feature.x == FEATURE_SHOP) {
        float sign_hi = c.floor_m - WINDOW_TOP_GAP_M * 0.25, sign_lo = sign_hi - SHOP_SIGN_M;
        c.zone = false;
        c.sign = above > sign_lo && above < sign_hi;
        c.s_lo = s0 + SHOP_PILLAR_M;
        c.s_hi = s0 + c.bay_m - SHOP_PILLAR_M;
        c.t_lo = SHOP_SILL_M;
        c.t_hi = sign_lo - FRAME_M;
        c.window = along > c.s_lo && along < c.s_hi && above > c.t_lo && above < c.t_hi;
        c.depth = SHOP_DEPTH_M;
    }
    return c;
}

// Sign colours of shopfronts, one picked per shop wall.
static const float3 SIGN_COLOURS[4] = { float3(38.0, 74.0, 52.0), float3(128.0, 34.0, 32.0),
                                        float3(30.0, 44.0, 86.0), float3(36.0, 36.0, 40.0) };

// facade() uses the mirror colour only on glass and in the faded average.
bool wants_mirror(WindowCell c)
{
    return c.window || c.fade < 1.0;
}

float3 facade(StructuredBuffer<float> styles, int style_count, StructuredBuffer<float> layouts,
              StructuredBuffer<uint> atlas, StructuredBuffer<uint> normals,
              StructuredBuffer<float> mat_info, float style,
              float along, float above, float top, float storey, float3 feature, float footprint,
              float cos_view, float3 sky, float macro,
              float3 r, float3 n, float3 sun, out float3 light_normal, out float light_lit,
              out float3 bump)
{
    light_normal = n;
    light_lit = 1.0;
    bump = n;
    int index = clamp(int(style) - 1, 0, style_count - 1);
    int row = 6 * index;
    int layout = LAYOUT_COLUMNS * int(styles[row + 3]);
    WindowCell win = window_cell(styles, style_count, layouts, style, along, above, top, footprint, storey,
                                  feature);
    float floor_m = win.floor_m, bay_m = win.bay_m;
    float f0 = win.f0, f1 = win.f1, b0 = win.b0, b1 = win.b1;
    uint seed = uint(index) * 0x9E3779B9u;
    float dirt = styles[row + 4] * weathering(along, above, top, footprint, seed);
    float3 tangent_normal;
    int mat = int(styles[row + 5]);
    float3 texture = material(atlas, normals, mat_info, mat, along, above, footprint, false,
                              tangent_normal);
    // At mid distances the fine sample averages out to flat colour. A sample at a
    // larger scale keeps structure there. Courses would grow with it, so only
    // isotropic materials get it. The LOD bias blurs small marks such as tie
    // holes, which would grow to large dots at that scale.
    float coarse_w = smoothstep(DISTANCE_NEAR_M, DISTANCE_FAR_M, footprint);
    if (coarse_w > 0.0 && isotropic(mat)) {
        float3 coarse_normal;
        float3 coarse = material(atlas, normals, mat_info, mat, along / DISTANCE_SCALE,
                                 above / DISTANCE_SCALE,
                                 footprint * exp2(DISTANCE_LOD_BIAS) / DISTANCE_SCALE, false,
                                 coarse_normal);
        texture = lerp(texture, coarse, coarse_w);
    }
    bump = bump_normal(tangent_normal, float3(-n.y, n.x, 0.0), float3(0.0, 0.0, 1.0), n, footprint);
    float3 colour = float3(styles[row], styles[row + 1], styles[row + 2]) * texture * (1.0 - dirt) * macro;
    float floor_ = win.floor_, bay = win.bay;
    float3 glass = window_glass(along, above, floor_m, bay_m, seed, cos_view, sky);
    bool zone = win.zone, window = win.window;
    if (int(styles[row + 3]) == WINDOWS_CLASSIC) {
        // A shop pane is one sheet of glass, without the window cross.
        float3 pane = glass;
        classic_detail(colour, glass, window, zone, floor_, bay, above, footprint, floor_m, bay_m,
                       f0, f1, b0, b1, top);
        if (feature.x == FEATURE_SHOP && above < floor_m) glass = pane;
    }
    uint shop = cell(feature.y) ^ seed;
    float3 leaf = DOOR_LEAF * (0.8 + 0.4 * unit_hash(shop));
    float3 pattern = window ? glass : win.door ? leaf : colour;
    if (win.sign) pattern = SIGN_COLOURS[min(int(unit_hash(shop ^ 0x68E31DA4u) * 4.0), 3)];
    float share = (f1 - f0) * (b1 - b0);
    float3 average = colour * (1.0 - share) + glass * share;
    float fade = win.fade;
    float depth = win.depth;
    if ((window || win.door) && depth > 0.0 && fade > 0.0) {
        Recess hit = window_recess(r, n, sun, along, above, win.s_lo, win.s_hi, win.t_lo, win.t_hi, depth);
        pattern = hit.glass ? (win.door ? leaf : glass) : colour * REVEAL_SHADE;
        bump = light_normal;
        light_normal = normalize(lerp(n, hit.normal, fade));
        bump = light_normal;
        light_lit = lerp(1.0, hit.lit, fade);
    }
    int kind = int(styles[row + 3]);
    if ((kind == WINDOWS_CLASSIC || kind == WINDOWS_HISTORIC) && fade > 0.0) {
        Band band = cornice_hit(r, n, sun, above, top, floor_m);
        if (band.hit) {
            pattern = TRIM * texture;
            light_normal = normalize(lerp(n, band.normal, fade));
            bump = light_normal;
            light_lit = lerp(1.0, band.lit, fade);
        } else {
            light_lit *= lerp(1.0, cornice_light(n, sun, above, top, floor_m), fade);
        }
    }
    return lerp(average, pattern, fade);
}

// ---- orthophoto levels (imagery.py sample) ----

struct Levels
{
    StructuredBuffer<uint> rgb0, rgb1, rgb2, rgb3;
};

// Per level k, three float4 at 3k: (h, w, -, -), (cx, cy, kx, ky1), (ky2, ky3, zoom, -).
struct LevelMaps
{
    StructuredBuffer<float4> params;
    int count;
};

uint level_texel(Levels lv, int k, int i)
{
    switch (k)
    {
    case 0: return lv.rgb0[i];
    case 1: return lv.rgb1[i];
    case 2: return lv.rgb2[i];
    default: return lv.rgb3[i];
    }
}

bool sample_level(Levels lv, LevelMaps m, int k, float east, float north, out float3 rgb)
{
    rgb = float3(0.0);
    int h = int(m.params[3 * k].x), w = int(m.params[3 * k].y);
    if (h == 0) return false;
    float4 a = m.params[3 * k + 1], b = m.params[3 * k + 2];
    float lx = a.x + east * a.z;
    float ly = a.y + north * (a.w + north * (b.x + north * b.y));
    if (lx < 0.0 || ly < 0.0 || lx > w - 1 || ly > h - 1) return false;
    lx = min(lx, w - 1.001);
    ly = min(ly, h - 1.001);
    int ix = int(lx), iy = int(ly);
    uint p00 = level_texel(lv, k, iy * w + ix), p01 = level_texel(lv, k, iy * w + ix + 1);
    uint p10 = level_texel(lv, k, (iy + 1) * w + ix), p11 = level_texel(lv, k, (iy + 1) * w + ix + 1);
    if ((p00 >> 24) == 0u || (p11 >> 24) == 0u) return false;
    float fx = lx - ix, fy = ly - iy;
    rgb = lerp(lerp(unpack(p00), unpack(p01), fx), lerp(unpack(p10), unpack(p11), fx), fy);
    return true;
}

float3 texture_at(Levels lv, LevelMaps m, float zoom, float east, float north)
{
    int k = 0;
    while (k + 1 < m.count && m.params[3 * (k + 1) + 2].z >= zoom) k++;
    float3 a = float3(0.0), b = float3(0.0);
    bool has_a = sample_level(lv, m, k, east, north, a);
    bool has_b = false;
    float t = 0.0;
    float zk = m.params[3 * k + 2].z;
    if (k + 1 < m.count && zoom < zk) {
        has_b = sample_level(lv, m, k + 1, east, north, b);
        t = clamp((zk - zoom) / (zk - m.params[3 * (k + 1) + 2].z), 0.0, 1.0);
    }
    if (has_a && has_b) return lerp(a, b, t);
    if (has_a) return a;
    if (has_b) return b;
    for (int j = k + 2; j < m.count; j++) {
        if (sample_level(lv, m, j, east, north, a)) return a;
    }
    return NO_DATA;
}

// ---- kernels ----

float3 view_ray(StructuredBuffer<float> base, uint i, float3 rot0, float3 rot1, float3 rot2)
{
    return base[3 * i] * rot0 + base[3 * i + 1] * rot1 + base[3 * i + 2] * rot2;
}

[shader("compute")]
[numthreads(64, 1, 1)]
void trace(uint3 tid: SV_DispatchThreadID, uniform RaytracingAccelerationStructure tlas,
           uniform StructuredBuffer<float> base, uniform float3 rot0, uniform float3 rot1,
           uniform float3 rot2, uniform float3 cam, uniform float reach, uniform uint count,
           uniform RWStructuredBuffer<float> out_t, uniform RWStructuredBuffer<uint> out_id,
           uniform RWStructuredBuffer<float> out_bary)
{
    uint i = tid.x;
    if (i >= count) return;
    RayDesc ray;
    ray.Origin = cam;
    ray.Direction = view_ray(base, i, rot0, rot1, rot2);
    ray.TMin = 0.0;
    ray.TMax = reach;
    RayQuery<RAY_FLAG_FORCE_OPAQUE> q;
    q.TraceRayInline(tlas, RAY_FLAG_NONE, 0xff, ray);
    q.Proceed();
    bool hit = q.CommittedStatus() == COMMITTED_TRIANGLE_HIT;
    out_t[i] = hit ? q.CommittedRayT() : -1.0;
    out_id[i] = hit ? q.CommittedInstanceID() + q.CommittedPrimitiveIndex() : 0xffffffffu;
    float2 bary = hit ? q.CommittedTriangleBarycentrics() : float2(0.0);
    out_bary[2 * i] = bary.x;
    out_bary[2 * i + 1] = bary.y;
}

float3 unpack_normal(float slot)
{
    uint u = asuint(slot);
    float x = f16tof32(u & 0xFFFFu), y = f16tof32(u >> 16);
    return float3(x, y, sqrt(max(0.0, 1.0 - x * x - y * y)));
}

float sun_visible(RaytracingAccelerationStructure tlas, float3 p, float3 n, float3 sun, float d)
{
    if (d > SHADOW_MAX_M) return 1.0;
    RayDesc ray;
    ray.Origin = p + n * SHADOW_LIFT_M;
    ray.Direction = sun;
    ray.TMin = 0.0;
    ray.TMax = SHADOW_RANGE_M;
    RayQuery<RAY_FLAG_FORCE_OPAQUE | RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH> q;
    q.TraceRayInline(tlas, RAY_FLAG_NONE, 0xff, ray);
    q.Proceed();
    return q.CommittedStatus() == COMMITTED_TRIANGLE_HIT ? 0.0 : 1.0;
}

// Sky visibility at p: 1 open, lower where nearby geometry blocks the sky.
float ambient_occlusion(RaytracingAccelerationStructure tlas, float3 p, float3 n, float d)
{
    if (d > AO_MAX_M) return 1.0;
    // A frame around n; the rotation comes from the world position, so the pattern
    // stays on the surface while the camera moves.
    float3 a = abs(n.z) < 0.9 ? float3(0.0, 0.0, 1.0) : float3(1.0, 0.0, 0.0);
    float3 tx = normalize(cross(a, n)), ty = cross(n, tx);
    uint3 q = uint3(int3(floor(p * 8.0)));
    float spin = unit_hash(q.x * 0x8DA6B343u ^ q.y * 0xD8163841u ^ q.z * 0xCB1AB31Fu) * 6.2831853;
    float blocked = 0.0;
    for (int k = 0; k < AO_RAYS; k++) {
        // Cosine-weighted directions on a spiral.
        float u = (k + 0.5) / AO_RAYS;
        float r = sqrt(u), phi = spin + k * 2.3999632;
        float3 dir = normalize(tx * (r * cos(phi)) + ty * (r * sin(phi)) + n * sqrt(1.0 - u));
        RayDesc ray;
        ray.Origin = p + n * SHADOW_LIFT_M;
        ray.Direction = dir;
        ray.TMin = 0.0;
        ray.TMax = AO_RANGE_M;
        RayQuery<RAY_FLAG_FORCE_OPAQUE> rq;
        rq.TraceRayInline(tlas, RAY_FLAG_NONE, 0xff, ray);
        rq.Proceed();
        if (rq.CommittedStatus() == COMMITTED_TRIANGLE_HIT)
            blocked += 1.0 - rq.CommittedRayT() / AO_RANGE_M;
    }
    float fade = 1.0 - smoothstep(AO_FADE_M, AO_MAX_M, d);
    return 1.0 - AO_STRENGTH * fade * blocked / AO_RAYS;
}

// Brightness structure of a roof material for close views of a blurry orthophoto.
// Pitched roofs get tiles with their columns down the slope; flat roofs concrete.
float roof_detail(StructuredBuffer<uint> atlas, StructuredBuffer<uint> normals,
                  StructuredBuffer<float> mat_info, float3 p, float3 n, float footprint,
                  out float3 bump)
{
    bump = n;
    float w = ROOF_DETAIL * (1.0 - smoothstep(ROOF_DETAIL_NEAR_M, ROOF_DETAIL_FAR_M, footprint));
    if (w <= 0.0) return 1.0;
    float2 down = n.xy;
    float slope = length(down);
    float3 f;
    float3 tn;
    if (slope < 0.25) {
        f = material(atlas, normals, mat_info, MAT_CONCRETE, p.x, p.y, footprint, true, tn);
        bump = bump_normal(tn, float3(1.0, 0.0, 0.0), float3(0.0, 1.0, 0.0), n, footprint);
    } else {
        down /= slope;
        float u = dot(p.xy, float2(-down.y, down.x));
        float v = dot(p.xy, down) * n.z - p.z * slope;
        f = material(atlas, normals, mat_info, MAT_ROOF_TILES, u, v, footprint, false, tn);
        float3 b = normalize(float3(down.x, down.y, -slope / max(n.z, 1e-3)));
        bump = bump_normal(tn, float3(-down.y, down.x, 0.0), b, n, footprint);
    }
    float luminance = dot(f, float3(0.3, 0.59, 0.11));
    return lerp(1.0, luminance, w);
}

struct Camera
{
    float3 rot0;
    float3 rot1;
    float3 rot2;
    float3 cam;
    float reach;
    int width, height;
    float pixel_angle, texel0;
    int style_count;
    int level_count;
};

struct Light
{
    float3 sun;
    float day, flat;
    int shadows;
    float3 horizon, zenith;
    float low, sin_az, cos_az;
    float3 gain;
    float sigma;
    uint seed;
};

float3 sky_colour(float3 dir, Light light)
{
    float alt = sqrt(clamp(dir.z, 0.0, 1.0));
    float3 c = light.horizon * (1.0 - alt) + light.zenith * alt;
    float toward = clamp(dir.x * light.sin_az + dir.y * light.cos_az, 0.0, 1.0);
    float t4 = toward * toward * toward * toward;
    return c + float3(60.0 * light.low, 30.0 * light.low, 0.0) * t4;
}

// What a window at p sees in the mirror direction: a facade (its style colour, lit
// by the sun), the ground or a roof (orthophoto), or the sky. No shadows, no windows:
// the glass shows it dimmed by the Fresnel share only.
float3 reflection(RaytracingAccelerationStructure tlas, StructuredBuffer<float4> records,
                  Levels levels, LevelMaps maps, StructuredBuffer<float> styles, int style_count,
                  float3 p, float3 n, float3 r, Light light, float texel0, float pixel_angle,
                  float d)
{
    float3 dir = r - 2.0 * dot(r, n) * n;
    RayDesc ray;
    ray.Origin = p + n * SHADOW_LIFT_M;
    ray.Direction = dir;
    ray.TMin = 0.0;
    ray.TMax = REFLECT_RANGE_M;
    RayQuery<RAY_FLAG_FORCE_OPAQUE> q;
    q.TraceRayInline(tlas, RAY_FLAG_NONE, 0xff, ray);
    q.Proceed();
    if (q.CommittedStatus() != COMMITTED_TRIANGLE_HIT) return sky_colour(dir, light);
    float t = q.CommittedRayT();
    uint id = q.CommittedInstanceID() + q.CommittedPrimitiveIndex();
    float4 ra = records[3 * id], rb = records[3 * id + 1];
    float3 hit_n = float3(ra.z, ra.w, rb.x);
    float3 h = ray.Origin + dir * t;
    float3 c;
    float3 shade_n = hit_n;
    if (ra.x == KIND_WALL) {
        int index = clamp(int(ra.y) - 1, 0, style_count - 1);
        c = float3(styles[6 * index], styles[6 * index + 1], styles[6 * index + 2]);
    } else if (ra.x == KIND_DETAIL) {
        c = records[3 * id + 2].xyz;
        if (dot(shade_n, dir) > 0.0) shade_n = -shade_n;
    } else {
        float footprint = (d + t) * pixel_angle;
        c = texture_at(levels, maps, log2(texel0 / max(footprint, 1e-3)), h.x, h.y);
        if (ra.x == KIND_ROOF) shade_n = float3(0.0, 0.0, 1.0);
    }
    if (light.day > 0.0) {
        float facing = clamp(dot(shade_n, light.sun), 0.0, 1.0);
        c *= 1.0 + light.day * ((AMBIENT + DIRECT * facing) / light.flat - 1.0);
    }
    float haze = (1.0 - exp(-t / HAZE_DISTANCE_M)) * HAZE_MAX;
    return c * (1.0 - haze) + light.horizon * haze;
}

[shader("compute")]
[numthreads(64, 1, 1)]
void shade(uint3 tid: SV_DispatchThreadID, uniform RaytracingAccelerationStructure tlas,
           uniform StructuredBuffer<float> base, uniform StructuredBuffer<float> dist,
           uniform StructuredBuffer<uint> ids, uniform StructuredBuffer<float4> records,
           uniform StructuredBuffer<float> bary,
           uniform Levels levels, uniform StructuredBuffer<float> styles,
           uniform StructuredBuffer<float> layouts, uniform StructuredBuffer<uint> atlas,
           uniform StructuredBuffer<uint> normals,
           uniform StructuredBuffer<float> mat_info, uniform ConstantBuffer<Camera> view,
           uniform StructuredBuffer<float4> level_params, uniform ConstantBuffer<Light> light,
           uniform RWStructuredBuffer<uint> out_rgb)
{
    float3 rot0 = view.rot0, rot1 = view.rot1, rot2 = view.rot2, cam = view.cam;
    float reach = view.reach, pixel_angle = view.pixel_angle, texel0 = view.texel0;
    int width = view.width, height = view.height, style_count = view.style_count;
    LevelMaps maps = { level_params, view.level_count };
    uint i = tid.x;
    if (i >= uint(width * height)) return;
    float3 r = view_ray(base, i, rot0, rot1, rot2);
    float raw = dist[i];
    bool hit = raw >= 0.0;
    float d = hit ? raw : reach;
    float3 color = float3(0.0);
    float3 nrm = float3(0.0, 0.0, 1.0);
    float lit = 1.0;
    float lit_scale = 1.0;
    if (hit) {
        uint id = ids[i];
        float4 ra = records[3 * id], rb = records[3 * id + 1], rc = records[3 * id + 2];
        float kind = ra.x;
        nrm = float3(ra.z, ra.w, rb.x);
        // A balcony parapet is one sheet: light the side the ray sees.
        if (kind == KIND_DETAIL && dot(nrm, r) > 0.0) nrm = -nrm;
        float3 face = nrm;
        float3 bump = nrm, relief_base = nrm;
        if (kind == KIND_TERRAIN) {
            float u = bary[2 * i], v = bary[2 * i + 1];
            nrm = normalize(unpack_normal(rc.x) * (1.0 - u - v) + unpack_normal(rc.y) * u
                            + unpack_normal(rc.z) * v);
        }
        float3 p = cam + r * d;
        float cos_view = abs(dot(r, nrm));
        float footprint = d * pixel_angle / max(cos_view, MIN_COS);
        if (kind == KIND_WALL) {
            float along = -nrm.y * p.x + nrm.x * p.y + rb.y;
            float3 light_normal;
            float recess_lit;
            float above = p.z - rb.z, top = rb.w - rb.z;
            // Windows mirror what is in front of them; far away they fade out anyway.
            // Plain wall shows no mirror, so it traces no ray.
            WindowCell cell = window_cell(styles, style_count, layouts, ra.y, along, above, top, footprint, rc.w,
                                          rc.xyz);
            float3 mirror = footprint < REFLECT_MAX_FOOTPRINT_M && wants_mirror(cell)
                ? reflection(tlas, records, levels, maps, styles, style_count, p, nrm, r, light,
                             texel0, pixel_angle, d)
                : light.horizon;
            color = facade(styles, style_count, layouts, atlas, normals, mat_info, ra.y, along,
                           above, top, rc.w, rc.xyz, footprint, cos_view, mirror,
                           macro_variation(p), r, nrm,
                           light.sun, light_normal, recess_lit, bump);
            relief_base = light_normal;
            nrm = light_normal;
            lit_scale = recess_lit;
        } else if (kind == KIND_DETAIL) {
            color = rc.xyz;
        } else {
            color = texture_at(levels, maps, log2(texel0 / footprint), p.x, p.y);
            if (kind == KIND_ROOF)
                color *= roof_detail(atlas, normals, mat_info, p, nrm, footprint, bump);
        }
        if (light.shadows != 0) lit = sun_visible(tlas, p, face, light.sun, d);
        lit *= lit_scale;
        float sky_light = light.day > 0.0 ? ambient_occlusion(tlas, p, face, d) : 1.0;
        if (light.day > 0.0) {
            // The orthophoto on a roof already holds the real roof light; shading its
            // planes again gave light and dark triangles. Cast shadows still count.
            float3 shade_n = kind == KIND_ROOF ? float3(0.0, 0.0, 1.0) : nrm;
            float facing = clamp(dot(shade_n, light.sun), 0.0, 1.0);
            color *= 1.0 + light.day * ((AMBIENT * sky_light + DIRECT * facing * lit) / light.flat - 1.0);
            // Material relief: our sun on the bumpy surface against the smooth one.
            float smooth_facing = clamp(dot(relief_base, light.sun), 0.0, 1.0);
            float bumpy_facing = clamp(dot(bump, light.sun), 0.0, 1.0);
            float relief = (AMBIENT * sky_light + DIRECT * bumpy_facing * lit)
                           / max(AMBIENT * sky_light + DIRECT * smooth_facing * lit, 1e-3);
            color *= lerp(1.0, relief, light.day);
        }
    }

    float haze = (1.0 - exp(-d / HAZE_DISTANCE_M)) * HAZE_MAX;
    color = color * (1.0 - haze) + light.horizon * haze;
    if (!hit) {
        float alt = sqrt(clamp(r.z, 0.0, 1.0));
        color = light.horizon * (1.0 - alt) + light.zenith * alt;
        float toward = clamp(r.x * light.sin_az + r.y * light.cos_az, 0.0, 1.0);
        float t4 = toward * toward * toward * toward;
        color += float3(60.0 * light.low, 30.0 * light.low, 0.0) * t4;
    }

    int x = int(i) % width, y = int(i) / width;
    float dx = x - width / 2.0, dy = y - height / 2.0;
    float r2 = (dx * dx + dy * dy) / ((width / 2.0) * (width / 2.0) + (height / 2.0) * (height / 2.0));
    color *= light.gain * (1.0 - VIGNETTE_STRENGTH * r2);
    uint key = i * 3u + light.seed;
    color += float3(gaussian(key), gaussian(key + 1u), gaussian(key + 2u)) * light.sigma;
    uint3 c = uint3(clamp(color, 0.0, 255.0));
    out_rgb[i] = c.x | (c.y << 8) | (c.z << 16) | 0xFF000000u;
}

// Every stride-th distance in both directions; the host reads only these.
[shader("compute")]
[numthreads(64, 1, 1)]
void subsample(uint3 tid: SV_DispatchThreadID, uniform StructuredBuffer<float> src,
               uniform int src_width, uniform int stride, uniform int width, uniform int height,
               uniform RWStructuredBuffer<float> dst)
{
    int i = int(tid.x);
    if (i >= width * height) return;
    int x = i % width, y = i / width;
    dst[i] = src[(y * stride) * src_width + x * stride];
}

// Supersampling: average each s x s block into one output pixel.
[shader("compute")]
[numthreads(64, 1, 1)]
void downsample(uint3 tid: SV_DispatchThreadID, uniform StructuredBuffer<uint> src,
                uniform int src_width, uniform int s, uniform int width, uniform int height,
                uniform RWStructuredBuffer<uint> dst)
{
    int i = int(tid.x);
    if (i >= width * height) return;
    int x = i % width, y = i / width;
    float3 sum = float3(0.0);
    for (int dy = 0; dy < s; dy++)
        for (int dx = 0; dx < s; dx++)
            sum += unpack(src[(y * s + dy) * src_width + x * s + dx]);
    uint3 c = uint3(clamp(sum / float(s * s) + 0.5, 0.0, 255.0));
    dst[i] = c.x | (c.y << 8) | (c.z << 16) | 0xFF000000u;
}
