#include "render/cpu/kernel.hpp"

#include "image/opencv.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace docksim::render::cpu {

namespace {

using namespace shading;

constexpr float earth_r = 6371008.8F;
constexpr float edge = raster::edge;
constexpr float inf = std::numeric_limits<float>::infinity();

float map_y(const raster::PixelMap& m, float north) {
    return m.cy + north * (m.ky1 + north * (m.ky2 + north * m.ky3));
}

bool dem_sample(const DemView& view, float east, float north, float& height) {
    if (!view.mosaic || view.mosaic->values.empty()) return false;
    const auto& v = view.mosaic->values;
    const auto& valid = view.mosaic->valid;
    const int w = v.width();
    const int h = v.height();
    float lx = view.map.cx + east * view.map.kx;
    float ly = map_y(view.map, north);
    if (lx < 0.0F || ly < 0.0F || lx > static_cast<float>(w - 1) || ly > static_cast<float>(h - 1))
        return false;
    lx = std::min(lx, static_cast<float>(w) - edge);
    ly = std::min(ly, static_cast<float>(h) - edge);
    const int ix = static_cast<int>(lx);
    const int iy = static_cast<int>(ly);
    if (valid(ix, iy) == 0 || valid(ix + 1, iy + 1) == 0) return false;
    const float fx = lx - static_cast<float>(ix);
    const float fy = ly - static_cast<float>(iy);
    const float top = v(ix, iy) * (1 - fx) + v(ix + 1, iy) * fx;
    const float bottom = v(ix, iy + 1) * (1 - fx) + v(ix + 1, iy + 1) * fx;
    height = top * (1 - fy) + bottom * fy;
    return true;
}

bool grid_covers(const GridView& g, float gx, float gy) {
    return g.grid != nullptr && !g.grid->height.empty() && gx >= 0.0F && gy >= 0.0F &&
           gx <= static_cast<float>(g.grid->width() - 1) &&
           gy <= static_cast<float>(g.grid->rows() - 1);
}

float grid_height(const GridView& g, float east, float north) {
    float gx = g.map.gx0 + east * g.map.bx;
    float gy = g.map.gy0 - north * g.map.by;
    if (!grid_covers(g, gx, gy)) return 0.0F;
    gx = std::min(gx, static_cast<float>(g.grid->width()) - edge);
    gy = std::min(gy, static_cast<float>(g.grid->rows()) - edge);
    const int ix = static_cast<int>(gx);
    const int iy = static_cast<int>(gy);
    const float fx = gx - static_cast<float>(ix);
    const float fy = gy - static_cast<float>(iy);
    const auto& v = g.grid->height;
    const float top = v(ix, iy) * (1 - fx) + v(ix + 1, iy) * fx;
    const float bottom = v(ix, iy + 1) * (1 - fx) + v(ix + 1, iy + 1) * fx;
    return top * (1 - fy) + bottom * fy;
}

std::uint32_t grid_packed(const GridView& g, float east, float north) {
    if (g.grid == nullptr || g.grid->height.empty()) return 0U;
    const int ix = static_cast<int>(std::rint(g.map.gx0 + east * g.map.bx));
    const int iy = static_cast<int>(std::rint(g.map.gy0 - north * g.map.by));
    if (ix < 0 || iy < 0 || ix >= g.grid->width() || iy >= g.grid->rows()) return 0U;
    return g.grid->style(ix, iy);
}

const GridView& grid_for(const BuildingView& b, float east, float north) {
    if (grid_covers(b.near, b.near.map.gx0 + east * b.near.map.bx,
                    b.near.map.gy0 - north * b.near.map.by)) {
        return b.near;
    }
    return b.far;
}

float ground_at(const Scene& s, float east, float north, float* building = nullptr) {
    const float h = building_at(s.buildings, east, north);
    if (building != nullptr) *building = h;
    return terrain_at(s.relief, east, north) + h;
}

// Earth curvature lowers far terrain below the tangent plane.
float ray_up(float camera_up, float rz, float t) {
    return camera_up + rz * t - (t * t) / (2.0F * earth_r);
}

bool sample_level(const LevelView& view, float east, float north, Vec3& rgb) {
    if (!view.mosaic || view.mosaic->valid.empty()) return false;
    const auto& l = *view.mosaic;
    const int w = l.width();
    const int h = l.height();
    float lx = view.map.cx + east * view.map.kx;
    float ly = map_y(view.map, north);
    if (lx < 0.0F || ly < 0.0F || lx > static_cast<float>(w - 1) || ly > static_cast<float>(h - 1))
        return false;
    lx = std::min(lx, static_cast<float>(w) - edge);
    ly = std::min(ly, static_cast<float>(h) - edge);
    const int ix = static_cast<int>(lx);
    const int iy = static_cast<int>(ly);
    if (l.valid(ix, iy) == 0 || l.valid(ix + 1, iy + 1) == 0) return false;
    const float fx = lx - static_cast<float>(ix);
    const float fy = ly - static_cast<float>(iy);
    const auto px = [&](int x, int y) {
        const auto* p =
            l.rgb.data() + 3 * (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) +
                                static_cast<std::size_t>(x));
        return Vec3{static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2])};
    };
    rgb = mix(mix(px(ix, iy), px(ix + 1, iy), fx), mix(px(ix, iy + 1), px(ix + 1, iy + 1), fx), fy);
    return true;
}

// Texture mip: blend the two levels around the wanted zoom. A level without
// data hands its weight to the other one; with neither, try coarser levels.
Vec3 texture_at(std::span<const LevelView> lv, float zoom, float east, float north) {
    const int levels = static_cast<int>(std::min<std::size_t>(lv.size(), max_levels));
    if (levels == 0) return {96.0F, 104.0F, 84.0F};
    const auto at = [&](int k) -> const LevelView& { return lv[static_cast<std::size_t>(k)]; };
    int k = 0;
    while (k + 1 < levels && at(k + 1).zoom >= zoom) ++k;
    Vec3 a{};
    Vec3 b{};
    const bool has_a = sample_level(at(k), east, north, a);
    bool has_b = false;
    float t = 0.0F;
    if (k + 1 < levels && zoom < at(k).zoom) {
        has_b = sample_level(at(k + 1), east, north, b);
        t = std::clamp((at(k).zoom - zoom) / (at(k).zoom - at(k + 1).zoom), 0.0F, 1.0F);
    }
    if (has_a && has_b) return mix(a, b, t);
    if (has_a) return a;
    if (has_b) return b;
    for (int j = k + 2; j < levels; ++j) {
        if (sample_level(at(j), east, north, a)) return a;
    }
    // Ground outside the imagery coverage.
    return {96.0F, 104.0F, 84.0F};
}

// PCG output permutation. Good enough for sensor noise.
constexpr std::uint32_t hash(std::uint32_t x) {
    x = x * 747796405U + 2891336453U;
    x = ((x >> ((x >> 28U) + 4U)) ^ x) * 277803737U;
    return (x >> 22U) ^ x;
}

float gaussian(std::uint32_t key) {
    const float u1 = static_cast<float>((hash(key) >> 8U) + 1U) * (1.0F / 16777217.0F);
    const float u2 = static_cast<float>(hash(key ^ 0x9e3779b9U) >> 8U) * (1.0F / 16777216.0F);
    return std::sqrt(-2.0F * std::log(u1)) * std::cos(6.2831853F * u2);
}

float unit_hash(std::uint32_t x) {
    return static_cast<float>(hash(x) >> 8U) * (1.0F / 16777216.0F);
}

std::uint32_t cell(float v) {
    return static_cast<std::uint32_t>(static_cast<int>(std::floor(v)));
}

float band_limit(float value, float mean, float pixels) {
    return mean + (value - mean) * smoothstep(1.0F, 2.0F, pixels);
}

float value_noise(float u, float v, std::uint32_t seed) {
    const float iu = std::floor(u);
    const float iv = std::floor(v);
    const float wu = smoothstep(0.0F, 1.0F, u - iu);
    const float wv = smoothstep(0.0F, 1.0F, v - iv);
    const auto cu = static_cast<std::uint32_t>(static_cast<int>(iu));
    const auto cv = static_cast<std::uint32_t>(static_cast<int>(iv));
    const float c00 = unit_hash((cu * 0x8DA6B343U ^ cv * 0xD8163841U) ^ seed);
    const float c10 = unit_hash(((cu + 1U) * 0x8DA6B343U ^ cv * 0xD8163841U) ^ seed);
    const float c01 = unit_hash((cu * 0x8DA6B343U ^ (cv + 1U) * 0xD8163841U) ^ seed);
    const float c11 = unit_hash(((cu + 1U) * 0x8DA6B343U ^ (cv + 1U) * 0xD8163841U) ^ seed);
    const float low = c00 * (1 - wu) + c10 * wu;
    const float high = c01 * (1 - wu) + c11 * wu;
    return low * (1 - wv) + high * wv;
}

float weathering(float along, float above, float top, float footprint, std::uint32_t seed) {
    const float u = along / streak_m;
    const float iu = std::floor(u);
    const std::uint32_t c = cell(u) ^ seed;
    const float s = unit_hash(c);
    const float strength = s * s;
    float half_w = std::lerp(streak_half_min, streak_half_max, unit_hash(c ^ 0x85EBCA6BU));
    const float wobble = 0.03F * (value_noise(above / 1.5F, iu, seed ^ 0x27D4EB2FU) - 0.5F);
    const float centre =
        0.5F + (0.47F - half_w) * (2.0F * unit_hash(c ^ 0x9E3779B9U) - 1.0F) + wobble;
    const float below = std::max(top - above, 0.0F);
    const float run = top * std::lerp(streak_length_min, 1.0F, unit_hash(c ^ 0xC2B2AE35U));
    const float fall = 1.0F - smoothstep(0.0F, run, below);
    half_w *= 0.4F + 0.6F * fall;
    const float profile = 1.0F - smoothstep(0.0F, 1.0F, std::abs(u - iu - centre) / half_w);
    const float fall_mean = 1.0F - smoothstep(0.0F, top * streak_length_mean, below);
    const float grain = 0.6F + 0.8F * value_noise(iu * 3.0F, above / 0.8F, seed ^ 0x165667B1U);
    const float column = strength * half_w * fall;
    float streak = std::lerp(column, strength * profile * fall,
                             smoothstep(1.0F, 2.0F, 2.0F * half_w * streak_m / footprint));
    streak = std::lerp(streak_mean * fall_mean, streak, smoothstep(1.0F, 2.0F, streak_m / footprint));
    const float stain = band_limit(value_noise(along / blotch_m, above / blotch_m, seed),
                                   blotch_mean, blotch_m / footprint);
    const float splash = std::clamp(1.0F - above / splash_m, 0.0F, 1.0F);
    return streak_dark * streak * grain + blotch_dark * stain + splash_dark * splash;
}

float trim_visible(float size_m, float footprint) {
    return smoothstep(trim_pixels, 2 * trim_pixels, size_m / footprint);
}

struct Cell {
    float floor_m;
    float bay_m;
    float f0;
    float f1;
    float b0;
    float b1;
};

// Stucco trim, cornices and ground floor grooves of a classic facade.
void classic_detail(Vec3& wall, Vec3& glass_colour, bool window, bool zone, float floor_, float bay,
                    float above, float footprint, const Cell& c, float top) {
    const bool ground = above < c.floor_m;
    Vec3 w = wall * (ground ? ground_shade : 1.0F);
    float groove = std::fmod(above, groove_m) < groove_line_m ? 1.0F : 0.0F;
    groove = band_limit(groove, groove_line_m / groove_m, groove_line_m / footprint);
    w *= 1.0F - groove_dark * groove * (ground ? 1.0F : 0.0F);
    float cornice = floor_ > 1.0F - cornice_m / c.floor_m ? 1.0F : 0.0F;
    cornice =
        band_limit(cornice, cornice_m / c.floor_m, cornice_m / footprint) * (ground ? 0.0F : 1.0F);
    w = mix(w, trim, cornice_mix * cornice);
    const bool ledge =
        floor_ > 1.0F - (cornice_m + ledge_m) / c.floor_m && floor_ <= 1.0F - cornice_m / c.floor_m;
    const float shadow = band_limit(ledge ? 1.0F : 0.0F, ledge_m / c.floor_m, ledge_m / footprint) *
                         (ground ? 0.0F : 1.0F);
    w *= 1.0F - ledge_dark * shadow;
    const float fu = frame_m / c.floor_m;
    const float fb = frame_m / c.bay_m;
    const bool frame = zone && !window && floor_ > c.f0 - fu && floor_ < c.f1 + fu &&
                       bay > c.b0 - fb && bay < c.b1 + fb;
    w = mix(w, trim, (frame ? 1.0F : 0.0F) * trim_visible(frame_m, footprint));
    // Sill and hood belong to the window of this floor.
    const float middle = above - (floor_ - 0.5F * (c.f0 + c.f1)) * c.floor_m;
    const bool row = middle > window_bottom_m && middle < top - window_top_gap_m;
    const float over = (frame_m + over_m) / c.bay_m;
    const bool wide = row && bay > c.b0 - over && bay < c.b1 + over;
    const float sill_top = c.f0 - fu;
    const float sill_low = sill_top - sill_m / c.floor_m;
    const bool sill = wide && floor_ > sill_low && floor_ <= sill_top;
    const bool drop = wide && floor_ > sill_low - sill_shadow_m / c.floor_m && floor_ <= sill_low;
    const bool hood = wide && floor_ >= c.f1 + fu && floor_ < c.f1 + fu + hood_m / c.floor_m;
    w = mix(w, trim, (sill ? 1.0F : 0.0F) * trim_visible(sill_m, footprint));
    w = mix(w, trim, (hood ? 1.0F : 0.0F) * trim_visible(hood_m, footprint));
    w *= 1.0F - sill_dark * (drop ? 1.0F : 0.0F) * trim_visible(sill_shadow_m, footprint);
    const float transom_at = c.f0 + transom * (c.f1 - c.f0);
    const bool cross = std::abs(bay - 0.5F * (c.b0 + c.b1)) * c.bay_m < 0.5F * mullion_m ||
                       std::abs(floor_ - transom_at) * c.floor_m < 0.5F * mullion_m;
    glass_colour =
        mix(glass_colour, trim, (cross ? 1.0F : 0.0F) * trim_visible(mullion_m, footprint));
    wall = w;
}

Vec3 mat_texel(const materials::Atlas& a, int base, int n, int ix, int iy) {
    ix = ((ix % n) + n) % n;
    iy = ((iy % n) + n) % n;
    const auto* t = a.texels.data() + 3 * static_cast<std::size_t>(base + iy * n + ix);
    return {static_cast<float>(t[0]), static_cast<float>(t[1]), static_cast<float>(t[2])};
}

Vec3 mat_level(const materials::Atlas& a, int m, int k, float fu, float fv) {
    const int n = materials::size >> k;
    int base = m * materials::block;
    for (int j = 0; j < k; ++j) base += (materials::size >> j) * (materials::size >> j);
    const float x = (fu - std::floor(fu)) * static_cast<float>(n) - 0.5F;
    const float y = (fv - std::floor(fv)) * static_cast<float>(n) - 0.5F;
    const float x0 = std::floor(x);
    const float y0 = std::floor(y);
    const float wx = x - x0;
    const float wy = y - y0;
    const int ix = static_cast<int>(x0);
    const int iy = static_cast<int>(y0);
    const Vec3 top = mix(mat_texel(a, base, n, ix, iy), mat_texel(a, base, n, ix + 1, iy), wx);
    const Vec3 bottom =
        mix(mat_texel(a, base, n, ix, iy + 1), mat_texel(a, base, n, ix + 1, iy + 1), wx);
    return mix(top, bottom, wy);
}

// Colour factor of photo material m at wall position u, v.
Vec3 material(const materials::Atlas* a, int m, float u, float v, float footprint) {
    if (a == nullptr || m < 0 || static_cast<std::size_t>(m) >= a->info.size()) return {1, 1, 1};
    const auto& mi = a->info[static_cast<std::size_t>(m)];
    const float lod = std::clamp(
        std::log2(std::max(footprint * static_cast<float>(materials::size) / mi.size_m, 1e-6F)),
        0.0F, static_cast<float>(materials::levels - 1));
    const int k0 = static_cast<int>(std::floor(lod));
    const int k1 = std::min(k0 + 1, materials::levels - 1);
    const float fu = u / mi.size_m;
    const float fv = -v / mi.size_m;
    const Vec3 c = mix(mat_level(*a, m, k0, fu, fv), mat_level(*a, m, k1, fu, fv),
                       lod - static_cast<float>(k0));
    return c / Vec3{mi.mean_r, mi.mean_g, mi.mean_b};
}

// Glass: room or curtain per window, sky reflection, reveal shadow.
Vec3 window_glass(float along, float above, float floor_, float bay, const Cell& c,
                  std::uint32_t seed, float cos_view, const Vec3& sky) {
    const std::uint32_t key =
        cell(along / c.bay_m) * 0x8DA6B343U ^ cell(above / c.floor_m) * 0xD8163841U;
    const float room = unit_hash(key ^ seed);
    const float curt = unit_hash(key ^ seed ^ 0x5BD1E995U) > curtain_share ? curtain_mix : 0.0F;
    Vec3 g = glass * (room_dark + room_spread * room);
    g = mix(g, curtain, curt);
    float grazing = 1.0F - std::clamp(cos_view, 0.0F, 1.0F);
    grazing = grazing * grazing * grazing * grazing * grazing;
    g = mix(g, sky, reflect_min + (1.0F - reflect_min) * grazing);
    const float s = (floor_ > c.f1 - reveal_m / c.floor_m ? reveal_top : 1.0F) *
                    (bay < c.b0 + reveal_m / c.bay_m ? reveal_side : 1.0F);
    return g * s;
}

Vec3 facade(const ShadeParams& p, float style, float along, float above, float top, float footprint,
            float cos_view) {
    if (p.styles.empty()) return {160, 150, 140};
    const int index =
        std::clamp(static_cast<int>(style) - 1, 0, static_cast<int>(p.styles.size()) - 1);
    const auto& row = p.styles[static_cast<std::size_t>(index)];
    const auto& layout = window_layouts[static_cast<std::size_t>(
        std::clamp(row.layout, 0, static_cast<int>(window_layouts.size()) - 1))];
    const Cell c{.floor_m = layout.floor_m,
                 .bay_m = layout.bay_m,
                 .f0 = layout.floor_lo,
                 .f1 = layout.floor_hi,
                 .b0 = layout.bay_lo,
                 .b1 = layout.bay_hi};
    const std::uint32_t seed = static_cast<std::uint32_t>(index) * 0x9E3779B9U;
    const float dirt = row.weathering * weathering(along, above, top, footprint, seed);
    const Vec3 texture = material(p.atlas, row.material, along, above, footprint);
    Vec3 colour = row.colour * texture * (1.0F - dirt);
    float floor_ = std::fmod(above, c.floor_m) / c.floor_m;
    float bay = std::fmod(along, c.bay_m) / c.bay_m;
    if (floor_ < 0.0F) floor_ += 1.0F;
    if (bay < 0.0F) bay += 1.0F;
    Vec3 g = window_glass(along, above, floor_, bay, c, seed, cos_view, p.sky.horizon);
    const bool zone = above > window_bottom_m && above < top - window_top_gap_m;
    const bool window = floor_ > c.f0 && floor_ < c.f1 && bay > c.b0 && bay < c.b1 && zone;
    if (row.layout == windows_classic)
        classic_detail(colour, g, window, zone, floor_, bay, above, footprint, c, top);
    const Vec3 pattern = window ? g : colour;
    // Band limit: windows smaller than a few pixels fade to the facade average.
    const float share = (c.f1 - c.f0) * (c.b1 - c.b0);
    const Vec3 average = colour * (1 - share) + g * share;
    const float fade = smoothstep(window_fade_lo, window_fade_hi, c.floor_m / footprint);
    return mix(average, pattern, fade);
}

}  // namespace

float terrain_at(const Relief& r, float east, float north) {
    // The far DEM is read only in the blend band or where the near DEM has no data.
    float far = 0.0F;
    float near = 0.0F;
    const float dist = std::sqrt(east * east + north * north);
    if (r.near.mosaic && dist < r.near_radius && dem_sample(r.near, east, north, near)) {
        const float w = smoothstep(r.blend_start * r.near_radius, r.near_radius, dist);
        if (w <= 0.0F) return near;
        if (!dem_sample(r.far, east, north, far)) far = r.fallback;
        return near * (1 - w) + far * w;
    }
    if (!dem_sample(r.far, east, north, far)) far = r.fallback;
    return far;
}

float building_at(const BuildingView& b, float east, float north) {
    return grid_height(grid_for(b, east, north), east, north);
}

Surface march(const Scene& scene, const MarchParams& p, const Vec3& r, float start) {
    Surface out{.distance = inf};
    const float ratio = std::pow(p.reach / 0.5F, 1.0F / static_cast<float>(march_steps - 1));
    // Empty air above the highest surface needs no steps.
    float t = std::max(0.5F, start);
    if (r.z < -1e-6F) t = std::max(t, (p.camera_up - scene.top) / -r.z);
    float lo = t;
    float hi = inf;
    for (int k = 0; k < 4 * march_steps && t <= p.reach; ++k) {
        if (ray_up(p.camera_up, r.z, t) <= ground_at(scene, r.x * t, r.y * t)) {
            hi = t;
            break;
        }
        lo = t;
        t *= ratio;
    }
    if (std::isinf(hi)) return out;
    for (int k = 0; k < refine_steps; ++k) {
        const float mid = (lo + hi) / 2.0F;
        if (ray_up(p.camera_up, r.z, mid) <= ground_at(scene, r.x * mid, r.y * mid)) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    float e = r.x * hi;
    float nn = r.y * hi;
    const float s = normal_step_m;
    const float dzx = (ground_at(scene, e + s, nn) - ground_at(scene, e - s, nn)) / (2.0F * s);
    const float dzy = (ground_at(scene, e, nn + s) - ground_at(scene, e, nn - s)) / (2.0F * s);
    const float len = std::sqrt(dzx * dzx + dzy * dzy + 1.0F);
    const float nx = -dzx / len;
    const float ny = -dzy / len;
    const float nz = 1.0F / len;
    Vec3 o{nx, ny, nz};

    float ie = e - nx * inside_m;
    float in_ = nn - ny * inside_m;
    const GridView& cell_grid = grid_for(scene.buildings, ie, in_);
    const std::uint32_t packed = grid_packed(cell_grid, ie, in_);
    out.style = static_cast<float>(packed & 0xFFU);
    const std::uint32_t row = packed >> 8U;
    if (nz < wall_normal_z && row > 0U && cell_grid.grid != nullptr &&
        row - 1U < cell_grid.grid->edges.size()) {
        // Move the hit onto the exact outline plane. The edge normal in
        // east/north is (a, -b); its sign is arbitrary, toward picks the side.
        const auto& line = cell_grid.grid->edges[row - 1U];
        const float wx = line[0];
        const float wy = -line[1];
        const float gx = cell_grid.map.gx0 + ie * cell_grid.map.bx;
        const float gy = cell_grid.map.gy0 - in_ * cell_grid.map.by;
        const float at_hit = (line[0] * gx + line[1] * gy - line[2]) / cell_grid.map.bx +
                             wx * (e - ie) + wy * (nn - in_);
        const float toward = wx * r.x + wy * r.y;
        // Near a corner the cell can hold the other wall's edge; the grid normal tells.
        const bool agree = std::abs(wx * nx + wy * ny) > min_agree * std::sqrt(nx * nx + ny * ny);
        const float move = std::abs(toward) > min_toward ? -at_hit / toward : 2.0F * max_snap_m;
        if (agree && std::abs(move) < max_snap_m) {
            hi += move;
            const float sign = toward > 0.0F ? -1.0F : 1.0F;
            o = {sign * wx, sign * wy, 0.0F};
            e = r.x * hi;
            nn = r.y * hi;
            ie = e - o.x * inside_m;
            in_ = nn - o.y * inside_m;
        }
    }
    out.distance = hi;
    out.normal = o;
    const float pz = ray_up(p.camera_up, r.z, hi);
    float b = 0.0F;
    const float surface = ground_at(scene, e, nn, &b);
    out.terrain = surface - b;
    out.roof = building_at(scene.buildings, ie, in_);

    // Shadow LoD: full steps near, half steps to shadow_max_m, none beyond.
    if (!p.shadows || hi > shadow_max_m) return out;
    const int count = hi > shadow_full_m ? shadow_steps / 2 : shadow_steps;
    const float qx = e + nx * shadow_lift_m;
    const float qy = nn + ny * shadow_lift_m;
    const float qz = pz + nz * shadow_lift_m;
    const float shadow_ratio =
        std::pow(shadow_range_m / 0.5F, 1.0F / static_cast<float>(count - 1));
    float u = 0.5F;
    for (int k = 0; k < count; ++k) {
        const float z = qz + p.sun.z * u;
        // Above the highest surface nothing can block the sun any more.
        if (z > scene.top) break;
        if (z < ground_at(scene, qx + p.sun.x * u, qy + p.sun.y * u)) {
            out.lit = 0.0F;
            break;
        }
        u *= shadow_ratio;
    }
    return out;
}

std::array<std::uint8_t, 3> shade(const ShadeParams& p, int i, const Vec3& r, const Surface& s) {
    const bool hit = std::isfinite(s.distance);
    const float d = hit ? s.distance : p.reach;
    const float east = r.x * d;
    const float north = r.y * d;
    const Vec3& nrm = s.normal;
    // Ground size of this pixel; it grows with distance and at grazing angles.
    const float cos_i = std::max(std::abs(dot(r, nrm)), min_cos);
    const float footprint = d * p.pixel_angle / cos_i;

    Vec3 color{};
    if (hit) color = texture_at(p.levels, std::log2(p.texel0 / footprint), east, north);

    const float up = ray_up(p.camera_up, r.z, d);
    const float above = up - s.terrain;
    if (s.roof > min_building_m && nrm.z < wall_normal_z && above < s.roof - roof_band_m) {
        const float style = std::fmod(s.style, 256.0F);
        // Anchored to a fixed point, so that the patterns stay on the wall as the camera moves.
        const float along = (east + p.anchor_e) * -nrm.y + (north + p.anchor_n) * nrm.x;
        color = facade(p, style, along, above, s.roof, footprint, std::abs(dot(r, nrm)));
    }
    if (p.light.day > 0.0F) {
        const float facing = std::clamp(dot(nrm, p.sun), 0.0F, 1.0F);
        color *= 1.0F + p.light.day * ((ambient + direct * facing * s.lit) / p.light.flat - 1.0F);
    }

    const float haze = (1.0F - std::exp(-d / haze_distance_m)) * haze_max;
    color = mix(color, p.sky.horizon, haze);
    if (!hit) {
        const float alt = std::sqrt(std::clamp(r.z, 0.0F, 1.0F));
        color = mix(p.sky.horizon, p.sky.zenith, alt);
        const float toward = std::clamp(r.x * p.sky.sin_az + r.y * p.sky.cos_az, 0.0F, 1.0F);
        const float t4 = toward * toward * toward * toward;
        color += Vec3{60.0F * p.sky.low, 30.0F * p.sky.low, 0.0F} * t4;
    }

    const int x = i % p.width;
    const int y = i / p.width;
    const float hw = static_cast<float>(p.width) / 2.0F;
    const float hh = static_cast<float>(p.height) / 2.0F;
    const float dx = static_cast<float>(x) - hw;
    const float dy = static_cast<float>(y) - hh;
    const float r2 = (dx * dx + dy * dy) / (hw * hw + hh * hh);
    color *= p.exposure.gain * (1.0F - vignette_strength * r2);
    if (p.sigma > 0.0F) {
        const std::uint32_t key = static_cast<std::uint32_t>(i) * 3U + p.seed;
        color += Vec3{gaussian(key), gaussian(key + 1U), gaussian(key + 2U)} * p.sigma;
    }
    const auto byte = [](float v) {
        return static_cast<std::uint8_t>(std::clamp(v, 0.0F, 255.0F));
    };
    return {byte(color.x), byte(color.y), byte(color.z)};
}

void downsample(std::span<const std::uint8_t> src, int src_width, int s,
                std::span<std::uint8_t> dst, int out_width, int out_height) {
    image::quiet_opencv();
    // cv::Mat has no const view. cv::resize only reads src.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
    auto* pixels = const_cast<std::uint8_t*>(src.data());
    const cv::Mat in(out_height * s, src_width, CV_8UC3, pixels);
    cv::Mat out(out_height, out_width, CV_8UC3, dst.data());
    cv::resize(in(cv::Rect(0, 0, out_width * s, out_height * s)), out, out.size(), 0, 0,
               cv::INTER_AREA);
}

}  // namespace docksim::render::cpu
