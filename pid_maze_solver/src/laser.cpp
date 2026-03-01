// ─── Members to add to your class ───────────────────────────────────────────
//
//   // Laser config (computed once in initLaser)
//   int    front_idx_;
//   int    north_idx_, south_idx_, east_idx_, west_idx_;
//   int    ne_idx_, nw_idx_, se_idx_, sw_idx_;
//   int    band_half_;          // ± ray count around each direction
//   double angle_increment_;
//   double angle_min_;
//   int    num_rays_;
//   bool   laser_initialized_{false};
//
//   // Tunable — set via declare_parameter or constructor
//   double WALL_CLOSE_THRESH = 0.25;   // (m) warn if wall closer than this
//   double BAND_DEGREES      = 10.0;   // ± degrees for averaging band
//   double OPENING_MIN_RANGE = 0.50;   // (m) ray longer than this → open
//   double OPENING_MIN_DEG   = 20.0;   // minimum angular width to count as
//   opening double MAX_RANGE_CLIP    = 3.50;   // clip inf readings to this
//
//   std::vector<Opening> openings_;    // populated each callback

// ─── Opening struct (only struct we keep) ────────────────────────────────────

struct Opening {
  double center_angle_deg; // robot-frame degrees from front (0=forward)
  double width_deg;        // angular width of the gap
  double avg_range;        // average range inside the gap (m)
};

// ─── Helper: average valid readings over [center ± half] indices ─────────────

static double bandAvg(const std::vector<float> &ranges, int center, int half,
                      double clip, float range_max) {
  double sum = 0.0;
  int cnt = 0;
  int n = static_cast<int>(ranges.size());
  for (int i = std::max(0, center - half); i <= std::min(n - 1, center + half);
       ++i) {
    float r = ranges[i];
    if (std::isfinite(r) && r > 0.01f && r < range_max)
      sum += std::min((double)r, clip), ++cnt;
  }
  return cnt > 0 ? sum / cnt : clip;
}

// ─── initLaser ───────────────────────────────────────────────────────────────
// Called once on the first scan message.
// Dynamically resolves every direction index from scan metadata.

void initLaser(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  angle_min_ = msg->angle_min;
  angle_increment_ = msg->angle_increment;
  num_rays_ = static_cast<int>(msg->ranges.size());

  // λ: degrees → nearest ray index
  auto toIdx = [&](double deg) {
    int idx = static_cast<int>(
        std::round((deg * M_PI / 180.0 - angle_min_) / angle_increment_));
    return std::clamp(idx, 0, num_rays_ - 1);
  };

  // Front (N) = 0°, then cardinal / ordinal directions.
  // Convention: 0°=forward, +90°=right(E), -90°=left(W), ±180°=rear(S).
  // Flip signs if your LIDAR frame differs.
  front_idx_ = north_idx_ = toIdx(0.0);
  east_idx_ = toIdx(90.0);
  west_idx_ = toIdx(-90.0);
  south_idx_ = toIdx(180.0); // or -180 — toIdx clamps either way
  ne_idx_ = toIdx(45.0);
  nw_idx_ = toIdx(-45.0);
  se_idx_ = toIdx(135.0);
  sw_idx_ = toIdx(-135.0);

  // Band half-width in ray indices from BAND_DEGREES parameter
  band_half_ =
      std::max(1, static_cast<int>(std::round((BAND_DEGREES * M_PI / 180.0) /
                                              angle_increment_)));

  RCLCPP_INFO(
      get_logger(), "[LaserInit] rays=%d  inc=%.3f°  band=±%d rays (±%.1f°)",
      num_rays_, angle_increment_ * 180.0 / M_PI, band_half_, BAND_DEGREES);
  RCLCPP_INFO(get_logger(),
              "  Indices → N:%d  NE:%d  E:%d  SE:%d  S:%d  SW:%d  W:%d  NW:%d",
              north_idx_, ne_idx_, east_idx_, se_idx_, south_idx_, sw_idx_,
              west_idx_, nw_idx_);

  laser_initialized_ = true;
}

// ─── laserCallback ───────────────────────────────────────────────────────────

void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!laser_initialized_)
    initLaser(msg);

  const auto &ranges = msg->ranges;

  // ── 1. Band-averaged distance for each preset direction ─────────────────
  double dN =
      bandAvg(ranges, north_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  double dNE =
      bandAvg(ranges, ne_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  double dE =
      bandAvg(ranges, east_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  double dSE =
      bandAvg(ranges, se_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  double dS =
      bandAvg(ranges, south_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  double dSW =
      bandAvg(ranges, sw_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  double dW =
      bandAvg(ranges, west_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);
  double dNW =
      bandAvg(ranges, nw_idx_, band_half_, MAX_RANGE_CLIP, msg->range_max);

  // ── 2. Wall proximity warnings ──────────────────────────────────────────
  if (dN < WALL_CLOSE_THRESH)
    RCLCPP_WARN(get_logger(), "WALL CLOSE → N  (%.2f m)", dN);
  if (dNE < WALL_CLOSE_THRESH)
    RCLCPP_WARN(get_logger(), "WALL CLOSE → NE (%.2f m)", dNE);
  if (dE < WALL_CLOSE_THRESH)
    RCLCPP_WARN(get_logger(), "WALL CLOSE → E  (%.2f m)", dE);
  if (dSE < WALL_CLOSE_THRESH)
    RCLCPP_WARN(get_logger(), "WALL CLOSE → SE (%.2f m)", dSE);
  if (dS < WALL_CLOSE_THRESH)
    RCLCPP_WARN(get_logger(), "WALL CLOSE → S  (%.2f m)", dS);
  if (dSW < WALL_CLOSE_THRESH)
    RCLCPP_WARN(get_logger(), "WALL CLOSE → SW (%.2f m)", dSW);
  if (dW < WALL_CLOSE_THRESH)
    RCLCPP_WARN(get_logger(), "WALL CLOSE → W  (%.2f m)", dW);
  if (dNW < WALL_CLOSE_THRESH)
    RCLCPP_WARN(get_logger(), "WALL CLOSE → NW (%.2f m)", dNW);

  // ── 3. Corridor balance (equidistant between opposite walls) ────────────
  //  Positive error → robot drifted toward the first direction listed
  double ew_drift = dE - dW; // >0 → too close to W, drift right
  double ne_nw_drift = dNE - dNW;
  double se_sw_drift = dSE - dSW;

  RCLCPP_DEBUG(get_logger(), "Drift  E-W=%.3f  NE-NW=%.3f  SE-SW=%.3f",
               ew_drift, ne_nw_drift, se_sw_drift);

  // ── 4. Detect continuous openings across the full scan ──────────────────
  openings_.clear();
  bool in_opening = false;
  int open_start = 0;
  double range_acc = 0.0;
  int range_cnt = 0;

  auto flushOpening = [&](int end_idx) {
    if (!in_opening || range_cnt == 0)
      return;
    double width_deg = (end_idx - open_start) * angle_increment_ * 180.0 / M_PI;
    if (width_deg >= OPENING_MIN_DEG) {
      double center_rad =
          angle_min_ + (open_start + end_idx) * 0.5 * angle_increment_;
      double center_deg = center_rad * 180.0 / M_PI;
      // wrap to (-180, 180]
      while (center_deg > 180.0)
        center_deg -= 360.0;
      while (center_deg <= -180.0)
        center_deg += 360.0;
      openings_.push_back({center_deg, width_deg, range_acc / range_cnt});
    }
    in_opening = false;
    range_acc = 0.0;
    range_cnt = 0;
  };

  for (int i = 0; i < num_rays_; ++i) {
    float r = ranges[i];
    bool open = std::isfinite(r) && r > static_cast<float>(OPENING_MIN_RANGE) &&
                r < msg->range_max;
    if (open) {
      if (!in_opening) {
        in_opening = true;
        open_start = i;
      }
      range_acc += r;
      ++range_cnt;
    } else {
      flushOpening(i);
    }
  }
  flushOpening(num_rays_ - 1);

  // ── 5. Log openings ─────────────────────────────────────────────────────
  for (auto &op : openings_)
    RCLCPP_INFO(get_logger(),
                "Opening  center=%.1f°  width=%.1f°  avg_range=%.2f m",
                op.center_angle_deg, op.width_deg, op.avg_range);

  // wall_dist values (dN … dNW) and openings_ are now ready for your
  // maze-solver / motion controller to consume this tick.
}

void PIDMazeSolver::initLaser(const Laser::SharedPtr msg) {
  angle_min_ = msg->angle_min;
  angle_increment_ = msg->angle_increment;
  num_rays_ = static_cast<int>(msg->ranges.size());

  auto toIdx = [&](double deg) {
    int idx = static_cast<int>(
        std::round((deg * M_PI / 180.0 - angle_min_) / angle_increment_));
    return std::clamp(idx, 0, num_rays_ - 1);
  };

  // TODO: After first run on real robot, check the RCLCPP_INFO indices below
  // and verify N/S/E/W match physical directions using a corner test.
  // If N and S are swapped, swap toIdx arguments for north/south (and
  // diagonals). If E and W are swapped, swap toIdx arguments for east/west (and
  // diagonals).
  north_idx_ = toIdx(0.0);
  south_idx_ = toIdx(180.0);
  east_idx_ = toIdx(90.0);
  west_idx_ = toIdx(-90.0);
  ne_idx_ = toIdx(45.0);
  nw_idx_ = toIdx(-45.0);
  se_idx_ = toIdx(135.0);
  sw_idx_ = toIdx(-135.0);
  front_idx_ = north_idx_;

  band_half_ =
      std::max(1, static_cast<int>(std::round((BAND_DEGREES * M_PI / 180.0) /
                                              angle_increment_)));

  RCLCPP_INFO(
      get_logger(),
      "[LaserInit] rays=%d  angle_min=%.2f°  inc=%.3f°  band=±%d rays (±%.1f°)",
      num_rays_, angle_min_ * 180.0 / M_PI, angle_increment_ * 180.0 / M_PI,
      band_half_, BAND_DEGREES);

  RCLCPP_INFO(get_logger(),
              "  Indices → N:%d  NE:%d  E:%d  SE:%d  S:%d  SW:%d  W:%d  NW:%d",
              north_idx_, ne_idx_, east_idx_, se_idx_, south_idx_, sw_idx_,
              west_idx_, nw_idx_);

  // Quick sanity: print band-averaged range for each direction on first scan
  // so you can immediately verify against known physical distances on the
  // robot.
  auto quickAvg = [&](int center) {
    double sum = 0.0;
    int cnt = 0;
    for (int i = std::max(0, center - band_half_);
         i <= std::min(num_rays_ - 1, center + band_half_); ++i) {
      float r = msg->ranges[i];
      if (std::isfinite(r) && r > 0.01f && r < msg->range_max)
        sum += r, ++cnt;
    }
    return cnt > 0 ? sum / cnt : -1.0;
  };

  RCLCPP_INFO(get_logger(),
              "  First-scan ranges (m) → N:%.2f  NE:%.2f  E:%.2f  SE:%.2f"
              "  S:%.2f  SW:%.2f  W:%.2f  NW:%.2f",
              quickAvg(north_idx_), quickAvg(ne_idx_), quickAvg(east_idx_),
              quickAvg(se_idx_), quickAvg(south_idx_), quickAvg(sw_idx_),
              quickAvg(west_idx_), quickAvg(nw_idx_));

  RCLCPP_INFO(
      get_logger(),
      "  Place robot facing a known wall (e.g. wall directly in front = N)."
      " N should show shortest range. Adjust toIdx args above if mismatch.");

  laser_initialized_ = true;
}