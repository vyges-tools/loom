// SPDX-License-Identifier: Apache-2.0
//! Rectilinear (Manhattan) polygon sets — boolean algebra over axis-aligned rectangles, and the
//! polygon outlines that fall out of it.
//!
//! Physical design is full of regions that are unions and differences of rectangles: the area a
//! design's rows cover, a power grid's straps minus its blockages, the space left after macros
//! are carved out. Answering "what shape is that, exactly?" — including which parts are **holes**
//! — is the operation, and it is shared substrate rather than any one engine's business.
//!
//! # Representation
//!
//! A set is stored as a canonical **vertical slab decomposition**: the plane is cut at every
//! x-coordinate that appears, and within each slab the covered region is a sorted list of
//! disjoint y-intervals. Adjacent slabs with identical intervals are merged. Two sets covering
//! the same region therefore have the *same* representation, whatever order their rectangles
//! arrived in — so equality is meaningful and boolean ops are a per-slab interval merge.
//!
//! Intervals are **half-open** in y, and slabs half-open in x: `[0,10)` and `[10,20)` abut, and
//! abutting rectangles merge into one region. That is what makes a stack of standard-cell rows
//! come out as a single shape rather than N separate ones.
//!
//! # Orientation
//!
//! Outlines are traced with the **interior on the left**, which makes outer boundaries
//! counter-clockwise (positive signed area) and holes clockwise (negative). That is the same
//! convention DEF/GDS readers expect, and it is what lets a hole be told from an island without
//! a containment test.

/// A point in database units.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Point {
    pub x: i32,
    pub y: i32,
}

impl Point {
    pub fn new(x: i32, y: i32) -> Point {
        Point { x, y }
    }
}

/// An axis-aligned rectangle, half-open: it covers `[x0, x1) × [y0, y1)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rect {
    pub x0: i32,
    pub y0: i32,
    pub x1: i32,
    pub y1: i32,
}

impl Rect {
    pub fn new(x0: i32, y0: i32, x1: i32, y1: i32) -> Rect {
        Rect { x0, y0, x1, y1 }
    }
    pub fn is_empty(&self) -> bool {
        self.x1 <= self.x0 || self.y1 <= self.y0
    }
    pub fn width(&self) -> i32 {
        self.x1 - self.x0
    }
    pub fn height(&self) -> i32 {
        self.y1 - self.y0
    }
}

/// A closed rectilinear outline. Points are corners in order; consecutive points always differ in
/// exactly one coordinate. The closing edge from the last point back to the first is implied.
pub type Outline = Vec<Point>;

/// One connected piece of a set: its outer boundary, and the holes inside it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Polygon90 {
    /// Counter-clockwise.
    pub outer: Outline,
    /// Clockwise, each strictly inside `outer`.
    pub holes: Vec<Outline>,
}

/// Sorted, disjoint, non-abutting half-open intervals.
type Ivals = Vec<(i32, i32)>;

/// A vertical slab `[x0, x1)` and the y-intervals covered within it.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Slab {
    x0: i32,
    x1: i32,
    ys: Ivals,
}

/// A region of the plane made of axis-aligned rectangles.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Poly90Set {
    slabs: Vec<Slab>,
}

/// Merge a list of intervals that may overlap or abut into the canonical sorted disjoint form.
fn normalize(mut v: Ivals) -> Ivals {
    v.retain(|(a, b)| b > a);
    v.sort_unstable();
    let mut out: Ivals = Vec::with_capacity(v.len());
    for (a, b) in v {
        match out.last_mut() {
            // `>=` and not `>`: abutting intervals merge, so abutting rectangles form one region.
            Some(last) if a <= last.1 => last.1 = last.1.max(b),
            _ => out.push((a, b)),
        }
    }
    out
}

/// Walk two canonical interval lists together, calling `keep(in_a, in_b)` to decide whether the
/// stretch between consecutive boundaries belongs to the result.
///
/// One function for union, difference and intersection: they differ only in that predicate, and
/// writing three near-identical sweeps is how the three drift apart.
fn combine(a: &[(i32, i32)], b: &[(i32, i32)], keep: fn(bool, bool) -> bool) -> Ivals {
    let mut edges: Vec<i32> = Vec::with_capacity((a.len() + b.len()) * 2);
    for (s, e) in a.iter().chain(b.iter()) {
        edges.push(*s);
        edges.push(*e);
    }
    edges.sort_unstable();
    edges.dedup();

    let covers = |v: &[(i32, i32)], lo: i32| v.iter().any(|(s, e)| *s <= lo && lo < *e);
    let mut out: Ivals = Vec::new();
    for w in edges.windows(2) {
        let (lo, hi) = (w[0], w[1]);
        if keep(covers(a, lo), covers(b, lo)) {
            match out.last_mut() {
                Some(last) if last.1 == lo => last.1 = hi,
                _ => out.push((lo, hi)),
            }
        }
    }
    out
}

impl Poly90Set {
    pub fn new() -> Poly90Set {
        Poly90Set::default()
    }

    /// Build from rectangles. Overlaps and abutments are resolved; order does not matter.
    pub fn from_rects(rects: &[Rect]) -> Poly90Set {
        let rects: Vec<Rect> = rects.iter().copied().filter(|r| !r.is_empty()).collect();
        if rects.is_empty() {
            return Poly90Set::default();
        }
        let mut xs: Vec<i32> = rects.iter().flat_map(|r| [r.x0, r.x1]).collect();
        xs.sort_unstable();
        xs.dedup();

        let mut slabs = Vec::with_capacity(xs.len().saturating_sub(1));
        for w in xs.windows(2) {
            let (x0, x1) = (w[0], w[1]);
            let ys = normalize(
                rects
                    .iter()
                    .filter(|r| r.x0 <= x0 && x1 <= r.x1)
                    .map(|r| (r.y0, r.y1))
                    .collect(),
            );
            slabs.push(Slab { x0, x1, ys });
        }
        let mut s = Poly90Set { slabs };
        s.compact();
        s
    }

    pub fn is_empty(&self) -> bool {
        self.slabs.is_empty()
    }

    /// Total covered area. `i64` because a die is easily 10^6 DBU on a side, and 10^12 does not
    /// fit in an `i32`.
    pub fn area(&self) -> i64 {
        self.slabs
            .iter()
            .map(|s| {
                let w = (s.x1 - s.x0) as i64;
                w * s.ys.iter().map(|(a, b)| (b - a) as i64).sum::<i64>()
            })
            .sum()
    }

    /// Drop empty slabs and merge neighbours that cover the same intervals, restoring the
    /// canonical form. Everything that builds a set ends here, which is what makes `==` mean
    /// "the same region" rather than "the same history".
    fn compact(&mut self) {
        self.slabs.retain(|s| !s.ys.is_empty() && s.x1 > s.x0);
        let mut out: Vec<Slab> = Vec::with_capacity(self.slabs.len());
        for s in self.slabs.drain(..) {
            match out.last_mut() {
                Some(last) if last.x1 == s.x0 && last.ys == s.ys => last.x1 = s.x1,
                _ => out.push(s),
            }
        }
        self.slabs = out;
    }

    /// The y-intervals covered in the slab starting at `x` — empty where nothing is covered.
    fn ys_at(&self, x: i32) -> &[(i32, i32)] {
        match self.slabs.binary_search_by(|s| {
            if s.x1 <= x {
                std::cmp::Ordering::Less
            } else if s.x0 > x {
                std::cmp::Ordering::Greater
            } else {
                std::cmp::Ordering::Equal
            }
        }) {
            Ok(i) => &self.slabs[i].ys,
            Err(_) => &[],
        }
    }

    fn boolean(&self, other: &Poly90Set, keep: fn(bool, bool) -> bool) -> Poly90Set {
        let mut xs: Vec<i32> = self
            .slabs
            .iter()
            .chain(other.slabs.iter())
            .flat_map(|s| [s.x0, s.x1])
            .collect();
        xs.sort_unstable();
        xs.dedup();

        let mut slabs = Vec::with_capacity(xs.len().saturating_sub(1));
        for w in xs.windows(2) {
            let (x0, x1) = (w[0], w[1]);
            let ys = combine(self.ys_at(x0), other.ys_at(x0), keep);
            slabs.push(Slab { x0, x1, ys });
        }
        let mut s = Poly90Set { slabs };
        s.compact();
        s
    }

    /// Everything covered by either set.
    pub fn union(&self, other: &Poly90Set) -> Poly90Set {
        self.boolean(other, |a, b| a || b)
    }
    /// Everything covered by this set and not the other.
    pub fn difference(&self, other: &Poly90Set) -> Poly90Set {
        self.boolean(other, |a, b| a && !b)
    }
    /// Everything covered by both.
    pub fn intersection(&self, other: &Poly90Set) -> Poly90Set {
        self.boolean(other, |a, b| a && b)
    }

    /// Grow the region by the given amounts in each direction (Minkowski dilation by a
    /// rectangle).
    ///
    /// Exact for a rectilinear set: growing every rectangle of the canonical decomposition and
    /// unioning the results is the dilation, because the set *is* that union.
    pub fn bloat(&self, west: i32, east: i32, south: i32, north: i32) -> Poly90Set {
        Poly90Set::from_rects(
            &self
                .rects()
                .into_iter()
                .map(|r| Rect::new(r.x0 - west, r.y0 - south, r.x1 + east, r.y1 + north))
                .collect::<Vec<_>>(),
        )
    }

    /// Shrink the region by the given amounts in each direction (Minkowski erosion).
    ///
    /// **Not** the same as shrinking each rectangle: two rectangles that abut form one region,
    /// and the shared edge must not erode. Computed by duality — erode the region by dilating its
    /// complement — which gets that right by construction.
    pub fn shrink(&self, west: i32, east: i32, south: i32, north: i32) -> Poly90Set {
        if self.is_empty() {
            return Poly90Set::default();
        }
        let (w, e, s, n) = (west.max(0), east.max(0), south.max(0), north.max(0));
        if w == 0 && e == 0 && s == 0 && n == 0 {
            return self.clone();
        }
        // A frame wide enough that the complement's dilation cannot reach in from outside.
        let b = self.bounds().expect("non-empty");
        let pad = w + e + s + n + 1;
        let frame =
            Poly90Set::from_rects(&[Rect::new(b.x0 - pad, b.y0 - pad, b.x1 + pad, b.y1 + pad)]);
        // Directions swap under complement: eroding the region from the west is dilating the
        // complement toward the east.
        let outside = frame.difference(self).bloat(e, w, n, s);
        frame.difference(&outside)
    }

    /// The bounding box, or `None` when the set is empty.
    pub fn bounds(&self) -> Option<Rect> {
        let first = self.slabs.first()?;
        let mut r = Rect::new(first.x0, i32::MAX, self.slabs.last()?.x1, i32::MIN);
        for s in &self.slabs {
            for &(y0, y1) in &s.ys {
                r.y0 = r.y0.min(y0);
                r.y1 = r.y1.max(y1);
            }
        }
        Some(r)
    }

    /// Keep only the connected pieces that **are** rectangles within the given size limits,
    /// inclusive.
    ///
    /// The fill algorithm uses this to discard partial shapes: a fill rectangle clipped by the
    /// area it was tiled into is no longer the right size, and a wrong-sized fill is a DRC
    /// violation rather than a smaller fill.
    pub fn keep_sized(&self, min_w: i32, max_w: i32, min_h: i32, max_h: i32) -> Poly90Set {
        let kept: Vec<Rect> = self
            .polygons()
            .into_iter()
            .filter_map(|p| {
                let xs = p.outer.iter().map(|q| q.x);
                let ys = p.outer.iter().map(|q| q.y);
                let (x0, x1) = (xs.clone().min()?, xs.max()?);
                let (y0, y1) = (ys.clone().min()?, ys.max()?);
                let (w, h) = (x1 - x0, y1 - y0);
                // The piece must BE the rectangle, not merely fit in one: an L-shaped offcut has
                // no holes and the right bounding box, and keeping it would emit a fill shape
                // that is not the size it claims. Four corners and no holes is exactly a
                // rectangle, for a rectilinear outline.
                let is_rectangle = p.outer.len() == 4 && p.holes.is_empty();
                (is_rectangle && (min_w..=max_w).contains(&w) && (min_h..=max_h).contains(&h))
                    .then_some(Rect::new(x0, y0, x1, y1))
            })
            .collect();
        Poly90Set::from_rects(&kept)
    }

    /// The canonical decomposition into disjoint rectangles (one per slab per interval).
    ///
    /// Useful when a caller wants area or coverage rather than shape — iterating these is much
    /// cheaper than tracing outlines.
    pub fn rects(&self) -> Vec<Rect> {
        self.slabs
            .iter()
            .flat_map(|s| s.ys.iter().map(|(y0, y1)| Rect::new(s.x0, *y0, s.x1, *y1)))
            .collect()
    }

    /// Does the set cover this point?
    pub fn contains(&self, x: i32, y: i32) -> bool {
        self.ys_at(x).iter().any(|(a, b)| *a <= y && y < *b)
    }

    /// Every boundary edge, directed so the interior is on the left.
    fn boundary_edges(&self) -> Vec<(Point, Point)> {
        let mut edges = Vec::new();
        // An empty set has no boundary. Returning early is not just an optimisation: the vertical
        // sweep below reads `slabs[i - 1]` when `i == slabs.len()`, which underflows at i = 0.
        if self.slabs.is_empty() {
            return edges;
        }

        // Horizontal: within a slab the coverage does not vary with x, so the bottom and top of
        // every interval are boundary for the slab's whole width.
        for s in &self.slabs {
            for &(y0, y1) in &s.ys {
                edges.push((Point::new(s.x0, y0), Point::new(s.x1, y0))); // bottom, +x
                edges.push((Point::new(s.x1, y1), Point::new(s.x0, y1))); // top, -x
            }
        }

        // Vertical: at every slab boundary, whatever the two sides disagree about is boundary.
        // The outside of the leftmost and rightmost slabs counts as empty, which is what closes
        // the outline.
        let empty: Ivals = Vec::new();
        for i in 0..=self.slabs.len() {
            let left = if i == 0 {
                &empty[..]
            } else {
                &self.slabs[i - 1].ys[..]
            };
            let right = if i == self.slabs.len() {
                &empty[..]
            } else {
                &self.slabs[i].ys[..]
            };
            let x = if i == self.slabs.len() {
                self.slabs[i - 1].x1
            } else {
                self.slabs[i].x0
            };
            // A gap between non-adjacent slabs is already handled: the right slab's x0 differs
            // from the left slab's x1, so each is compared against emptiness in its own turn.
            if i > 0 && i < self.slabs.len() && self.slabs[i - 1].x1 != self.slabs[i].x0 {
                for &(y0, y1) in &self.slabs[i - 1].ys {
                    edges.push((
                        Point::new(self.slabs[i - 1].x1, y0),
                        Point::new(self.slabs[i - 1].x1, y1),
                    ));
                }
                for &(y0, y1) in &self.slabs[i].ys {
                    edges.push((
                        Point::new(self.slabs[i].x0, y1),
                        Point::new(self.slabs[i].x0, y0),
                    ));
                }
                continue;
            }
            for &(y0, y1) in &combine(left, right, |a, b| a && !b) {
                edges.push((Point::new(x, y0), Point::new(x, y1))); // interior left of it, +y
            }
            for &(y0, y1) in &combine(left, right, |a, b| !a && b) {
                edges.push((Point::new(x, y1), Point::new(x, y0))); // interior right of it, -y
            }
        }
        edges
    }

    /// Trace the set's outlines: outer boundaries counter-clockwise, holes clockwise.
    fn outlines(&self) -> Vec<Outline> {
        use std::collections::HashMap;
        let edges = self.boundary_edges();
        let mut out_by_start: HashMap<Point, Vec<Point>> = HashMap::new();
        for (a, b) in &edges {
            out_by_start.entry(*a).or_default().push(*b);
        }

        let mut loops = Vec::new();
        let mut remaining: usize = edges.len();

        while remaining > 0 {
            // Start anywhere an edge is still unused.
            let start = *out_by_start
                .iter()
                .find(|(_, v)| !v.is_empty())
                .expect("an unused edge exists while remaining > 0")
                .0;
            let mut pts: Vec<Point> = Vec::new();
            let mut cur = start;
            let mut incoming: Option<(i32, i32)> = None;

            loop {
                let outs = out_by_start
                    .get_mut(&cur)
                    .expect("a traced point has an exit");
                if outs.is_empty() {
                    break;
                }
                // At a pinch point two loops meet. Taking the LEFTMOST turn keeps them separate,
                // which is what makes two rectangles touching at a corner two polygons rather
                // than one figure-eight.
                let pick = match incoming {
                    _ if outs.len() == 1 => 0,
                    None => 0,
                    Some((ix, iy)) => outs
                        .iter()
                        .enumerate()
                        .max_by_key(|(_, n)| {
                            let (ox, oy) = ((n.x - cur.x).signum(), (n.y - cur.y).signum());
                            let cross = ix * oy - iy * ox;
                            let dot = ix * ox + iy * oy;
                            // left(+1) > straight(0) > right(-1) > reverse
                            match (cross, dot) {
                                (c, _) if c > 0 => 3,
                                (0, d) if d > 0 => 2,
                                (c, _) if c < 0 => 1,
                                _ => 0,
                            }
                        })
                        .map(|(i, _)| i)
                        .unwrap_or(0),
                };
                let next = outs.swap_remove(pick);
                remaining -= 1;
                pts.push(cur);
                incoming = Some(((next.x - cur.x).signum(), (next.y - cur.y).signum()));
                cur = next;
                if cur == start {
                    break;
                }
            }
            // Drop the collinear midpoints the slab decomposition introduces: a straight run
            // split across two slabs is one edge of the shape, not two.
            let simplified = simplify(&pts);
            if simplified.len() >= 4 {
                loops.push(simplified);
            }
        }
        loops
    }

    /// The set as polygons, each with its holes.
    pub fn polygons(&self) -> Vec<Polygon90> {
        let loops = self.outlines();
        let (outers, holes): (Vec<_>, Vec<_>) =
            loops.into_iter().partition(|l| signed_area2(l) > 0);

        let mut polys: Vec<Polygon90> = outers
            .into_iter()
            .map(|outer| Polygon90 {
                outer,
                holes: Vec::new(),
            })
            .collect();

        for hole in holes {
            // A hole belongs to the smallest outer boundary that contains it — smallest, because
            // an island inside a hole inside a polygon would otherwise claim it.
            let probe = hole[0];
            let best = polys
                .iter()
                .enumerate()
                .filter(|(_, p)| point_in_outline(&p.outer, probe))
                .min_by_key(|(_, p)| signed_area2(&p.outer).abs())
                .map(|(i, _)| i);
            match best {
                Some(i) => polys[i].holes.push(hole),
                // Cannot happen for a well-formed set; keeping it as an outline of its own is
                // better than dropping geometry on the floor.
                None => polys.push(Polygon90 {
                    outer: hole,
                    holes: Vec::new(),
                }),
            }
        }
        polys
    }
}

/// Remove points that sit in the middle of a straight run, and the duplicate closing point.
fn simplify(pts: &[Point]) -> Outline {
    let n = pts.len();
    if n < 3 {
        return pts.to_vec();
    }
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let prev = pts[(i + n - 1) % n];
        let cur = pts[i];
        let next = pts[(i + 1) % n];
        let collinear =
            (prev.x == cur.x && cur.x == next.x) || (prev.y == cur.y && cur.y == next.y);
        if !collinear {
            out.push(cur);
        }
    }
    out
}

/// Twice the signed area (the shoelace sum). Positive is counter-clockwise.
///
/// Doubled and in `i64` so it stays exact: halving would need a rounding rule, and the sign is
/// all the caller wants.
pub fn signed_area2(outline: &[Point]) -> i64 {
    let n = outline.len();
    (0..n)
        .map(|i| {
            let a = outline[i];
            let b = outline[(i + 1) % n];
            a.x as i64 * b.y as i64 - b.x as i64 * a.y as i64
        })
        .sum()
}

/// Is the point strictly inside the outline? Crossing count on a ray to +x.
fn point_in_outline(outline: &[Point], p: Point) -> bool {
    let n = outline.len();
    let mut inside = false;
    for i in 0..n {
        let (a, b) = (outline[i], outline[(i + 1) % n]);
        if (a.y > p.y) != (b.y > p.y) {
            let dy = (b.y - a.y) as i64;
            let t = (p.y - a.y) as i64;
            let x_at = a.x as i64 + (b.x - a.x) as i64 * t / dy;
            if x_at > p.x as i64 {
                inside = !inside;
            }
        }
    }
    inside
}

#[cfg(test)]
mod tests {
    use super::*;

    fn set(rects: &[(i32, i32, i32, i32)]) -> Poly90Set {
        Poly90Set::from_rects(
            &rects
                .iter()
                .map(|&(a, b, c, d)| Rect::new(a, b, c, d))
                .collect::<Vec<_>>(),
        )
    }

    #[test]
    fn a_single_rectangle_traces_counter_clockwise() {
        let s = set(&[(0, 0, 10, 10)]);
        assert_eq!(s.area(), 100);
        let polys = s.polygons();
        assert_eq!(polys.len(), 1);
        assert!(polys[0].holes.is_empty());
        assert_eq!(
            polys[0].outer.len(),
            4,
            "four corners, no collinear midpoints"
        );
        assert!(
            signed_area2(&polys[0].outer) > 0,
            "outer boundaries are counter-clockwise"
        );
        assert_eq!(signed_area2(&polys[0].outer), 200, "twice the area");
    }

    #[test]
    fn the_representation_is_canonical_whatever_order_rectangles_arrive_in() {
        // The same region built three ways must compare equal, or `==` means nothing and the
        // boolean ops cannot be trusted to have simplified.
        let a = set(&[(0, 0, 10, 10), (10, 0, 20, 10)]);
        let b = set(&[(10, 0, 20, 10), (0, 0, 10, 10)]);
        let c = set(&[(0, 0, 20, 10)]);
        assert_eq!(a, b);
        assert_eq!(a, c, "two abutting halves ARE one rectangle");
        assert_eq!(a.rects().len(), 1);
    }

    #[test]
    fn overlapping_rectangles_are_counted_once() {
        let s = set(&[(0, 0, 10, 10), (5, 5, 15, 15)]);
        assert_eq!(s.area(), 100 + 100 - 25);
        assert_eq!(s.polygons().len(), 1, "they overlap, so it is one shape");
    }

    #[test]
    fn an_l_shape_keeps_its_six_corners() {
        let s = set(&[(0, 0, 10, 4), (0, 0, 4, 10)]);
        let polys = s.polygons();
        assert_eq!(polys.len(), 1);
        assert_eq!(polys[0].outer.len(), 6, "an L has six corners");
        assert!(signed_area2(&polys[0].outer) > 0);
        assert_eq!(s.area(), 40 + 40 - 16);
    }

    #[test]
    fn disjoint_rectangles_are_separate_polygons() {
        let s = set(&[(0, 0, 10, 10), (20, 0, 30, 10)]);
        let polys = s.polygons();
        assert_eq!(polys.len(), 2);
        assert!(polys
            .iter()
            .all(|p| p.outer.len() == 4 && p.holes.is_empty()));
    }

    #[test]
    fn a_hole_is_found_and_wound_the_other_way() {
        // The case the whole module exists for: a region with something carved out of the middle.
        let outer = set(&[(0, 0, 30, 30)]);
        let s = outer.difference(&set(&[(10, 10, 20, 20)]));
        assert_eq!(s.area(), 900 - 100);

        let polys = s.polygons();
        assert_eq!(polys.len(), 1, "one shape...");
        assert_eq!(polys[0].holes.len(), 1, "...with one hole");
        assert!(signed_area2(&polys[0].outer) > 0, "outer counter-clockwise");
        assert!(
            signed_area2(&polys[0].holes[0]) < 0,
            "a hole winds the other way, which is what tells it from an island"
        );
        assert_eq!(polys[0].holes[0].len(), 4);
        assert_eq!(signed_area2(&polys[0].holes[0]).abs(), 200);
    }

    #[test]
    fn a_notch_cut_from_the_edge_is_not_a_hole() {
        // Same subtraction, but touching the boundary: the result is one outline with eight
        // corners, not an outline plus a hole.
        let s = set(&[(0, 0, 30, 30)]).difference(&set(&[(10, 20, 20, 30)]));
        let polys = s.polygons();
        assert_eq!(polys.len(), 1);
        assert!(
            polys[0].holes.is_empty(),
            "it reaches the edge, so it is a notch"
        );
        assert_eq!(polys[0].outer.len(), 8);
    }

    #[test]
    fn subtracting_everything_leaves_nothing() {
        let s = set(&[(0, 0, 10, 10)]).difference(&set(&[(0, 0, 10, 10)]));
        assert!(s.is_empty());
        assert_eq!(s.area(), 0);
        assert!(s.polygons().is_empty());
        assert!(!s.contains(5, 5));
    }

    #[test]
    fn intersection_keeps_only_the_shared_part() {
        let a = set(&[(0, 0, 10, 10)]);
        let b = set(&[(5, 5, 15, 15)]);
        let i = a.intersection(&b);
        assert_eq!(i, set(&[(5, 5, 10, 10)]));
        assert!(i.contains(6, 6));
        assert!(!i.contains(4, 4));
        // Disjoint inputs intersect in nothing.
        assert!(a.intersection(&set(&[(100, 100, 110, 110)])).is_empty());
    }

    #[test]
    fn two_rectangles_touching_at_a_corner_stay_two_polygons() {
        // The pinch point. Tracing has a choice at (10,10) and the wrong one produces a single
        // figure-eight outline that no downstream consumer can interpret.
        let s = set(&[(0, 0, 10, 10), (10, 10, 20, 20)]);
        let polys = s.polygons();
        assert_eq!(
            polys.len(),
            2,
            "they meet at one point, so they are two shapes"
        );
        assert!(polys.iter().all(|p| p.outer.len() == 4));
        assert_eq!(s.area(), 200);
    }

    #[test]
    fn a_stack_of_abutting_rows_is_one_region() {
        // Why intervals are half-open. Rows abut exactly; if they did not merge, the row area of
        // every real design would come out as thousands of separate shapes.
        let rows: Vec<Rect> = (0..50)
            .map(|i| Rect::new(0, i * 10, 100, i * 10 + 10))
            .collect();
        let s = Poly90Set::from_rects(&rows);
        assert_eq!(
            s.rects(),
            vec![Rect::new(0, 0, 100, 500)],
            "one rectangle, not fifty"
        );
        assert_eq!(s.polygons().len(), 1);
        assert_eq!(s.polygons()[0].outer.len(), 4);
    }

    #[test]
    fn the_row_region_of_a_block_with_a_macro_comes_out_as_one_shape_with_a_hole() {
        // The `tap` use case end to end: rows tile the core, a macro sits in the middle, and the
        // rows crossing it have been cut. What is left must be one polygon with one hole.
        let core = Rect::new(0, 0, 1000, 1000);
        let macro_ = Rect::new(400, 400, 600, 600);
        let mut rows = Vec::new();
        for i in 0..100 {
            let (y0, y1) = (i * 10, i * 10 + 10);
            if y1 <= macro_.y0 || y0 >= macro_.y1 {
                rows.push(Rect::new(core.x0, y0, core.x1, y1));
            } else {
                rows.push(Rect::new(core.x0, y0, macro_.x0, y1));
                rows.push(Rect::new(macro_.x1, y0, core.x1, y1));
            }
        }
        let s = Poly90Set::from_rects(&rows);
        assert_eq!(s.area(), 1_000_000 - 200 * 200);

        let polys = s.polygons();
        assert_eq!(polys.len(), 1);
        assert_eq!(
            polys[0].holes.len(),
            1,
            "the macro is a hole in the row region"
        );
        assert_eq!(
            polys[0].outer.len(),
            4,
            "the outside is still a plain rectangle"
        );
        assert_eq!(signed_area2(&polys[0].holes[0]).abs(), 2 * 200 * 200);
    }

    #[test]
    fn the_upstream_mask_dance_is_just_an_intersection() {
        // OpenROAD's getBoundaryAreas computes the row region as
        //     mask = core - rows;  region = core - mask
        // because Boost's polygon set is easier to drive that way. With a real set type that is
        // just `core ∩ rows`, and this pins the two together so the simplification is checked
        // rather than assumed.
        let core = set(&[(0, 0, 100, 100)]);
        let rows = set(&[(10, 0, 90, 40), (10, 60, 90, 100), (-20, 45, 120, 55)]);

        let mask = core.difference(&rows);
        let via_mask = core.difference(&mask);
        assert_eq!(via_mask, core.intersection(&rows));
        // ...and it really is the rows, clipped to the core: nothing outside survives.
        assert!(
            !via_mask.contains(-5, 50),
            "the row overhanging the core is clipped"
        );
        assert!(via_mask.contains(0, 50));
    }

    #[test]
    fn an_island_inside_a_hole_is_its_own_polygon() {
        // Ring with something in the middle of the gap. The island must NOT be recorded as a
        // hole of the ring — it is solid.
        let ring = set(&[(0, 0, 30, 30)]).difference(&set(&[(5, 5, 25, 25)]));
        let s = ring.union(&set(&[(10, 10, 20, 20)]));
        let polys = s.polygons();
        assert_eq!(polys.len(), 2, "the ring and the island");
        let island = polys
            .iter()
            .find(|p| p.outer.len() == 4 && p.holes.is_empty());
        assert!(island.is_some(), "the island stands alone");
        let ring_poly = polys
            .iter()
            .find(|p| !p.holes.is_empty())
            .expect("the ring has a hole");
        assert_eq!(ring_poly.holes.len(), 1);
    }

    #[test]
    fn empty_and_degenerate_inputs_are_handled_rather_than_panicking() {
        assert!(Poly90Set::from_rects(&[]).is_empty());
        assert!(
            Poly90Set::from_rects(&[Rect::new(5, 5, 5, 10)]).is_empty(),
            "zero width"
        );
        assert!(
            Poly90Set::from_rects(&[Rect::new(5, 5, 10, 5)]).is_empty(),
            "zero height"
        );
        assert!(
            Poly90Set::from_rects(&[Rect::new(10, 10, 0, 0)]).is_empty(),
            "inverted"
        );
        let s = Poly90Set::new();
        assert_eq!(s.union(&set(&[(0, 0, 1, 1)])), set(&[(0, 0, 1, 1)]));
        assert!(s.difference(&set(&[(0, 0, 1, 1)])).is_empty());
    }

    #[test]
    fn union_and_intersection_agree_with_area_arithmetic() {
        // |A| + |B| == |A ∪ B| + |A ∩ B|, on a shape neither trivial nor symmetric.
        let a = set(&[(0, 0, 10, 30), (0, 0, 30, 10)]);
        let b = set(&[(5, 5, 25, 25)]);
        assert_eq!(
            a.area() + b.area(),
            a.union(&b).area() + a.intersection(&b).area()
        );
        // ...and difference is the union minus the other side's share.
        assert_eq!(
            a.difference(&b).area(),
            a.area() - a.intersection(&b).area()
        );
    }

    #[test]
    fn contains_agrees_with_the_half_open_convention() {
        let s = set(&[(0, 0, 10, 10)]);
        assert!(s.contains(0, 0), "the low corner is inside");
        assert!(s.contains(9, 9));
        assert!(!s.contains(10, 5), "the high edge is not");
        assert!(!s.contains(5, 10));
        assert!(!s.contains(-1, 5));
    }
}

#[cfg(test)]
mod sizing_tests {
    use super::*;

    fn set(rects: &[(i32, i32, i32, i32)]) -> Poly90Set {
        Poly90Set::from_rects(
            &rects
                .iter()
                .map(|&(a, b, c, d)| Rect::new(a, b, c, d))
                .collect::<Vec<_>>(),
        )
    }

    #[test]
    fn bloating_grows_each_side_independently() {
        let s = set(&[(10, 20, 30, 40)]).bloat(1, 2, 3, 4);
        assert_eq!(s.rects(), vec![Rect::new(9, 17, 32, 44)]);
    }

    #[test]
    fn bloating_merges_shapes_that_grow_into_each_other() {
        // Two rectangles 4 apart, grown 2 each way, become one.
        let s = set(&[(0, 0, 10, 10), (14, 0, 24, 10)]).bloat(2, 2, 0, 0);
        assert_eq!(
            s.rects(),
            vec![Rect::new(-2, 0, 26, 10)],
            "one shape, not two"
        );
        assert_eq!(s.polygons().len(), 1);
    }

    #[test]
    fn shrinking_does_not_erode_a_shared_edge() {
        // The reason shrink is not per-rectangle: these two abut, so the region is one 20-wide
        // rectangle and shrinking by 2 leaves 16 — not two 6-wide pieces with a gap.
        let s = set(&[(0, 0, 10, 10), (10, 0, 20, 10)]).shrink(2, 2, 0, 0);
        assert_eq!(s.rects(), vec![Rect::new(2, 0, 18, 10)]);
    }

    #[test]
    fn shrinking_past_the_extent_leaves_nothing() {
        assert!(set(&[(0, 0, 10, 10)]).shrink(6, 6, 0, 0).is_empty());
        assert!(set(&[(0, 0, 10, 10)]).shrink(0, 0, 20, 20).is_empty());
        assert!(Poly90Set::new().shrink(1, 1, 1, 1).is_empty());
    }

    #[test]
    fn shrink_then_bloat_drops_what_was_too_thin_to_survive() {
        // The fill algorithm's actual use: a shrink/bloat cycle removes anything too small to
        // hold the shape about to be placed, and restores the rest to full size.
        let s = set(&[(0, 0, 100, 100), (200, 40, 204, 60)]); // a block plus a detached 4-wide sliver
        let cycled = s.shrink(3, 3, 3, 3).bloat(3, 3, 3, 3);
        assert!(cycled.contains(50, 50), "the body survives");
        assert!(
            !cycled.contains(202, 50),
            "the detached sliver is too thin and goes"
        );
        assert_eq!(
            cycled.bounds(),
            Some(Rect::new(0, 0, 100, 100)),
            "and nothing else moved"
        );
    }

    #[test]
    fn a_thin_spur_attached_to_a_body_is_not_removed_by_the_cycle() {
        // Worth pinning because it is the intuitive-but-wrong reading of shrink/bloat: erosion is
        // about NEIGHBOURHOODS, so points in a narrow spur next to a wide body still have room
        // around them and survive, then bloat restores part of the spur. Only DETACHED slivers
        // vanish.
        let s = set(&[(0, 0, 100, 100), (100, 40, 104, 60)]);
        let cycled = s.shrink(3, 3, 3, 3).bloat(3, 3, 3, 3);
        assert!(
            cycled.contains(101, 50),
            "the spur next to the body partly survives"
        );
    }

    #[test]
    fn a_no_op_sizing_returns_the_same_region() {
        let s = set(&[(0, 0, 10, 10), (20, 20, 30, 30)]);
        assert_eq!(s.bloat(0, 0, 0, 0), s);
        assert_eq!(s.shrink(0, 0, 0, 0), s);
    }

    #[test]
    fn bounds_covers_every_piece() {
        let s = set(&[(0, 0, 10, 10), (50, 60, 70, 80)]);
        assert_eq!(s.bounds(), Some(Rect::new(0, 0, 70, 80)));
        assert_eq!(Poly90Set::new().bounds(), None);
    }

    #[test]
    fn the_size_filter_keeps_only_whole_shapes() {
        // Fill tiles clipped by their area are the wrong size, and a wrong-sized fill is a DRC
        // violation rather than a smaller fill.
        let s = set(&[(0, 0, 10, 10), (20, 0, 26, 10), (40, 0, 50, 10)]);
        let kept = s.keep_sized(10, 10, 10, 10);
        assert_eq!(
            kept.rects(),
            vec![Rect::new(0, 0, 10, 10), Rect::new(40, 0, 50, 10)]
        );
        assert!(!kept.contains(22, 5), "the 6-wide offcut is dropped");
        // A piece with a hole is not a whole shape either.
        let ring = set(&[(0, 0, 30, 30)]).difference(&set(&[(10, 10, 20, 20)]));
        assert!(ring.keep_sized(30, 30, 30, 30).is_empty());

        // Nor is an L: it has no holes and a 10x10 bounding box, but it is not the shape.
        let ell = set(&[(0, 0, 10, 4), (0, 0, 4, 10)]);
        assert!(
            ell.keep_sized(10, 10, 10, 10).is_empty(),
            "a piece must BE the rectangle, not merely fit inside one"
        );
    }

    #[test]
    fn shrinking_agrees_with_a_brute_force_erosion() {
        // Erosion is "every point whose neighbourhood is inside", checked directly on a grid.
        const N: i32 = 30;
        let s = set(&[(2, 2, 20, 12), (12, 12, 28, 20)]);
        let (w, e, so, no) = (2, 3, 1, 2);
        let got = s.shrink(w, e, so, no);
        for x in 0..N {
            for y in 0..N {
                let inside_all =
                    (x - w..=x + e).all(|px| (y - so..=y + no).all(|py| s.contains(px, py)));
                assert_eq!(got.contains(x, y), inside_all, "({x},{y})");
            }
        }
    }
}

/// Property tests. This module is substrate several engines will trust, and the failure mode that
/// matters — a boundary traced slightly wrong — produces plausible-looking output that no example
/// test happens to cover. So the properties are checked against an INDEPENDENT brute force over a
/// small grid, which shares no code with the slab decomposition.
#[cfg(test)]
mod properties {
    use super::*;

    /// A deterministic generator: reproducible failures beat unreproducible coverage.
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self, n: i32) -> i32 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((self.0 >> 33) % n as u64) as i32
        }
        fn rects(&mut self, count: usize, span: i32) -> Vec<Rect> {
            (0..count)
                .map(|_| {
                    let (x0, y0) = (self.next(span), self.next(span));
                    let (w, h) = (1 + self.next(span / 2), 1 + self.next(span / 2));
                    // Clamped to the grid the brute force covers -- otherwise the set would
                    // legitimately count area outside it and the comparison would be unfair.
                    Rect::new(x0, y0, (x0 + w).min(span), (y0 + h).min(span))
                })
                .collect()
        }
    }

    /// Coverage computed the dumb way: one boolean per unit cell.
    fn brute(rects: &[Rect], span: i32) -> Vec<bool> {
        let mut g = vec![false; (span * span) as usize];
        for r in rects {
            for x in r.x0.max(0)..r.x1.min(span) {
                for y in r.y0.max(0)..r.y1.min(span) {
                    g[(x * span + y) as usize] = true;
                }
            }
        }
        g
    }

    #[test]
    fn area_and_containment_agree_with_brute_force() {
        const SPAN: i32 = 40;
        let mut rng = Lcg(12345);
        for case in 0..200 {
            let rects = rng.rects(1 + (case % 7), SPAN);
            let s = Poly90Set::from_rects(&rects);
            let g = brute(&rects, SPAN);

            let expected: i64 = g.iter().filter(|c| **c).count() as i64;
            assert_eq!(
                s.area(),
                expected,
                "case {case}: area disagrees with the grid"
            );

            for x in 0..SPAN {
                for y in 0..SPAN {
                    assert_eq!(
                        s.contains(x, y),
                        g[(x * SPAN + y) as usize],
                        "case {case}: contains({x},{y}) disagrees with the grid"
                    );
                }
            }
        }
    }

    #[test]
    fn the_traced_outlines_enclose_exactly_the_covered_area() {
        // The real check on the tracer: shoelace over every outline (outers positive, holes
        // negative) must total the area computed from the slabs. A mis-traced boundary, a missed
        // hole, or a hole attached to the wrong polygon all break this.
        const SPAN: i32 = 40;
        let mut rng = Lcg(999);
        for case in 0..200 {
            let s = Poly90Set::from_rects(&rng.rects(1 + (case % 6), SPAN));
            let from_outlines: i64 = s
                .polygons()
                .iter()
                .map(|p| {
                    signed_area2(&p.outer) + p.holes.iter().map(|h| signed_area2(h)).sum::<i64>()
                })
                .sum();
            assert_eq!(
                from_outlines,
                2 * s.area(),
                "case {case}: the outlines do not enclose the area the slabs report"
            );
        }
    }

    #[test]
    fn every_outer_is_counter_clockwise_and_every_hole_is_not() {
        const SPAN: i32 = 30;
        let mut rng = Lcg(4242);
        for case in 0..200 {
            let s = Poly90Set::from_rects(&rng.rects(1 + (case % 5), SPAN));
            for p in s.polygons() {
                assert!(
                    signed_area2(&p.outer) > 0,
                    "case {case}: an outer wound clockwise"
                );
                assert!(
                    p.outer.len() >= 4 && p.outer.len() % 2 == 0,
                    "case {case}: odd outline"
                );
                for h in &p.holes {
                    assert!(
                        signed_area2(h) < 0,
                        "case {case}: a hole wound counter-clockwise"
                    );
                }
            }
        }
    }

    #[test]
    fn the_canonical_form_is_a_fixed_point() {
        // Decomposing to rectangles and rebuilding must give the identical representation, or
        // `==` is comparing history rather than geometry.
        let mut rng = Lcg(777);
        for case in 0..200 {
            let s = Poly90Set::from_rects(&rng.rects(1 + (case % 8), 40));
            assert_eq!(Poly90Set::from_rects(&s.rects()), s, "case {case}");
        }
    }

    #[test]
    fn sizing_round_trips_and_orders_correctly() {
        // Bloat then shrink by the same amount recovers a set that lost nothing (it may gain,
        // where a bloat merged two pieces), and shrink never grows the region.
        let mut rng = Lcg(20260812);
        for case in 0..120 {
            let s = Poly90Set::from_rects(&rng.rects(1 + (case % 5), 40));
            let (w, e, so, no) = (case % 4, (case + 1) % 3, case % 3, (case + 2) % 4);
            let (w, e, so, no) = (w as i32, e as i32, so as i32, no as i32);

            let grown = s.bloat(w, e, so, no);
            assert!(
                grown.area() >= s.area(),
                "case {case}: bloat shrank the region"
            );
            assert_eq!(
                grown.union(&s),
                grown,
                "case {case}: bloat must contain the original"
            );

            let eroded = s.shrink(w, e, so, no);
            assert!(
                eroded.area() <= s.area(),
                "case {case}: shrink grew the region"
            );
            assert_eq!(
                eroded.union(&s),
                s,
                "case {case}: shrink must stay inside the original"
            );

            // Closing (bloat then shrink) never loses a point of the original.
            let closed = grown.shrink(w, e, so, no);
            assert_eq!(
                closed.union(&s),
                closed,
                "case {case}: closing lost part of the region"
            );
        }
    }

    #[test]
    fn the_boolean_identities_hold() {
        let mut rng = Lcg(31337);
        for case in 0..150 {
            let a = Poly90Set::from_rects(&rng.rects(1 + (case % 4), 40));
            let b = Poly90Set::from_rects(&rng.rects(1 + (case % 3), 40));

            assert_eq!(
                a.union(&b),
                b.union(&a),
                "case {case}: union is commutative"
            );
            assert_eq!(
                a.intersection(&b),
                b.intersection(&a),
                "case {case}: intersection is commutative"
            );
            assert_eq!(a.union(&a), a, "case {case}: union is idempotent");
            assert!(a.difference(&a).is_empty(), "case {case}: A - A is empty");
            assert_eq!(
                a.area() + b.area(),
                a.union(&b).area() + a.intersection(&b).area(),
                "case {case}: inclusion-exclusion"
            );
            // (A - B) and (A ∩ B) partition A, with nothing shared and nothing lost.
            assert_eq!(
                a.difference(&b).union(&a.intersection(&b)),
                a,
                "case {case}: difference and intersection partition A"
            );
            assert!(a.difference(&b).intersection(&b).is_empty(), "case {case}");
        }
    }
}
