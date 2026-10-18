FILE = "src/lrs_psenak_valko/include/lrs_psenak_valko/voxel_occupancy_grid.h"
TITLE = "Sparse voxel grid (voxel_occupancy_grid.h, A1.1)"
INTRO = """<p>Stores only the <b>occupied</b> voxels, in a hash set. "Free" = not in the set. This
needs little memory for a big, mostly empty hangar. It fills the racks and inflates the obstacles
by the safety radius.</p>"""

NOTES = [
("// Copyright (c) 2026 STU FEI URK", "Header comment",
 """<p>Lists the operations and their order: <code>build()</code> → <code>addBox()</code>* →
 <code>inflate()</code>.</p>"""),
("#pragma once", "Includes", """<p>Standard containers and PCL point types.</p>"""),
("class VoxelOccupancyGrid", "IndexBounds and constructor",
 """<p><code>IndexBounds</code>: inclusive min/max voxel index range. The constructor stores the
 voxel size.</p>"""),
("void build(const pcl::PointCloud<pcl::PointXYZ>& cloud)", "build(): points → voxels",
 """<p>Every point's voxel index is encoded into one 64-bit key and inserted into the set
 (duplicates vanish automatically). The inflated layer starts as a copy.</p>"""),
("/// Fill a solid box", "addBox(): fill a rack",
 """<p>For a box rotated by yaw: loop over the voxels of its axis-aligned bounding box, rotate each
 voxel centre into the box frame and keep it if it is inside the half-sizes. Returns how many new
 voxels were added. Racks are solid in the map because the point cloud only has their
 surfaces.</p>"""),
("/// Grow every occupied voxel into a sphere of radius_m", "inflate(): the safety margin",
 """<p>Converts the radius to voxels (0.45 / 0.15 = 3), precomputes once all offsets
 (dx, dy, dz) inside that sphere, then adds every occupied voxel + every offset to the inflated set.
 It is spherical and 3D, so it also covers above and below obstacles. After this the drone can be
 treated as a point.</p>"""),
("// ---- queries in world coordinates", "Queries",
 """<p>"Is this world point occupied?" in the raw or the inflated layer, and the index version used
 to fill A*'s dense grid.</p>"""),
("pcl::PointCloud<pcl::PointXYZ> inflatedPointCloud() const", "inflatedPointCloud()",
 """<p>Turns the inflated voxels back into points (voxel centres), for RViz and the saved PCD.</p>"""),
("/// Index range covered by the inflated map", "inflatedBounds() and counters",
 """<p>Min/max index over all inflated voxels (needed to size the dense grid), and simple counters
 for log messages.</p>"""),
("private:", "toIndex()", """<p>World coordinate → voxel index: <code>floor(coord / size)</code>.</p>"""),
("// Pack three voxel indices into one hashable key", "Encoding three indices in one integer",
 """<p>Each index gets +2<sup>20</sup> (so negatives become positive) and 21 bits; x is shifted by
 42, y by 21: <code>key = x·2⁴² | y·2²¹ | z</code>. Decoding masks and shifts back. One integer key
 is fast to hash.</p>"""),
("float leaf_size_;", "Members",
 """<p>Voxel size, the raw set and the inflated set.</p>"""),
]
