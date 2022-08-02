"""Tools to complete and combine meshes and pointclouds
"""
import completiontools.utils as ut
import geomapi
import open3d as o3d
import numpy as np

from typing import Tuple, List

# The main function to combine 2 aligned geometries
def combine_geometry(ogGeometry: o3d.geometry.Geometry, newGeometry :o3d.geometry.Geometry, distanceTreshold : float = 0.05) -> o3d.geometry.Geometry:
    """Combines 2 aligned geometries assuming the ogGeometry is the reference and the newGeometry will suplement it.
    this is performed in a few steps:
        1) Create a convex hull of the newGeometry
        2) Filter the relevant points of the ogGeometry
        3) Perform a 2 step relevance check on the ogPoints if they fail either, they will be removed
            3a) Coverage check: perform a distance query, points that to far away from the mesh could be either out of date or not scanned
            3b) Visibility check: points that are inside the mesh are considered invisible and are kept, visible points are deemed out of date and are removed.
        4) Filter the newPoints to only add the points that are changed
        5) combine the changed-newPoints, the invisible-not-covered-ogPoints, the covered-ogPoints and the irrelevant points

    Args:
        ogGeometry (o3d.geometry): The reference geometry to be completed
        newGeometry (o3d.geometry): _description_
        distanceTreshold (float, optional): _description_. Defaults to 0.05.

    Returns:
        o3d.geometry: the combined geometry
    """

    # Step 1: Create a convex hull of the newGeometry
    newGeoHull = get_convex_hull(newGeometry)
    print("Covex hull created")
    # Step 2: Filter out the irrelevant points in the ogGeometry
    relevantOg, irrelevantOg = get_points_in_hull(ogGeometry, newGeoHull)
    print("Irrelevant points filtered")
    # Step 3: Isolate the not covered points of the ogGeometry compared to the newGeometry
    newGeometryPoints = newGeometry.sample_points_poisson_disk(number_of_points=100000)
    coveredPoints, unCoveredPoints = ut.filter_pcd_by_distance(relevantOg, newGeometryPoints, distanceTreshold)
    print("Covered poinys calculated")
    # Step 4: Perform the visibility check of the not covered points
    invisibleUncoveredPoints = get_invisible_points(unCoveredPoints, newGeometry)
    print("invisible points detected")
    # Step 5: Filter the newGeometryPoints to only keep the changed geometry
    existingNewGeo, newNewGeo = ut.filter_pcd_by_distance(newGeometryPoints, relevantOg, distanceTreshold)
    print("new points filtered")
    # Step 6: Combine the irrelevant, unchanged and changed geometry
    newCombinedGeometry = irrelevantOg + coveredPoints + invisibleUncoveredPoints + newNewGeo
    print("geometries combined")
    return newCombinedGeometry

# Converts a geometry to a covex hull
def get_convex_hull(geometry : o3d.geometry.Geometry) ->  o3d.geometry.Geometry:
    """Calculates a convex hull of a generic geometry

    Args:
        geometry (open3d.geometry): The geometry to be hulled

    Returns:
        open3d.geometry: The resulting hull
    """

    hull, _ = geometry.compute_convex_hull()
    return hull

# Returns a filtered mpcd with only points which are inside the convex hull
def get_points_in_hull(geometry : o3d.geometry.Geometry, hull: o3d.geometry.Geometry) -> Tuple[o3d.geometry.Geometry,o3d.geometry.Geometry]:
    """Separates a geometry in points inside and outside a convex hull

    Args:
        geometry (open3d.geometry): The geometry to be filtered
        hull (open3d.geometry): The hull to filter

    Returns:
       Tuple[open3d.geometry, open3d.geometry]: Points inside the hull, Points outside the hull
    """

    hullVerts = np.asarray(hull.vertices)
    points = np.asarray(geometry.points)
    idxs = ut.get_indices_in_hull(points, hullVerts)
    pcdInHull = geometry.select_by_index(idxs)
    pcdOutHull = geometry.select_by_index(idxs, True)
    return pcdInHull, pcdOutHull

# checks if the points is inside the (closed) mesh
def check_point_inside_mesh(points: List, mesh :o3d.geometry.Geometry) -> Tuple[List, List]:
    """Performs a visibility check to chack if the points are inside the mesh or not

    Args:
        points (List): The points to check
        mesh (open3d.geometry): The mesh to check against

    Returns:
        Tuple[List, List]: The points withing the mesh, the points outside the mesh
    """

    # step 0: Set up a raycasting scene
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    insideList = []
    outsideList = []

    for point in points:
        # step 1: get the closest point to the mesh
        queryPoint = o3d.core.Tensor([point], dtype=o3d.core.Dtype.Float32)
        queryResult = scene.compute_closest_points(queryPoint)
        closestPoint =  queryResult['points'].numpy()
        # step 2 Get the normal of the triangle
        closestTriangle = queryResult['primitive_ids'][0].item()
        triangleNormal = np.asarray(mesh.triangle_normals[closestTriangle])
        # step 3: compare the normal with the ray direction
        closestPointDirection = closestPoint - point
        dotProduct = (closestPointDirection @ triangleNormal)[0]
        if (dotProduct > 0):
            insideList.append(point)
        else:
            outsideList.append(point)

    return insideList, outsideList

# geturns all the points that are inside the (closed) mesh
def get_invisible_points(points : o3d.geometry.Geometry, mesh: o3d.geometry.Geometry) -> o3d.geometry.Geometry:
    """Returns all the points that are not visible because the are within a mesh

    Args:
        points (open3d.geometry): The points to chack
        mesh (open3d.geometry): The mesh to check against

    Returns:
        open3d.geometry: The filtered poincloud
    """
    
    insideList, outsideList = check_point_inside_mesh(points.points, mesh)
    visiblePoints = o3d.geometry.PointCloud()
    visiblePoints.points = o3d.utility.Vector3dVector(outsideList)
    invisiblePoints = o3d.geometry.PointCloud()
    invisiblePoints.points = o3d.utility.Vector3dVector(insideList)
    return invisiblePoints

def sample_mesh_with_density(mesh: o3d.geometry.TriangleMesh, density : float) -> o3d.geometry.PointCloud:
    print("not implemented yet")
    pass