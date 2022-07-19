"""Tools to complete and combine meshes and pointclouds
"""

import completiontools.params as params
import completiontools.utils as ut
import geomapi
import open3d as o3d
import numpy as np

# The main function to combine 2 aligned geometries
def combine_geometry(ogGeometry, newGeometry, distanceTreshold):

    # Step 1: Create a convex hull of the newGeometry
    newGeoHull = get_convex_hull(newGeometry)
    # Step 2: Filter out the irrelevant points in the ogGeometry
    relevantOg, irrelevantOg = get_points_in_hull(ogGeometry, newGeoHull)
    # Step 3: Isolate the not covered points of the ogGeometry compared to the newGeometry
    newGeometryPoints = newGeometry.sample_points_poisson_disk(number_of_points=100000)
    coveredPoints, unCoveredPoints = ut.filter_pcd_by_distance(relevantOg, newGeometryPoints, distanceTreshold)
    # Step 4: Perform the visibility check of the not covered points
    invisibleUncoveredPoints = get_invisible_points(unCoveredPoints, newGeometry)
    # Step 5: Filter the newGeometryPoints to only keep the changed geometry
    existingNewGeo, newNewGeo = ut.filter_pcd_by_distance(newGeometryPoints, relevantOg, distanceTreshold)
    # Step 6: Combine the irrelevant, unchanged and changed geometry
    newCombinedGeometry = irrelevantOg + coveredPoints + invisibleUncoveredPoints + newNewGeo
    return newCombinedGeometry

# Converts a geometry to a covex hull
def get_convex_hull(geometry : o3d.geometry):

    hull, _ = geometry.compute_convex_hull()
    return hull

# Returns a filtered mpcd with only points which are inside the convex hull
def get_points_in_hull(geometry, hull):
    hullVerts = np.asarray(hull.vertices)
    points = np.asarray(geometry.points)
    idxs = ut.get_indices_in_hull(points, hullVerts)
    pcdInHull = geometry.select_by_index(idxs)
    pcdOutHull = geometry.select_by_index(idxs, True)
    return pcdInHull, pcdOutHull

# checks if the points is inside the (closed) mesh
def check_point_inside_mesh(points, mesh):

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
def get_invisible_points(points, mesh):
    
    insideList, outsideList = check_point_inside_mesh(points, mesh)
    visiblePoints = o3d.geometry.PointCloud()
    visiblePoints.points = o3d.utility.Vector3dVector(outsideList)
    invisiblePoints = o3d.geometry.PointCloud()
    invisiblePoints.points = o3d.utility.Vector3dVector(insideList)
    return invisiblePoints

def pcd_to_mesh(geometry):
    pass