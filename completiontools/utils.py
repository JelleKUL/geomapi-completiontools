import open3d as o3d
import matplotlib
from scipy.spatial import Delaunay
import numpy as np
import completiontools.params as params

def hello_world():
    return "hello world!"

def get_geometry(path):
    """Returns the open3d geometry object from a path"""
    
    newGeometry = None
    if(path.endswith(tuple(params.MESH_EXTENSION))):
        newGeometry = o3d.io.read_triangle_mesh(path)
        #if not newGeometry.has_vertex_normals():
        newGeometry.compute_vertex_normals()
    elif(path.endswith(tuple(params.PCD_EXTENSION))):
        newGeometry = o3d.io.read_point_cloud(path)
    return newGeometry

def show_geometries(geometries, color = False):
    "displays the array of meshes in a 3D view"

    viewer = o3d.visualization.Visualizer()
    viewer.create_window()
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
    viewer.add_geometry(frame)
    for i, geometry in enumerate(geometries):
        if color:
            geometry.paint_uniform_color(matplotlib.colors.hsv_to_rgb([float(i)/len(geometries),0.8,0.8]))
        viewer.add_geometry(geometry)
    opt = viewer.get_render_option()
    opt.background_color = np.asarray([1,1,1])
    opt.light_on = True
    viewer.run()

def get_points_in_hull(points, hull):
    """
    Test if points in `points` are in `hull`

    `points` should be a `NxK` coordinates of `N` points in `K` dimensions
    `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the 
    coordinates of `M` points in `K`dimensions for which Delaunay triangulation
    will be computed
    Returns a bool array with in or not
    """

    if not isinstance(hull,Delaunay):
        hull = Delaunay(hull)

    return hull.find_simplex(points)>=0

def get_indices_in_hull(points, hull):
    """
    Test if points in `points` are in `hull`

    `points` should be a `NxK` coordinates of `N` points in `K` dimensions
    `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the 
    coordinates of `M` points in `K`dimensions for which Delaunay triangulation
    will be computed
    Returns the indices of the points that are in the hull, usefull for open3d pointclouds
    """

    if not isinstance(hull,Delaunay):
        hull = Delaunay(hull)
    ind = hull.find_simplex(points)>=0

    intList = []
    for i,x in enumerate(ind):
        if ind[i]:
            intList.append(i)

    return intList

def filter_pcd_by_distance(sourcePcd, testPcd, maxDistance : float):
    """ 
    Returns a filtered point cloud based on the distance to another, 
        1) all the points close enough, 
        2) all the far away points
    """

    dists = sourcePcd.compute_point_cloud_distance(testPcd)
    dists = np.asarray(dists)
    ind = np.where(dists < maxDistance)[0]

    return sourcePcd.select_by_index(ind), sourcePcd.select_by_index(ind, True)