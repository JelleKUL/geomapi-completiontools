"Help functions for generic combination algorithms"

import open3d as o3d
import matplotlib
from scipy.spatial import Delaunay
import numpy as np
import completiontools.params as params
from typing import List, Tuple

def hello_world():
    return "hello world!"

def get_geometry(path : str) -> o3d.geometry:
    """Gets a open3d Geometry from a path

    Args:
        path (str): The absulute path to the resource

    Returns:
        o3d.geometry: open3d.Pointcloud or open3d.Trianglemesh, depending on the extension
    """

    newGeometry = None
    if(path.endswith(tuple(params.MESH_EXTENSION))):
        newGeometry = o3d.io.read_triangle_mesh(path)
        #if not newGeometry.has_vertex_normals():
        newGeometry.compute_vertex_normals()
    elif(path.endswith(tuple(params.PCD_EXTENSION))):
        newGeometry = o3d.io.read_point_cloud(path)
        
    return newGeometry

def show_geometries(geometries : 'List[o3d.geometry]', color : bool = False):
    """Displays different types of geometry in a scene

    Args:
        geometries (List[open3d.geometry]): The list of geometries
        color (bool, optional): recolor the objects to have a unique color. Defaults to False.
    """

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

def get_lineset(geometry: o3d.geometry.TriangleMesh, color: Tuple[float, float, float] = (1,0,0)) -> o3d.geometry.LineSet:
    """Returns a lineset representation of a mesh

    Args:
        geometry (open3d.geometry.trianglemesh): the mesh to convert
        color (Tuple, optional): the color to paint the lineset. Defaults to (1,0,0).

    Returns:
        open3d.geometry.LineSet: the lineset from the mesh
    """

    ls = o3d.geometry.LineSet.create_from_triangle_mesh(geometry)
    ls.paint_uniform_color(color)

    return ls

def get_indices_in_hull(points : np.array, hull :np.array) -> List[int]:
    """Get the indices of all tyhe points that are inside the hull

    Args:
        points (numpy.array): should be a `NxK` coordinates of `N` points in `K` dimensions
        hull (np.array): is either a scipy.spatial.Delaunay object or the `MxK` array of the 
coordinates of `M` points in `K`dimensions for which Delaunay triangulation
will be computed

    Returns:
        List[int]: the indices of the points that are in the hull
    """

    if not isinstance(hull,Delaunay):
        hull = Delaunay(hull)
    ind = hull.find_simplex(points)>=0

    intList = []
    for i,x in enumerate(ind):
        if ind[i]:
            intList.append(i)

    return intList

def filter_pcd_by_distance(sourcePcd : o3d.geometry.PointCloud, testPcd: o3d.geometry.PointCloud, maxDistance : float) -> Tuple[o3d.geometry.PointCloud,o3d.geometry.PointCloud]:
    """Splits the sourcePcd in close and too far point compared to the testpcd

    Args:
        sourcePcd (open3d.geometry.PointCloud): The pcd to be split
        o3d (open3d.geometry.PointCloud): the pcd to test against

    Returns:
        Tuple(o3d.geometry.PointCloud,o3d.geometry.PointCloud): The points close enough and the points too far away
    """

    dists = sourcePcd.compute_point_cloud_distance(testPcd)
    dists = np.asarray(dists)
    ind = np.where(dists < maxDistance)[0]

    return (sourcePcd.select_by_index(ind), sourcePcd.select_by_index(ind, True))