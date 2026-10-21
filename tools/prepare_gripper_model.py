#!/usr/bin/env python3
"""Offline STEP tessellation; OCP is a build-time dependency, never a robot dependency."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS


def xyz(value):
    return [value.X(), value.Y(), value.Z()]


def children(shape, kind):
    exp = TopExp_Explorer(shape, kind)
    while exp.More():
        yield exp.Current()
        exp.Next()


def bounds(shape):
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    return list(box.Get())


def triangles(shape):
    result = []
    for item in children(shape, TopAbs_FACE):
        face = TopoDS.Face_s(item)
        loc = TopLoc_Location()
        mesh = BRep_Tool.Triangulation_s(face, loc)
        if mesh is None:
            continue
        points = np.array([xyz(mesh.Node(i).Transformed(loc.Transformation()))
                           for i in range(1, mesh.NbNodes()+1)])
        for i in range(1, mesh.NbTriangles()+1):
            indices = np.array(mesh.Triangle(i).Get())-1
            if face.Orientation() == TopAbs_REVERSED:
                indices = indices[::-1]
            result.append(points[indices])
    return np.array(result, dtype=np.float32)


def describe(shape):
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    surfaces = []
    for item in children(shape, TopAbs_FACE):
        face = TopoDS.Face_s(item)
        surface = BRepAdaptor_Surface(face)
        area = GProp_GProps()
        BRepGProp.SurfaceProperties_s(face, area)
        if surface.GetType() == GeomAbs_Plane and area.Mass() > 80:
            surfaces.append(dict(type='plane', area=area.Mass(), center=xyz(area.CentreOfMass()),
                                 normal=xyz(surface.Plane().Axis().Direction())))
        elif surface.GetType() == GeomAbs_Cylinder and surface.Cylinder().Radius() > 2.5:
            cylinder = surface.Cylinder()
            surfaces.append(dict(type='cylinder', radius=cylinder.Radius(), area=area.Mass(),
                                 origin=xyz(cylinder.Location()), axis=xyz(cylinder.Axis().Direction())))
    return dict(bounds_mm=bounds(shape), volume_mm3=props.Mass(),
                center_mm=xyz(props.CentreOfMass()), surfaces=surfaces)


def convert(source, name, directory):
    reader = STEPControl_Reader()
    if reader.ReadFile(str(source)) != IFSelect_RetDone:
        raise ValueError(f'Cannot read STEP: {source}')
    reader.TransferRoots()
    shape = reader.OneShape()
    exact_bounds = bounds(shape)
    BRepMesh_IncrementalMesh(shape, .18, False, .25, True)
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    path = directory/(name+'.stl')
    if not writer.Write(shape, str(path)):
        raise ValueError(f'Cannot write STL: {path}')
    solids = list(children(shape, TopAbs_SOLID))
    np.savez_compressed(directory/(name+'-parts.npz'),
                        **{f'solid_{i}': triangles(s) for i, s in enumerate(solids)})
    report = dict(source_name=source.name, source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  units='mm', stl_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  bounds_mm=exact_bounds, solids=[describe(s) for s in solids],
                  tessellation=dict(linear_deflection_mm=.18, angular_deflection_rad=.25))
    (directory/(name+'-cad.json')).write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(f'{name}: {len(solids)} solids; {path.stat().st_size} bytes; {report["bounds_mm"]}', flush=True)


def simplify_mesh(source, target):
    import vtk
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(source))
    decimate = vtk.vtkDecimatePro()
    decimate.SetInputConnection(reader.GetOutputPort())
    decimate.SetTargetReduction(.88)
    decimate.PreserveTopologyOn()
    decimate.BoundaryVertexDeletionOff()
    decimate.SplittingOff()
    decimate.SetErrorIsAbsolute(True)
    decimate.SetAbsoluteError(.08)
    writer = vtk.vtkSTLWriter()
    writer.SetFileName(str(target))
    writer.SetFileTypeToBinary()
    writer.SetInputConnection(decimate.GetOutputPort())
    if not writer.Write():
        raise ValueError('Cannot write display mesh')


def write_triangles(path, points):
    import struct
    dtype = np.dtype([('normal', '<f4', (3,)), ('points', '<f4', (3, 3)), ('attribute', '<u2')])
    result = np.zeros(len(points), dtype=dtype)
    result['points'] = points
    normals = np.cross(points[:, 1]-points[:, 0], points[:, 2]-points[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    result['normal'] = normals/np.maximum(lengths[:, None], 1e-15)
    path.write_bytes(b'CAD solid groups'.ljust(80, b'\0')+struct.pack('<I', len(result))+result.tobytes())


def prepare_display(directory, destination):
    import shutil
    destination.mkdir(parents=True, exist_ok=True)
    manifest = dict(units='mm', geometry_id='taccap-step-20261020-v1', files={},
                    note='Measured normalized opening drives two rigid CAD jaw groups; internal linkage deformation is approximate. TCP stays fixed.')
    for name in ('adapter', 'gripper'):
        report = json.loads((directory/(name+'-cad.json')).read_text())
        source, target = directory/(name+'.stl'), destination/(name+'.stl')
        if name == 'adapter':
            shutil.copyfile(source, target)
        else:
            simplify_mesh(source, target)
        raw = target.read_bytes()
        manifest['files'][target.name] = dict(sha256=hashlib.sha256(raw).hexdigest(),
            source_name=report['source_name'], source_sha256=report['source_sha256'],
            bytes=len(raw), tessellation=report['tessellation'])
    parts = np.load(directory/'gripper-parts.npz')
    groups = {'gripper_body': list(range(27)),
              'jaw_positive': list(range(27, 36)), 'jaw_negative': list(range(36, 45))}
    report = json.loads((directory/'gripper-cad.json').read_text())
    # Use the observed gear shaft centres, ±19.95 / Z=-10 mm. The provided
    # STEP has only one open pose; approximate each articulated finger as one
    # rigid group. Define visual closed contact by a 1 mm gap, not motor angle.
    points = np.concatenate([parts[f'solid_{i}'] for i in groups['jaw_positive']]).reshape(-1, 3)
    shifted = points-np.array([19.95, 0., -10.])
    low, high = 0., np.deg2rad(40.)
    for _ in range(50):
        angle = (low+high)/2
        contact = (19.95+np.cos(angle)*shifted[:, 0]+np.sin(angle)*shifted[:, 2]).min()
        if contact > .5:
            low = angle
        else:
            high = angle
    close_angle = float(np.rad2deg((low+high)/2))
    manifest['articulation'] = {'model': 'two_rigid_jaws', 'approximate': True,
        'open_pose_normalized': 1., 'closed_gap_mm': 1., 'jaws': {
            'jaw_positive': {'pivot_mm': [19.95, 0., -10.], 'close_angle_deg': close_angle},
            'jaw_negative': {'pivot_mm': [-19.95, 0., -10.], 'close_angle_deg': -close_angle}}}
    for name, indices in groups.items():
        source, target = directory/(name+'.stl'), destination/(name+'.stl')
        write_triangles(source, np.concatenate([parts[f'solid_{i}'] for i in indices]))
        simplify_mesh(source, target)
        raw = target.read_bytes()
        manifest['files'][target.name] = dict(sha256=hashlib.sha256(raw).hexdigest(),
            source_name=report['source_name'], source_sha256=report['source_sha256'],
            source_solids=indices, bytes=len(raw), tessellation=report['tessellation'])
    (destination/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path('/home/lim/models'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--display-output', type=Path)
    parser.add_argument('--reuse', action='store_true', help='Reuse existing tessellation in --output')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if not args.reuse:
        for filename, name in [('转接.STEP', 'adapter'), ('从夹爪-外发版本.STEP', 'gripper')]:
            convert(args.source/filename, name, args.output)
    if args.display_output:
        prepare_display(args.output, args.display_output)


if __name__ == '__main__':
    main()
