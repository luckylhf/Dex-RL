"""Generate the V4 seven-cylinder locomotion model and walk deployment configuration."""
import ast
import re
import struct
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
asset_dir = ROOT / 'TienKung-Lab/legged_lab/assets/Tienkung_Dex'
for name in ('waist_pitch_link_lower.obj', 'waist_pitch_link_upper.obj'):
    path = asset_dir / 'mujoco/meshes_import' / name
    if not path.is_file():
        raise FileNotFoundError(f'Missing split torso mesh: {path}')
urdf = ET.parse(asset_dir / 'urdf/tiangong3.urdf').getroot()
fixed = {'head_pitch_joint', 'head_yaw_joint', 'elbow_yaw_l_joint', 'elbow_yaw_r_joint',
         'wrist_pitch_l_joint', 'wrist_pitch_r_joint', 'wrist_roll_l_joint', 'wrist_roll_r_joint'}
for joint in urdf.findall('joint'):
    if joint.get('name') in fixed:
        joint.set('type', 'fixed')
for mesh in urdf.findall('.//mesh'):
    mesh.set('filename', '../meshes/' + Path(mesh.get('filename')).name)
ET.indent(urdf)
ET.ElementTree(urdf).write(asset_dir / 'urdf/tiangong3_locomotion.urdf', encoding='utf-8', xml_declaration=True)
ros_urdf = ET.parse(asset_dir / 'urdf/tiangong3.urdf').getroot()
for joint in ros_urdf.findall('joint'):
    if joint.get('name') in {'head_pitch_joint', 'head_yaw_joint'}:
        joint.set('type', 'fixed')
for mesh in ros_urdf.findall('.//mesh'):
    mesh.set('filename', '../meshes/' + Path(mesh.get('filename')).name)
ET.indent(ros_urdf)
ET.ElementTree(ros_urdf).write(asset_dir / 'urdf/tiangong3_ros.urdf', encoding='utf-8', xml_declaration=True)
asset = ast.parse((ROOT / 'TienKung-Lab/legged_lab/assets/Tienkung_Dex/walk.py').read_text())
call = next(n.value for n in asset.body if isinstance(n, ast.Assign) and
            any(isinstance(a, ast.Name) and a.id == 'DEX_V3_CFG' for a in n.targets))
keywords = {k.arg: k.value for k in call.keywords}
default = ast.literal_eval(next(k.value for k in keywords['init_state'].keywords if k.arg == 'joint_pos'))
gains = {}
for actuator in keywords['actuators'].values:
    for field in ['stiffness', 'damping', 'effort_limit_sim']:
        values = ast.literal_eval(next(k.value for k in actuator.keywords if k.arg == field))
        for name in default:
            for pattern, value in values.items():
                if re.fullmatch(pattern, name):
                    gains.setdefault(name, {})[field] = value

path = ROOT / 'Deploy_Tienkung/policy/walk_amp/config/walk_amp.yaml'
cfg = yaml.safe_load(path.read_text())
# Deployment keeps the selected Duan policy's paired posture and gains.


robot_path = ROOT / 'Deploy_Tienkung/config/dex_config.yaml'
robot_cfg = yaml.safe_load(robot_path.read_text())
joint_names = ['hip_pitch_l_joint', 'hip_roll_l_joint', 'hip_yaw_l_joint', 'knee_pitch_l_joint',
               'ankle_pitch_l_joint', 'ankle_roll_l_joint', 'hip_pitch_r_joint', 'hip_roll_r_joint',
               'hip_yaw_r_joint', 'knee_pitch_r_joint', 'ankle_pitch_r_joint', 'ankle_roll_r_joint',
               'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint', 'shoulder_pitch_l_joint',
               'shoulder_roll_l_joint', 'shoulder_yaw_l_joint', 'elbow_pitch_l_joint', 'elbow_yaw_l_joint',
               'wrist_pitch_l_joint', 'wrist_roll_l_joint', 'shoulder_pitch_r_joint', 'shoulder_roll_r_joint',
               'shoulder_yaw_r_joint', 'elbow_pitch_r_joint', 'elbow_yaw_r_joint', 'wrist_pitch_r_joint', 'wrist_roll_r_joint']
joints = {j.get('name'): j for j in urdf.findall('joint')}
for key, name in zip(robot_cfg['robot_interface']['joint_limits'], joint_names):
    limit = joints[name].find('limit')
    robot_cfg['robot_interface']['joint_limits'][key] = {
        'min': float(limit.get('lower')), 'max': float(limit.get('upper'))}
robot_path.write_text(yaml.safe_dump(robot_cfg, sort_keys=False))

# MuJoCo imports the authoritative URDF, including the full inertia tensor.
import mujoco
import os
source = ET.fromstring(ET.tostring(ros_urdf))
for link in source.findall('link'):
    for kind in ['visual', 'collision']:
        for index, geom in enumerate(link.findall(kind)):
            geom.set('name', link.get('name')+'_'+kind+'_'+str(index))
mesh_files = {}
for name in {Path(mesh.get('filename')).name for mesh in source.findall('.//mesh')}:
    path = asset_dir / 'meshes' / name
    with path.open('rb') as stream:
        stream.seek(80)
        faces = struct.unpack('<I', stream.read(4))[0]
    mesh_files[name] = name
    if faces > 200000:  # MuJoCo's STL reader limit; OBJ keeps the same triangles.
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        triangles = np.fromfile(path, dtype=dtype, offset=84)['vertices']
        vertices, indices = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
        obj = asset_dir / 'mujoco/meshes_import' / (path.stem + '.obj')
        obj.parent.mkdir(exist_ok=True)
        with obj.open('w') as stream:
            np.savetxt(stream, vertices, fmt='v %.9g %.9g %.9g')
            np.savetxt(stream, indices.reshape(-1, 3)+1, fmt='f %d %d %d')
        mesh_files[name] = os.path.relpath(obj, asset_dir / 'meshes')
for mesh in source.findall('.//mesh'):
    mesh.set('filename', mesh_files[Path(mesh.get('filename')).name])
special = ET.SubElement(source, 'mujoco')
ET.SubElement(special, 'compiler', meshdir=str(asset_dir / 'meshes'),
              discardvisual='false', fusestatic='false', strippath='false')
plant = mujoco.MjModel.from_xml_string(ET.tostring(source, encoding='unicode'))
xml_path = ROOT / 'xSIM_MUJOCO-main/resources/dex_v3/urdf/dex_v3.xml'
xml_path.parent.mkdir(parents=True, exist_ok=True)
mujoco.mj_saveLastXML(str(xml_path), plant)
xml = ET.parse(xml_path)
model = xml.getroot()
# mj_saveLastXML rounds numbers; retain the imported body frames and inertia at full precision.
for body in model.findall('.//body'):
    index = plant.body(body.get('name')).id
    body.set('pos', ' '.join(format(v, '.17g') for v in plant.body_pos[index]))
    body.set('quat', ' '.join(format(v, '.17g') for v in plant.body_quat[index]))
    inertial = body.find('inertial')
    if inertial is not None:
        inertial.set('mass', format(plant.body_mass[index], '.17g'))
        inertial.set('pos', ' '.join(format(v, '.17g') for v in plant.body_ipos[index]))
        inertial.set('quat', ' '.join(format(v, '.17g') for v in plant.body_iquat[index]))
        inertial.set('diaginertia', ' '.join(format(v, '.17g') for v in plant.body_inertia[index]))
model.find('compiler').set('meshdir', '../meshes')
for mesh in model.findall('asset/mesh'):
    file = Path(mesh.get('file')).name
    mesh.set('file', mesh_files.get(file, '../mujoco/meshes_import/' + file))

# Preserve the shipped MuJoCo model's split torso meshes and display settings.
# The source URDF uses a single torso mesh, so this is a MuJoCo-specific override.
asset = model.find('asset')
torso_mesh = next(mesh for mesh in asset.findall('mesh') if mesh.get('name') == 'waist_pitch_link')
index = list(asset).index(torso_mesh)
asset.remove(torso_mesh)
for offset, (name, file, scale) in enumerate([
    ('waist_pitch_chest', 'waist_pitch_link_lower.obj', None),
    ('waist_pitch_neck', 'waist_pitch_link_upper.obj', '1.002 1.002 1.002'),
]):
    attrs = {'name': name, 'content_type': 'model/obj',
             'file': '../mujoco/meshes_import/' + file}
    if scale:
        attrs['scale'] = scale
    asset.insert(index + offset, ET.Element('mesh', attrs))

torso = next(body for body in model.findall('.//body') if body.get('name') == 'waist_pitch_link')
for kind in ('visual', 'collision'):
    original = next(geom for geom in torso.findall('geom') if geom.get('name') == f'waist_pitch_link_{kind}_0')
    index = list(torso).index(original)
    torso.remove(original)
    for offset, mesh in enumerate(('waist_pitch_chest', 'waist_pitch_neck')):
        geom = ET.Element('geom', original.attrib.copy())
        geom.set('name', f'waist_pitch_link_{kind}_{offset}')
        geom.set('mesh', mesh)
        torso.insert(index + offset, geom)

light_links = {'hip_roll_l_link', 'hip_yaw_l_link', 'hip_roll_r_link',
               'hip_yaw_r_link', 'camera_body_front_link'}
for body in model.findall('.//body'):
    for geom in body.findall('geom'):
        light = body.get('name') in light_links or geom.get('mesh') == 'waist_pitch_chest'
        geom.set('rgba', '0.55 0.55 0.55 1' if light else '0.32 0.32 0.32 1')

visual = ET.Element('visual')
ET.SubElement(visual, 'headlight', ambient='0.4 0.4 0.4', diffuse='0.45 0.45 0.45')
model.insert(list(model).index(asset), visual)
asset.insert(0, ET.Element('texture', type='skybox', builtin='gradient', rgb1='0.4 0.5 0.6',
                           rgb2='0 0 0', width='512', height='3072'))
asset.insert(1, ET.Element('texture', name='grid', type='2d', builtin='checker', width='512',
                           height='512', rgb1='0.1 0.2 0.3', rgb2='0.2 0.3 0.4'))
asset.insert(2, ET.Element('material', name='grid', texture='grid', texrepeat='10 10',
                           texuniform='true', reflectance='0.2'))
ET.SubElement(model, 'option', timestep='0.0025', integrator='implicitfast')
world = model.find('worldbody')
ET.SubElement(world, 'light', pos='0 0 4', dir='0 0 -1', directional='true',
              castshadow='true', diffuse='0.35 0.35 0.35', specular='0.2 0.2 0.2')
ET.SubElement(world, 'geom', name='ground', type='plane', size='10 10 0.1', material='grid', friction='1 0.005 0.0001')
pelvis = next(b for b in world.findall('body') if b.get('name') == 'pelvis')
pelvis.set('pos', '0 0 0.995')
ET.SubElement(pelvis, 'freejoint', name='pelvis_to_world_joint')
ET.SubElement(pelvis, 'site', name='imu', size='0.01')
actuator = ET.SubElement(model, 'actuator')
sensors = ET.SubElement(model, 'sensor')
active = {j.get('name'): j for j in ros_urdf.findall('joint') if j.get('type') != 'fixed'}
for name, joint in active.items():
    effort = gains[name]['effort_limit_sim'] if name in gains else float(joint.find('limit').get('effort'))
    mj_joint = next(j for j in model.findall('.//joint') if j.get('name') == name)
    limit = joint.find('limit')
    mj_joint.set('range', limit.get('lower')+' '+limit.get('upper'))
    mj_joint.set('actuatorfrcrange', f'-{effort} {effort}')
    ET.SubElement(actuator, 'motor', name='motor_'+name, joint=name, ctrlrange=f'-{effort} {effort}')
    ET.SubElement(sensors, 'jointpos', name=name+'_pos', joint=name)
    ET.SubElement(sensors, 'jointvel', name=name+'_vel', joint=name)
    ET.SubElement(sensors, 'jointactuatorfrc', name=name+'_torque', joint=name)
ET.SubElement(sensors, 'framequat', name='orientation', objtype='site', objname='imu')
ET.SubElement(sensors, 'gyro', name='angular-velocity', site='imu')
ET.SubElement(sensors, 'accelerometer', name='linear-acceleration', site='imu')
ET.SubElement(sensors, 'framepos', name='position', objtype='site', objname='imu')
ET.SubElement(sensors, 'framelinvel', name='linear-velocity', objtype='site', objname='imu')
ET.SubElement(sensors, 'magnetometer', name='magnetometer', site='imu')
ET.indent(model)
xml.write(xml_path, encoding='unicode')
print('Synced V4: training 23 joints, ROS 29 joints, seven foot cylinders, 400/100 Hz, height 0.995 m.')
