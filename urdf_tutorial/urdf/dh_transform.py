import json
import yaml
import math
from math import cos, sin, atan, atan2, sqrt
# import xml.etree.ElementTree as ET
from xml.dom import minidom
import xml
from lxml import etree as ET
import json
import transformations
import pprint

def get_parameters(part):
    with open("/Users/maciekswiech/Desktop/ANRO/swiech_szmurlo/urdf_tutorial/urdf/params_server.json", "r") as read_file:
        data = json.load(read_file)
    part_parameters = data[part]
    return part_parameters


def get_xyz_rpy():
    with open("/Users/maciekswiech/Desktop/ANRO/swiech_szmurlo/urdf_tutorial/urdf/dh_params.json", "r") as file:
        dh_params = json.load(file)
    rpy_xyz={}
    inter = 1
    iterator = 1

    xyz_array = []
    rpy_array = []
    params_array = []

    for i in dh_params:
        dh_row = json.loads(json.dumps(i))
        a_translation = transformations.translation_matrix((dh_row["a"],0,0))
        d_translation = transformations.translation_matrix((0,0,dh_row["d"]))
        alpha_rotation = transformations.rotation_matrix(dh_row["alpha"],(1, 0, 0))
        theta_rotation = transformations.rotation_matrix(dh_row["theta"],(0, 0, 1))
        trans_matrix = a_translation @ alpha_rotation @ d_translation @ theta_rotation
        rpy = transformations.euler_from_matrix(trans_matrix)
        xyz = transformations.translation_from_matrix(trans_matrix)

        params_array.append({'xyz': xyz, 'rpy': rpy, 'd': dh_row['d']})
        xyz_array.append(xyz)
        rpy_array.append(rpy)

    return params_array



def create_xml_link(roll, pitch, yaw, x, y, z, length, name):

    parameters = get_parameters(name)

    z_translation = str(float(-0.5*length))
    length = str(length)

    link_name = parameters['link_name']
    mass_value = parameters['mass']
    ixx = parameters['inertia']['ixx']
    ixy = parameters['inertia']['ixy']
    ixz = parameters['inertia']['ixz']
    iyy = parameters['inertia']['iyy']
    iyz = parameters['inertia']['iyz']
    izz = parameters['inertia']['izz']
    xyz = f'{0} {0} {z_translation}'
    rpy = f'{0} {0} {0}'
    radius = parameters['radius']
    material_name = parameters['material_name']
    color_rgba = parameters['color']
    mu = parameters['mu']
    kp = parameters['kp']
    kd = parameters['kd']


    link = ET.Element('link', name = link_name)
    inertial = ET.SubElement(link, 'inertial')
    mass_interial = ET.SubElement(inertial, "mass", value=mass_value)
    interia_interial = ET.SubElement(inertial, "interia", ixx = ixx, ixy = ixy, ixz = ixz, iyy = iyy, iyz = iyz, izz = izz)
    origin_interial = ET.SubElement(inertial, "origin")

    visual = ET.SubElement(link, "visual")
    origin_visual = ET.SubElement(visual, "origin", xyz = xyz, rpy = rpy)
    geometry_visual = ET.SubElement(visual, "geometry")
    cylinder_visual = ET.SubElement(geometry_visual, "cylinder", radius = radius, length = length)
    material_visual = ET.SubElement(visual, "material", name = material_name)
    color_visual = ET.SubElement(material_visual, "color", rgba = color_rgba)

    collision = ET.SubElement(link, "collision")
    origin_collision = ET.SubElement(collision, "origin", xyz = xyz, rpy = rpy)
    geometry_collision = ET.SubElement(collision, "geometry")
    cylinder_collision = ET.SubElement(geometry_collision, "cylinder", radius = radius, length = length)
    contact_coefficients_collision = ET.SubElement(collision, "contact_coefficients", mu = mu, kp = kp, kd = kd)
   
    return link

def create_xml_joint(roll, pitch, yaw, x, y, z, name):

    parameters = get_parameters(name)

    joint_name = parameters['joint_name']
    joint_type = parameters['joint_type']
    parent_link = parameters['parent']
    child_link = parameters['child']
    origin_xyz = f'{x} {y} {z}'
    origin_rpy = f'{roll} {pitch} {yaw}'
    axis_xyz = parameters['axis_xyz']
    upper_limit = parameters['upper_limit']
    lower_limit = parameters['lower_limit']
    effort_limit = parameters['effort']
    velocity_limit = parameters['velocity']

    joint = ET.Element('joint', name = joint_name, type = joint_type)
    parent = ET.SubElement(joint, 'parent', link = parent_link)
    child = ET.SubElement(joint, 'child', link = child_link)
    origin = ET.SubElement(joint, 'origin', xyz = origin_xyz, rpy = origin_rpy)
    axis = ET.SubElement(joint, 'axis', xyz = axis_xyz)
    limit = ET.SubElement(joint, 'limit', upper = upper_limit, lower = lower_limit, effort = effort_limit, velocity = velocity_limit)
    
    return joint


if __name__ == "__main__":

    params_array = get_xyz_rpy()
    array_of_urdfs = []

    joint_iterator = 1

    for params in params_array:
        rpy = params['rpy']
        xyz = params['xyz']
        d_translation = params['d']
        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]
        x = xyz[0]
        y = xyz[1]

        if joint_iterator == 1:
            z = xyz[2]+1 # podwyzszenie bazy
        else:
            z = xyz[2]
        d = d_translation

        joint_name_in_server = f'joint_{joint_iterator}'
        link_name_in_server = f'link_{joint_iterator}'

        joint_urdf = create_xml_joint(roll, pitch, yaw, x, y ,z, joint_name_in_server)
        link_urdf = create_xml_link(roll, pitch, yaw, x, y, z, d_translation, link_name_in_server)
        array_of_urdfs.append(joint_urdf)
        array_of_urdfs.append(link_urdf)

        # print (f'Roll: {roll}, Pitch: {pitch}, Yaw: {yaw} \n')
    base_urdf = create_xml_link(roll, pitch, yaw, x, y, z, 1, "base")
    tool_urdf = create_xml_link(roll, pitch, yaw, x, y, z, 1, "tool")
    body_tool_joint_urdf = create_xml_joint(roll, pitch, yaw, x, y, z, "joint_tool")

    tree = ET.Element("robot")
    tree.extend([base_urdf])
    tree.extend(array_of_urdfs)
    tree.extend([body_tool_joint_urdf])
    tree.extend([tool_urdf])
    ET.ElementTree(tree).write('/Users/maciekswiech/Desktop/ANRO/swiech_szmurlo/urdf_tutorial/urdf/my_robot.urdf.xml', pretty_print=True)