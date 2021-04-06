import json
import yaml
import math
from math import cos, sin, atan, atan2, sqrt
# import xml.etree.ElementTree as ET
from lxml import etree as ET

def create_homogen(alpha, a, d, theta):
    homogen_matrix = [
    [cos(theta), -1*sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
    [sin(theta), cos(theta)*cos(alpha), -1*cos(theta)*sin(alpha), a*sin(theta)],
    [0, sin(alpha), cos(alpha), a],
    [0, 0, 0, 1]]
    return homogen_matrix


def multiply_matrix(matrix_1, matrix_2):

    column_index = 0
    row_index = 0
    current_element = 0
    num_of_columns = len(matrix_2[0])
    num_of_rows = len(matrix_1)
    
    new_matrix = [[0]*num_of_columns for i in range (num_of_rows)]

    for r in range (0, num_of_rows):
        for c in range (0, num_of_columns):
            current_sum = 0
            for i in range (0, len(matrix_1[r])):
                current_sum += matrix_1[r][i]*matrix_2[i][c]
            new_matrix [r][c] = current_sum
    return new_matrix


def create_rotation_matrix(matrix):
    rotation_matrix = [[0]*3 for i in range (3)]
    for r in range (0, 3):
        for c in range(0, 3):
            rotation_matrix[r][c] = matrix[r][c]
    return rotation_matrix


def roll_pitch_yaw_params(rotation_matrix):
    phi_2 = atan2(rotation_matrix[2][1], rotation_matrix[2][2])
    psi_2 = atan2(rotation_matrix[1][0], rotation_matrix[0][0])
    theta_2 = atan2(-1*rotation_matrix[2][0], (sqrt((rotation_matrix[2][1])**2 + (rotation_matrix[2][2])**2)))

    roll = phi_2
    pitch = theta_2
    yaw = psi_2

    return roll, pitch, yaw


def create_xml_link(r, p, y):
    link_name = "axis"
    mass_value = '1'
    ixx = '100'
    ixy = '0'
    ixz = '0'
    iyy = '100'
    iyz = '0'
    izz = '0'
    xyz = "0 0 0"
    rpy = "1.57 0 0"
    radius = "0.01"
    length = "0.5"
    material_name = "grey"
    color_rgba = "0.2 0.2 0.2 1"
    mu = "0"
    kp="1000.0"
    kd="1.0"


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

    tree = ET.ElementTree(link)
    tree.write('output.xml', pretty_print=True)
    return

def create_xml_joint():
    joint_name = "tilt"
    joint_type = "revolute"
    parent_link = "axis"
    child_link = "body"
    origin_xyz = "0 0 0"
    origin_rpy = "1.57 0 0"
    axis_xyz = "0 1 0"
    upper_limit = "0"
    lower_limit = "-0.5"
    effort_limit = "10"
    velocity_limit = "10"

    joint = ET.Element('joint', name = joint_name, type = joint_type)
    parent = ET.SubElement(joint, 'parent', link = parent_link)
    child = ET.SubElement(joint, 'child', link = child_link)
    origin = ET.SubElement(joint, 'origin', xyz = origin_xyz, rpy = origin_rpy)
    axis = ET.SubElement(joint, 'axis', xyz = axis_xyz)
    limit = ET.SubElement(joint, 'limit', upper = upper_limit, lower = lower_limit, effort = effort_limit, velocity = velocity_limit)
    tree = ET.ElementTree(joint)
    tree.write('output_joint.xml', pretty_print=True)
    return


if __name__ == "__main__":
    table_of_homogen_matrixes = []
    with open("/Users/maciekswiech/Desktop/ANRO/swiech_szmurlo/urdf_tutorial/urdf/dh_params.json", "r") as file:
        params_table = json.load(file)

    for single_table in params_table:
        alpha = single_table['alpha']
        a = single_table['a']
        d = single_table['d']
        theta = single_table['theta']
        homogen_matrix = create_homogen(alpha, a, d, theta)
        table_of_homogen_matrixes.append(homogen_matrix)


    columns = 4 
    rows = columns
    temporary_matrix = table_of_homogen_matrixes[len(table_of_homogen_matrixes)-1]

    array_of_rpy_params = []
    for matrix in table_of_homogen_matrixes:
        rpy_params = roll_pitch_yaw_params(matrix)
        array_of_rpy_params.append(rpy_params)
    
    for element in array_of_rpy_params:
        print(element)



    for i in range (3, 1, -1):
        temporary_matrix = multiply_matrix(temporary_matrix, table_of_homogen_matrixes[i-2]) 
    
    homogen_0_n_matrix = temporary_matrix
    rotation_matrix = create_rotation_matrix(homogen_0_n_matrix)

    (roll, pitch, yaw) = roll_pitch_yaw_params(rotation_matrix)
    create_xml_link(roll, pitch, yaw)
    create_xml_joint()

    # print(f'Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}')

    # yaml_dict = create_yaml_dict(roll, pitch, yaw)

    # with open ("/Users/maciekswiech/Desktop/ANRO/swiech_szmurlo/urdf_tutorial/urdf/rpy.yaml", "w") as yaml_file:

    #     rpy_params = yaml.dump(yaml_dict, yaml_file)