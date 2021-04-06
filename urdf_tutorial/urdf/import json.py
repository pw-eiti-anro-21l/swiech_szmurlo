import json
import yaml
import math
from math import cos, sin, atan, atan2, sqrt

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


def create_yaml_dict(r, p, y):
    #{'params' : [{'roll': r}, {'pitch': p}, {'yaw': y}]}
    dict_file = [
                {'link' : [
                {
                'interial': [
                {'mass': [{'properties': [{'format': [{'type': 'string', 'xml':{'attribute': 'true'}}]}]}]},
                'interia',
                'origin'
                ]
                }, 
                {'visual' : ['origin', {'geometry': ['cylinder']}, {'material': ['color']}]},
                {'collision' : ['origin', {'geometry': ['cylinder']}, 'contact_coefficients']},
                ]
                }
    ]
    return dict_file



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
    # print(f'Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}')

    yaml_dict = create_yaml_dict(roll, pitch, yaw)

    with open ("/Users/maciekswiech/Desktop/ANRO/swiech_szmurlo/urdf_tutorial/urdf/rpy.yaml", "w") as yaml_file:

        rpy_params = yaml.dump(yaml_dict, yaml_file)


    dict_file = [
                {'link' : [
                {
                'interial': [
                {
                'type': 'object',
                'xml': {'name': 'mass'},
                'properties': {'coordinate': {'type': 'string',
                'properties': [{'format': [{'type': 'string', 'xml':{'attribute': 'true'}}]}]}},
                'interia': 'lala',
                'origin': 'lala'
                }
                ]
                }, 
                {'visual' : ['origin', {'geometry': ['cylinder']}, {'material': ['color']}]},
                {'collision' : ['origin', {'geometry': ['cylinder']}, 'contact_coefficients']},
                ]
                }
    ]






    dict_file = [
                {'link':
                {
                'interial':{
                'mass': {'_value': '1'},
                'interia': {'_ixx': 100},
                'origin': ''
                },
                'visual': ['origin', {'geometry': ['cylinder']}, {'material': ['color']}],
                'collision' : ['origin', {'geometry': ['cylinder']}, 'contact_coefficients']
                }
                }
    ]

        dict_file = [
                {'link' : [
                {
                'interial': [
                {
                'type': 'object',
                'xml': {'name': 'mass'},
                'properties': {'coordinate': {'type': 'string',
                'properties': [{'format': [{'type': 'string', 'xml':{'attribute': 'true'}}]}]}},
                'interia': 'lala',
                'origin': 'lala'
                }
                ]
                }, 
                {'visual' : ['origin', {'geometry': ['cylinder']}, {'material': ['color']}]},
                {'collision' : ['origin', {'geometry': ['cylinder']}, 'contact_coefficients']},
                ]
                }
    ]