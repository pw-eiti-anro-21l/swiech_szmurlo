import json
import transformations
import pprint

def get_xyz_rpy():
    with open("/Users/maciekswiech/Desktop/ANRO/swiech_szmurlo/urdf_tutorial/urdf/dh_params.json", "r") as file:
            dh_params = json.load(file)

    # with open('urdf.yaml', 'w') as file:
    rpy_xyz={}
    inter = 1
    for i in dh_params:
        dh_row = json.loads(json.dumps(i))
        a_translation = transformations.translation_matrix((dh_row['a'],0,0))
        d_translation = transformations.translation_matrix((0,0,dh_row["d"]))
        alpha_rotation = transformations.rotation_matrix(dh_row["alpha"],(1, 0, 0))
        theta_rotation = transformations.rotation_matrix(dh_row["theta"],(0, 0, 1))
        trans_matrix = a_translation @ alpha_rotation @ d_translation @ theta_rotation
        rpy = transformations.euler_from_matrix(trans_matrix)
        xyz = transformations.translation_from_matrix(trans_matrix)
        print (f'{xyz} \n')
        rpy_xyz['i'+ str(inter)] = (rpy, tuple(xyz))
        inter+=1
        pprint.pprint(rpy_xyz)
        
get_xyz_rpy()