import yaml
import argparse
import os
import numpy as np

def repeat_params(args: dict):
    print("Running script with these args")
    for key, value in args.items():
        print(f"{key}: {value}")
        
def calc_CG(positions: np.ndarray, masses: np.ndarray) -> np.ndarray:
    return np.sum(positions * masses, axis=0) / np.sum(masses)
        
def calc_Jcg(positions: np.ndarray, masses: np.ndarray, inertias: list, cg: np.ndarray) -> np.ndarray:
    Js = np.zeros((3,3))
    
    for position, mass, inertia in zip(positions, masses, inertias):
        r_cg_p = cg - position
        mass = mass.item()
        Js = Js + inertia + mass*(np.dot(r_cg_p, r_cg_p)*np.eye(3) - np.outer(r_cg_p, r_cg_p))
    
    return Js
    
def main(param_file: str, output_file: str) -> None:
    if param_file is None:
        print("Error. Please specify a param .yaml file. Run with \'-h\' for more information.")
        return None
    try:
        with open(param_file, "r") as stream:
                file = yaml.safe_load(stream)
    except (FileNotFoundError, yaml.YAMLError) as e:
        print("Error opening the file")
        print(e)
        return None

    positions = np.array(file["component_positions"]).astype(float)
    masses = np.array(file["component_masses"]).astype(float).reshape(-1,1)
    inertias = file["component_inertias"]
    
    for i, inertia in enumerate(inertias):
        inertias[i] = np.array(inertia)
    
    cg: np.ndarray = calc_CG(positions, masses)
    
    print("CG:", cg)
    
    Jcg: np.ndarray = calc_Jcg(positions, masses, inertias, cg)
    
    print("J around CG:", Jcg)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script to compute the inertia tensor given point masses from yaml file")
    parser.add_argument('-f', '--param-file', type=str, default=None, help="Location of the config file. Required")
    parser.add_argument('-o', '--output-file', type=str, default=os.path.join(os.getcwd(), "inertia_tensor.yaml"), help="Location of the output file. Default is current working directory")
    
    args = vars(parser.parse_args())
    
    repeat_params(args)
    
    main(**args)