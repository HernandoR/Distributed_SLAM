import os

paths = ["src", "include"]


def Get_Reletive_path_of_Files(path):
    root_path = os.getcwd()
    for root, dirs, files in os.walk(path):
        for file in files:
            abs_path = os.path.join(root, file)
            ret_path = abs_path.replace(root_path, "", 1)
            print(ret_path)


for path in paths:
    Get_Reletive_path_of_Files(path)
