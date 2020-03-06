import glob
import subprocess

for file in glob.glob("*.stl"):
    print(file)
    file_name_no_ext = file.split(".")[0]
    subprocess.Popen(["ctmconv", file, file_name_no_ext + ".off"])

