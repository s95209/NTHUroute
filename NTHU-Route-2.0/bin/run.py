#!/users/student/mr109/ykfang20/anaconda3/bin/python
import subprocess
import argparse
import os
from datetime import datetime 

testcase_path = "../../testcases"

parser = argparse.ArgumentParser()
parser.add_argument("testcase", type=str)
args = parser.parse_args()

testcase = args.testcase
year, caseNum = testcase.split('_')
param_path = f"./param_{year}_{caseNum}.txt" 

if caseNum[-1] == 'm':
    caseNum = caseNum[:-1] + '_metal5'

dir_path = f"{testcase_path}/ispd{year}_test{caseNum}"
if not os.path.exists(dir_path):
    os.makedirs(dir_path)

subprocess.run(["./route", "--input", "test",
"--def", f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.input.def",
"--lef", f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.input.lef", 
"--guide", f"{dir_path}/ispd{year}_test{caseNum}.input.guide",
"--param", f"{param_path}"])

# r --input test --def ../testcase/ispd19_test9/ispd19_test9.input.def --lef ../testcase/ispd19_test9/ispd19_test9.input.lef --guide ../testcase/ispd19_test9/ispd19_test9.input.guide

# r --input test --def ../testcase/ispd18_test5/ispd18_test5.input.def --lef ../testcase/ispd18_test5/ispd18_test5.input.lef --guide ../testcase/ispd18_test5/ispd18_test5.input.guide
