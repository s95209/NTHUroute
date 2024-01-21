#!/users/student/mr109/ykfang20/anaconda3/bin/python

import subprocess
import argparse
import os

parser = argparse.ArgumentParser()

parser.add_argument("testcase", type=str)

args = parser.parse_args()

testcase = args.testcase

year, caseNum = testcase.split('_')

print(f"Running test case ispd{year}_test{caseNum}...")

if caseNum[-1] == 'm':
    caseNum = caseNum[:-1] + '_metal5'

testcase_path = "/users/student/mr109/ykfang20/YKFang/testcases"#"../../testcases"

d = f"{testcase_path}/ispd{year}_test{caseNum}/"
subdirs = [os.path.join(d, o) for o in os.listdir(d) 
                    if os.path.isdir(os.path.join(d,o))]

for subdir in subdirs:
	report_path = os.path.join(subdir, "report.txt")
	if not os.path.exists(report_path):
		os.chdir(f"/users/student/mr109//ykfang20/COLA_normal/ispd{year}eval")
		if not os.path.exists(f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.input.lef"): continue
		if not os.path.exists(f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.input.def"): continue
		if not os.path.exists(f"{subdir}/ispd{year}_test{caseNum}.input.guide"): continue
		if not os.path.exists(f"{subdir}/ispd{year}_test{caseNum}.output.def"): continue
		if year == '18':
			with open(report_path, "w") as fout:
				subprocess.run([f"./ispd{year}eval.sh",
				"-lef", f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.input.lef", 
				"-guide", f"{subdir}/ispd{year}_test{caseNum}.input.guide",
				"-def", f"{subdir}/ispd{year}_test{caseNum}.output.def"], stdout=fout)
		elif year == '19':
			with open(report_path, "w") as fout:
				subprocess.run([f"./ispd{year}eval.sh",
				"-lef", f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.input.lef", 
				"-guide", f"{subdir}/ispd{year}_test{caseNum}.input.guide",
				"-idef", f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.input.def",
				"-odef", f"{subdir}/ispd{year}_test{caseNum}.output.def"], stdout=fout)
		else:
			assert(False)


# os.chdir(f"../ispd{year}eval")
# if year == '18':
#     subprocess.run([f"./ispd{year}eval.sh",
#     "-lef", f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.input.lef", 
#     "-guide", f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.input.guide",
#     "-def", f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.output.def"])
# else:
#     subprocess.run([f"./ispd{year}eval.sh",
#     "-lef", f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.input.lef", 
#     "-guide", f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.input.guide",
#     "-idef", f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.input.def",
#     "-odef", f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.output.def"])
# os.chdir(f"../bin")
