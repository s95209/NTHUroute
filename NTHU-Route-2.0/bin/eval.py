#!/users/student/mr109/ykfang20/anaconda3/bin/python

import subprocess
import argparse
import os

# parser = argparse.ArgumentParser()

# parser.add_argument("testcase", type=str)

# args = parser.parse_args()

# testcase = args.testcase

# year, caseNum = testcase.split('_')

# print(f"Running test case ispd{year}_test{caseNum}...")

# if caseNum[-1] == 'm':
#     caseNum = caseNum[:-1] + '_metal5'

testcase_path = "/users/student/mr109//ykfang20/COLA_normal/testcase"

#d = f"/users/student/mr109/ykfang20/NTHU-Route_w_TritonRoute/bin/ispd{year}_test{caseNum}/"
#d = f"/users/student/mr109/ykfang20/05_15_paper/bin/ispd{year}_test{caseNum}/"
testcases = ['18_5', '18_8', '18_10', '19_7', '19_8', '19_9', '18_5m', '18_8m', '18_10m', '19_7m', '19_8m', '19_9m']
for testcase in testcases:
	print(f"Running {testcase}")
	year, caseNum = testcase.split('_')
	if caseNum[-1] == 'm':
		caseNum = caseNum[:-1] + '_metal5'
	
	d = f"/users/student/mr109/ykfang20/iccad2023_paper_results/ispd{year}_test{caseNum}"
	report_path = os.path.join(d, "report_exp.txt")
	if not os.path.exists(report_path):
		os.chdir(f"/users/student/mr109//ykfang20/COLA_normal/ispd{year}eval")
		if not os.path.exists(f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.input.lef"): continue
		if not os.path.exists(f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.input.def"): continue
		if not os.path.exists(f"{d}/ispd{year}_test{caseNum}.input.guide"): continue
		if not os.path.exists(f"{d}/ispd{year}_test{caseNum}.exp.output.def"): continue
		if year == '18':
			with open(report_path, "w") as fout:
				subprocess.run([f"./ispd{year}eval.sh",
				"-lef", f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.input.lef", 
				"-guide", f"{d}/ispd{year}_test{caseNum}.input.guide",
				"-def", f"{d}/ispd{year}_test{caseNum}.exp.output.def"], stdout=fout)
		elif year == '19':
			with open(report_path, "w") as fout:
				subprocess.run([f"./ispd{year}eval.sh",
				"-lef", f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.input.lef", 
				"-guide", f"{d}/ispd{year}_test{caseNum}.input.guide",
				"-idef", f"{testcase_path}/ispd{year}_test{caseNum}/ispd{year}_test{caseNum}.input.def",
				"-odef", f"{d}/ispd{year}_test{caseNum}.exp.output.def"], stdout=fout)
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
