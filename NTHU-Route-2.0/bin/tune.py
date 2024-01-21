#!/users/student/mr109/ykfang20/anaconda3/bin/python
import subprocess
import argparse

rcong_min = [0.7, 0.8, 0.9]
#rcong_min = [0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
rcong_diff = [0.1, 0.2, 0.3]
#rcong_diff = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
rcongV_min = [0.5, 0.6]
rcongV_diff = [0.1, 0.2]
rcongH_min = [0.5, 0.6]
rcongH_diff = [0.1, 0.2]
rcongNonZeroV_min = [0.5]
rcongNonZeroV_diff = [0.3]
rcongNonZeroH_min = [0.5]
rcongNonZeroH_diff = [0.3]
#init_reduct = [0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
init_reduct = [0.7, 0.8, 0.9]

parser = argparse.ArgumentParser()
parser.add_argument("testcase", type=str)
args = parser.parse_args()

testcase = args.testcase
# if testcase == '18_8':
#     rcong_2d_min = [0.9]
#     rcong_2d_diff = [0.1]
# elif testcase == '18_8m':
#     rcong_2d_min = [0.9]
#     rcong_2d_diff = [0.1]
# elif testcase == '18_10':
#     rcong_2d_min = [0.7]
#     rcong_2d_diff = [0.3]
# elif testcase == '18_10m':
#     rcong_2d_min = [0.9]
#     rcong_2d_diff = [0.1]
# elif testcase == '19_8':
#     rcong_2d_min = [0.7]
#     rcong_2d_diff = [0.3]
# elif testcase == '19_8m':
#     rcong_2d_min = [0.5]
#     rcong_2d_diff = [0.5]

for rcong_min_ in rcong_min:
    for rcong_diff_ in rcong_diff:
        for rcongV_min_ in rcongV_min:
            for rcongV_diff_ in rcongV_diff:
                for rcongH_min_ in rcongH_min:
                    for rcongH_diff_ in rcongH_diff:
                        for rcongNonZeroV_min_ in rcongNonZeroV_min:
                            for rcongNonZeroV_diff_ in rcongNonZeroV_diff:
                                for rcongNonZeroH_min_ in rcongNonZeroH_min:
                                    for rcongNonZeroH_diff_ in rcongNonZeroH_diff:
                                        for init_reduct_ in init_reduct:
                                            if rcong_min_+rcong_diff_<=1.0 and rcongV_min_+rcongV_diff_<=1.0\
                                                and rcongH_min_+rcongH_diff_<=1.0 and rcongNonZeroV_min_+rcongNonZeroV_diff_<=1.0\
                                                and rcongNonZeroH_min_+rcongNonZeroH_diff_<=1.0:
                                                                                        
                                                with open(f"./param_{testcase}.txt", 'w') as f:
                                                    f.write(f"rcong_min:{rcong_min_}" + '\n')
                                                    f.write(f"rcong_diff:{rcong_diff_}" + '\n')
                                                    f.write(f"rcongV_min:{rcongV_min_}" + '\n')
                                                    f.write(f"rcongV_diff:{rcongV_diff_}" + '\n')
                                                    f.write(f"rcongH_min:{rcongH_min_}" + '\n')
                                                    f.write(f"rcongH_diff:{rcongH_diff_}" + '\n')
                                                    f.write(f"rcongNonZeroV_min:{rcongNonZeroV_min_}" + '\n')
                                                    f.write(f"rcongNonZeroV_diff:{rcongNonZeroV_diff_}" + '\n')
                                                    f.write(f"rcongNonZeroH_min:{rcongNonZeroH_min_}" + '\n')
                                                    f.write(f"rcongNonZeroH_diff:{rcongNonZeroH_diff_}" + '\n')
                                                    f.write(f"init_reduct:{init_reduct_}" + '\n')
                                                subprocess.run(['./run_tune.py', f"{args.testcase}"])

