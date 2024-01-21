#!/users/student/mr109/ykfang20/anaconda3/bin/python

import subprocess

#testcases = [ '18_10', ]
# testcases = ['18_8','18_10','19_8','19_9','18_8m','18_10m','19_8m']
# testcases = ['19_7m','19_8m','19_9m']
# testcases = ['18_5', '18_8', '18_10', '19_7', '19_8', '19_9']
#testcases = ['18_8m', '19_7m']
#testcases = ['19_7m', '19_9m']
testcases = ['18_5', '18_8', '18_10', '19_7', '19_8', '19_9', '18_5m', '18_8m', '18_10m', '19_7m', '19_8m', '19_9m']
#testcases = ['18_8m', '18_10m', '19_7m', '19_8m', '19_9m']
#testcases = ['19_7', '19_9', '19_7m', '19_8m', '19_9m']
#testcases = ['18_10m', '19_7m', '19_8m', '19_9m']
#testcases = ['18_5', '18_8', '18_10', '19_7', '19_8', '19_9']
#testcases = ['19_8m', '19_9m']
for testcase in testcases:
    print(f"Running {testcase}...")
    subprocess.call(['./eval_tune.py', testcase])
