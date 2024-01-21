# A Fast and Robust Detailed-Routability-Driven Global Router

## Overview
- Code 有兩個版本
    1. ASP-DAC 2024 版
        - 與投稿 ASP-DAC 2024 論文 consistent
    2. 論文版
        - 與論文 consistent
        - 將 Thesis/src/router/MM_mazeroute.cpp 中的 compiler flag ```#GAMER``` 打開可得 *Parallelized* 版本，註解掉可得 *Sequential* 版本 (與 ICCAD 2023 投稿論文 consistent)

- 目前的 code 會去 TritonRoute-WXL 的資料夾下的 testcaes 資料夾讀進 initial congestion map ```cmap.txt``` 和 pin accsee information ```netlist.txt```
- GR 或 DR 的 input, output 都放在 ```testcases```資料夾

## Experimental Machines
- Run on ic56 or ic53
- Evaluate on ic21, ic22, ic51 (To use Innovus)

## Build Tools
- To build our global router
    - GCC >= 9.3.0
- To build TritonRoute-WXL
    - CMake ver. >= ? (I built it on ic51)
    - GCC >= 9.3.0

## Pre-requisite Library
- Boost >= 1.77.0

## How to Build
GR:
兩種版本 build 法一樣，以 Thesis 版舉例
- In Thesis/src/:
    ```
    make clean
    make -j
    ```
- 執行檔 ```route``` 生出來會在 Thesis/bin/

DR(TritonRoute-WXL):
可參考 TritonRoute-WXL Github: https://github.com/ABKGroup/TritonRoute-WXL

## How to Run
To generate global routing results:
- In Thesis/bin/:
    ```
    ./route --input test --def path/to/input.def --lef path/to/input.lef --guide path/to/guide --param path/to/parameter_file
    ```
- Input 有兩個:
    - ispd1X_testX.input.def
    - ispd1X_testX.input.lef
- Output 有一個 (Global routing guide)
    - ispd1X_testX.input.guide
- ```--inptu test``` 可能可以不用打，但我沒改(後面解釋)

也可以使用 ```run.py```, ```run_tune.py```, and ```runAll.py``` 來跑，會比較方便。(需要 Python3)

To generate detailed routing results:
- In Thesis/TritonRoute-WXL/build/:
    ```
    ./run_tune.py $testcase_name
    ```
    - ```$testcase_name``` can be: 18_5, 18_8, 18_10, 19_7, 19_8, 19_9, 18_5m, 18_8m, 18_10m, 19_7m, 19_8m and 19_9m

## How to Evaluate
- 要在可以跑 Innovus 的機器上跑
- In Thesis/bin/:
    ```
    ./eval_tune.py $testcase_name
    ```
    - 可參考 ```eval_tune.py``` 內的寫法
    - 需要有 ICCAD 2019 比賽提供的 evaluation script. (路徑記得改)
    - http://iccad-contest.org/2019/problems.html
## ASP-DAC 2024 版本和 Thesis 版本 (*Sequential*) 差異
- 2D Capacity Reduction 用隔壁兩個 Gcells congestion 平均來打折 edge capacity.
- Maze routing 找到 sink 不會 break.
- Capacity Reduction 參數不一樣
---
## Future Works
### Major
- 自動化調參數 (e.g., ML), 目前每個 testcase 的參數寫在 ```bin/``` 資料夾下, 例如 $ispd18\_test5$ 的 parameter file 是 ```param_18_5.txt```
    - 不同參數對結果影響很大，可能可以調出更好的參數(手動(參考 bin/tune.py) or 自動(ML))
    - 目前每個 testcase 的每條 Gedge or Gcell 打折用一樣參數，可能可以不同
- GAMER 可以嘗試做在 GPU 上
    - 可參考 src/router/sum.cpp 裡面有 intrarow parallelism 實作(沒有採用)
- Layer assignment 的 DP update 可能可以換成 CUGR 2.0 的 complexity 較低的演算法
    - 但我實作出來沒有比較快，可能是我寫不好，可參考 Layerassignment.cpp 裡的 #DAC2023 compiler flag.
    - 跑出來結果非 *metal5* testcases 會不一樣，我不知道為什麼，可能是我實作錯，我已盡量讓它 behavior 和 COLA 一樣.
- 和 Innovus 比較，觀察 Innovus 是不是可以都做出無 DRC
- 加速其他 bottleneck

### Minor
- 跑 GR command 裡的 ```--input test``` 可能可以省略。原本的 layer assignment 會先寫一個 ```test.input``` file 再讀進來再轉後成最後的 guide 寫檔。我把這步簡化只寫一次檔，所以應該不用 ```--input test``` 的 option.
- 原本 GARY 用 CUGR 的 Rsyn 來 parse .lef 和 .def 檔案，但現在直接用 TritonRoute-WXL 的 inputs，所以有關 Rsyn 的 ```database```, ```grDatabase``` 應都不需要。但 code 還是會跑 Rsyn.
    - 目前 code 還是有用到 ```database``` 和 ```grDatabase``` 的東西。但都是一些常數，例如 layer direction, gCellStep 等等。
- 將 TritonRoute-WXL 的 input parsing 寫進 code, 而不是像現在這樣讀檔。
- 整理 code, 現在 code base 裡可能有滿多不會用到的東西。
- 可能可以考慮調 src/router/parameter.cpp 裡的參數，例如讓 maze routing 不要做這麽多 iteratinos 節省時間
- 可以考慮調 *Parallelized* 版本 (使用 GAMER 加速) 的 capacity reduction 參數，分數可能可以更好，且可以弄到沒有 DRC，我沒有調，目前用與投稿 ICCAD 2023 版本相同參數。



