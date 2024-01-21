# NTHU Route 2.0: ISPD24 version

<br>

## A. Folder Architecture

The architecture of the folder:

- benchmarks
    - nangate45
        - def
        - example_output
        - ...
- environment
- NTHU-Route-2.0
- script
- ...

<br>

## B. To set up the environment

- Install the boost 1.77.0 (.gz file) in the `~/` folder ([link](https://sourceforge.net/projects/boost/files/boost/1.77.0/boost_1_77_0.tar.gz/download)).
- Go to `./environment` folder.
- Run the following code:

```shell=
>> ./env_set.sh
``` 
<br>

## C. makefile

- Go to `./NTHU-Route-2.0/src`.
- Edit the *boost dir* in the makefile.
- Run the following code:

```shell=
>> make clean
>> make -j
``` 

<br>

## D. To run NTHU Route 2.0

- Go to `./NTHU-Route-2.0/bin`.
- Run the following code:

```shell=
>> ./route.sh <suffix>
``` 
- The suffix is optional, only for file name recongnition.
- e.g. `./route.sh test`
    
    - output file name: `ariane51_test.PR_out`

<br>

## E. To evaluate the GR result

- Go to `./script`.
- Run the following code:

```shell=
>> ./evaluation.sh <suffix>
``` 

- The suffix is optional, only for file name recongnition.
- e.g. `./evaluation.sh test`
    
    - output file name: `ariane51_test.log`

<br>

## F. To build the docker environment

###  1. To install docker (register at [docker hub](https://hub.docker.com/))

- ```sudo apt install docker.io```
- ```sudo docker login```

### 2. To get the NGC CLI (NVIDIA GPU CLOUD Command Line Interface) ([tutorial](https://docs.nvidia.com/ai-enterprise/deployment-guide-spark-rapids-accelerator/0.1.0/appendix-ngc.html))

- Register an [NCG account](https://ngc.nvidia.com/signin).
- Follow the instruction to create an API key.
- ```sudo docker login nvcr.io```

    - Username: $oauthtoken
    - Password: *[Your API key]*

### 3. Use dockfile to setup the environment

- ```sudo docker build -t [Author]/[Image name] [Dockerfile path]```
- ```sudo docker build -t pacificfeng/ispd2024_gpu_env .```

### 4. Check if build the image successfully

- ```docker images```

### 5. To list the container information

- ```sudo docker ps [-a (optional)]```
- use `-a` will also list the stopped image.

### 6. To run the container
- ```sudo docker run -it [Author]/[Image name]```

- ```sudo docker run -it pacificfeng/ispd2024_gpu_env```

- `Author` is optional.

### 7. To exit the container

- ```exit```

### 8. To stop the container

- ```sudo docker stop [Container ID]```

### 9. To delete the container

- ```sudo docker rm [Container ID]```

<br>

## G. TODO

1. To close the capacity reduction
1. To add the find group funtion
1. Implement the #GAMER flag
1. Implement the #GAMER on GPU