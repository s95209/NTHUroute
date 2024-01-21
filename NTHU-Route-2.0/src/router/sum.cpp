#include <iostream>
#include <vector>
#include <thread>
#include <cmath>
#include <omp.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

/*
void parallel_sum(std::vector<int>& arr, int start, int end) {
    for (int step = 1; step < end - start + 1; step *= 2) {
        for (int i = start; i + step < end; i += step * 2) {
            arr[i] += arr[i + step];
        }
    }
}

int sum_std(const vector<int> &input) {
    std::vector<int> arr = input;
    int num_threads = 2;
    int chunk_size = arr.size() / num_threads;

    std::vector<std::thread> threads;
    for (int i = 0; i < num_threads; ++i) {
        int start = i * chunk_size;
        int end = (i + 1) * chunk_size;
        threads.emplace_back(parallel_sum, std::ref(arr), start, end);
    }

    for (auto& t : threads) {
        t.join();
    }

    int total_sum = arr[0] + arr[chunk_size];
    std::cout << "std Sum of the array: " << total_sum << std::endl;
}


typedef struct {
    int* arr;
    int start;
    int end;
} SumArgs;

void* parallel_sum(void* args) {
    SumArgs* sum_args = (SumArgs*) args;
    int* arr = sum_args->arr;
    int start = sum_args->start;
    int end = sum_args->end;

    for (int step = 1; step < end - start + 1; step *= 2) {
        for (int i = start; i + step < end; i += step * 2) {
            arr[i] += arr[i + step];
        }
    }

    return NULL;
}

int sum_pthread(const vector<int> &input) {
    int arr[] = input;
    int num_threads = 2;
    int chunk_size = sizeof(arr) / sizeof(arr[0]) / num_threads;

    pthread_t threads[num_threads];
    SumArgs args[num_threads];

    for (int i = 0; i < num_threads; ++i) {
        args[i].arr = arr;
        args[i].start = i * chunk_size;
        args[i].end = (i + 1) * chunk_size;
        pthread_create(&threads[i], NULL, parallel_sum, (void*)&args[i]);
    }

    for (int i = 0; i < num_threads; ++i) {
        pthread_join(threads[i], NULL);
    }

    int total_sum = arr[0] + arr[chunk_size];
    printf("pthread Sum of the array: %d\n", total_sum);
}

int sum_omp(const vector<int> &input) {
    std::vector<int> arr = input;
    int num_threads = 2;

    #pragma omp parallel num_threads(num_threads)
    {
        int tid = omp_get_thread_num();
        int num_threads = omp_get_num_threads();
        int n = arr.size();
        int logn = static_cast<int>(ceil(log2(n)));
        for (int k = logn-1; k>=0; --k) {
            for (int s=0; s<static_cast<int>(ceil((1<<k)/num_threads)); ++s) {
                arr[tid + s*num_threads] = arr[2*(tid + s*num_threads)] + arr[2*(tid + s*num_threads) + 1];
            }
            #pragma omp barrier
        }
    }

    int total_sum = arr[0] + arr[chunk_size];
    std::cout << "omp Sum of the array: " << total_sum << std::endl;
}
*/

int parallel_sum(const std::vector<long long>& input) {
    // omp_set_num_threads(1);
    int n = input.size();
    int levels = static_cast<int>(std::ceil(std::log2(static_cast<double>(n))));
    int padded_size = static_cast<int>(std::pow(2, levels));
    std::vector<long long> padded_input(padded_size, 0);
    std::copy(input.begin(), input.end(), padded_input.begin());
    for (int d=0; d<levels; ++d) {
        int stride = 1 << (d+1);
        #pragma omp parallel for
        for (int i=0; i<padded_size; i+=stride)
            padded_input[i] += padded_input[i + (stride / 2)];
            // padded_input[i + stride - 1] = padded_input[i + (stride / 2) - 1] + padded_input[i + stride - 1];
    }
    return padded_input[0];
    // return padded_input[padded_size-1];
}

void parallel_prefix_sum(long long *input, long long *output, long long n) {
    using namespace std::chrono;

    // omp_set_num_threads(128);

    long long levels = static_cast<long long>(std::ceil(std::log2(static_cast<double>(n))));
    std::cout << "n = " << n << ", levels = " << levels << '\n';
    long long padded_size = static_cast<long long>(std::pow(2, levels));
    std::cout << "padded_size = " << padded_size << '\n';

    // std::vector<long long> padded_input(padded_size);
    long long *padded_input;
    try {
        padded_input = new long long[padded_size];
    } catch (std::bad_alloc&) {
        std::cout << "No memory\n";
        exit(0);
    }
    // std::copy(input.begin(), input.end(), padded_input.begin());


    auto cal_start = std::chrono::high_resolution_clock::now();

    long long d = 0, stride;
    #pragma omp parallel
    {
        #pragma omp for schedule(static)
        for (long long i=0; i<n; ++i) {
            padded_input[i] = input[i];
        }

        do {
            #pragma omp master
            {
                stride = 1 << (d+1);
            }
            #pragma omp barrier

            #pragma omp for schedule(static) firstprivate(stride)
            for (long long i = 0; i < padded_size; i += stride) {
                padded_input[i + stride - 1] = padded_input[i + (stride / 2) - 1] + padded_input[i + stride - 1];
            }

            #pragma omp master
            {
                ++d;
            }
            #pragma omp barrier
        } while (d < levels);
        #pragma omp barrier

        #pragma omp master
        {
            padded_input[padded_size - 1] = 0;
            d = levels - 1;
        }
        #pragma omp barrier

        do {
            #pragma omp master
            {
                stride = 1 << (d+1);
            }
            #pragma omp barrier

            #pragma omp for schedule(static) firstprivate(stride)
            for (long long i = 0; i < padded_size; i += stride) {
                long long t = padded_input[i + (stride / 2) - 1];
                padded_input[i + (stride / 2) - 1] = padded_input[i + stride - 1];
                padded_input[i + stride - 1] = t + padded_input[i + (stride / 2) - 1];
            }

            #pragma omp master
            {
                --d;
            }
            #pragma omp barrier

        } while (d>=0);
        #pragma omp barrier
        #pragma omp for schedule(static)
        for (long long i=0; i<n; ++i) {
            output[i] = padded_input[i] + input[i];
        }
    }
    auto cal_end = std::chrono::high_resolution_clock::now();


    // Up-sweep phase (reduce)
    // for (int d = 0; d < levels; ++d) {
    //     int stride = 1 << (d + 1);
    //     #pragma omp parallel for
    //     for (int i = 0; i < padded_size; i += stride) {
    //         padded_input[i + stride - 1] = padded_input[i + (stride / 2) - 1] + padded_input[i + stride - 1];
    //     }
    // }

    // padded_input[padded_size - 1] = 0;

    // // Down-sweep phase (scan)
    // for (int d = levels - 1; d >= 0; --d) {
    //     int stride = 1 << (d + 1);
    //     #pragma omp parallel for
    //     for (int i = 0; i < padded_size; i += stride) {
    //         long long t = padded_input[i + (stride / 2) - 1];
    //         padded_input[i + (stride / 2) - 1] = padded_input[i + stride - 1];
    //         padded_input[i + stride - 1] = t + padded_input[i + (stride / 2) - 1];
    //     }
    // }
    // auto cal_end = std::chrono::high_resolution_clock::now();

    // // std::copy(padded_input.begin(), padded_input.begin() + n, output.begin());
    // #pragma omp parallel for
    // for (int i=0; i<n; ++i) {
    //     output[i] = padded_input[i];
    // }

    duration<double> time_span3 = duration_cast<duration<double>>(cal_end - cal_start);
    std::cerr << std::endl <<"Calculating time taken: " << time_span3.count() << std::endl;
}

void parallel_prefix_sum_no_copy(std::vector<long long>& arr)
{
    int n = arr.size();
    int levels = static_cast<int>(std::ceil(std::log2(static_cast<double>(n))));

}

// int main() {
//     int n = 1000000; // size of the array
//     std::vector<int> array(n);

//     // Initialize the array with values from 1 to n
//     for (int i = 0; i < n; i++) {
//         array[i] = i + 1;
//     }

//     std::vector<int> prefix_sum(n, 0);
//     parallel_prefix_sum(array, prefix_sum);

//     long long sum = prefix_sum[n - 1] + array[n - 1];

//     std::cout << "Sum: " << sum << std::endl;

//     return 0;
// }

