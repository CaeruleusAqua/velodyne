#include "Utils.h"
namespace utils {

    /**
    * This is a versatile function to create lists containing arithmetic progressions.
    * It is most often used in for loops
    *
    * @param start The starting value of the sequence.
    * @param stop The end value of the sequence, unless endpoint is set to False. In that case, the sequence
    * consists of all but the last of num + 1 evenly spaced samples, so that stop is excluded.
    * @param step Stepsize used to generate the samples. Default is 1. Can also be negative.
    * @param endpoint If True, stop is the last sample. Otherwise, it is not included. Default is False.
    */
    std::vector<double> range(double start, double stop, double step, bool endpoint) {
        if (start > stop && step > 0) {
            step = -step;
        }
        std::vector<double> array;
        uint64_t num = 0;
        if (endpoint) {
            num = abs((stop - start) / step) + 1;
        } else {
            num = abs((stop - start - step) / step) + 1;
        }
        std::cout << "Num: " << num << std::endl;

        array.reserve(num);
        if (step >= 0) {
            for (uint64_t i = 0; i < num; i++) {
                array.push_back(start);
                start += step;
            }
        } else {
            for (uint64_t i = 0; i < num; i++) {
                array.push_back(start);
                start += step;
            }
        }
        return array;
    }

    /**
    * Return evenly spaced numbers over a specified interval.
    *
    * @param start The starting value of the sequence.
    * @param stop The end value of the sequence, unless endpoint is set to False. In that case, the sequence
    * consists of all but the last of num + 1 evenly spaced samples, so that stop is excluded.
    * Note that the step size changes when endpoint is False.
    * @param num Number of samples to generate. Default is 50. Must be non-negative.
    * @param endpoint If True, stop is the last sample. Otherwise, it is not included. Default is True.
    */
    std::vector<double> linspace(double start, double stop, uint64_t num, bool endpoint) {
        std::vector<double> array;
        double step = 0;
        if (endpoint) {
            if (num == 1) {
                array.push_back(start);
                return array;
            }
            step = (stop - start) / (num - 1);
        } else {
            if (num == 0) {
                return array;
            }
            step = (stop - start) / (num);
        }
        array.reserve(num);
        if (start == stop) {
            for (uint64_t i = 0; i < num; i++) {
                array.push_back(start);
            }
        } else if (step >= 0) {
            for (uint64_t i = 0; i < num; i++) {
                array.push_back(start);
                start += step;
            }
        } else {
            for (uint64_t i = 0; i < num; i++) {
                array.push_back(start);
                start += step;
            }
        }
        return array;
    }
}


