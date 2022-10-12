#ifndef  MY_LOOP_TIMER_H
#define  MY_LOOP_TIMER_H

#include <vector>
#include <chrono>
#include <iostream>
#include <iomanip>

namespace my {

/**
 * @brief LoopTimer is a timing tool optimized for high frequency loops (e.g., millions of iterations in a short time),
 * where any redundant computations or I/O operations can impact timing accuracy. Therefore, they should be avoided and
 * saved to be executed after the loop. This class is implemented based on chrono::steady_clock, which requires a
 * minimum execution time (e.g., 0.068 us) to place a time stamp. By placing multiple "stamps" in the loop/function,
 * LoopTimer can compute the average running time in between every two consecutive stamps, while minimizing additional
 * time consumption caused by the timing operation itself.
 * 
 * Example code:
 * 
 * @code
 * function_to_analyze() {
 *   my::LoopTimer timer(10);  // plan to place at most 10 stamps; this also starts the timer
 *   ...
 *   timer.stamp(0);
 *   for (int i = 0; i < 100000; ++i) {
 *     timer.stamp(1);  // place here to serve as a timing startpoint; in this case, 0-1 duration is not important
 *     ...
 *     timer.stamp(2);  // ends 1-2 duration and starts 2-3 duration
 *     ...
 *     timer.stamp(3);
 *     ...
 *   }
 *   timer.stamp(4);  // we end up using 5 stamps
 *   timer.printTimeMilliseconds();  // compute and print the average running time between every two stamps
 * }
 * @endcode
 * 
 * @author Hanzhe Teng
 * @date Aug 10, 2021
 */
class LoopTimer {
 public:
  /**
   * @brief Constructor. Initialize the number of stamps, such that memory can be allocated accordingly.
   * It will also start a new time duration automatically.
   * @param num_stamp the total number of stamps to be used
   */
  LoopTimer (int num_stamp)
      : num_stamp_ (num_stamp),
        counter_(num_stamp),
        sum_time_(num_stamp) {
    tic_ = std::chrono::steady_clock::now();
  }

  /**
   * @brief Place a time stamp to end the previous duration (toc) and start a new duration (tic).
   * @param idx the index of time stamp; stamps with the same index will be grossed and averaged.
   * @note No sanity check on index for the sake of performance.
   * This function requires minimal execution time, which is only 0.068 us for example.
   */
  inline void stamp(int idx = 0) {
    toc_ = std::chrono::steady_clock::now();
    counter_[idx] += 1;
    sum_time_[idx] += std::chrono::duration_cast<std::chrono::nanoseconds>(toc_ - tic_);
    tic_ = toc_;
  }

  /**
   * @brief Print the average running time in each duration and how many times each duration has been recorded.
   * @note The unit of time is second (s). This function should be called only once, after the loop.
   */
  void printTimeSeconds() const {
    for (int idx = 0; idx < num_stamp_; ++idx) {
      if (!counter_[idx]) continue;
      double time = sum_time_[idx].count() / 1000000000.0 / counter_[idx];
      std::cout << "[LoopTimer] stamp" << std::setw(3) << idx << ":  " << std::setw(7)
        << std::fixed << std::setprecision(3) << time << " s with counter = " << counter_[idx] << "\n";
    }
  }
  
  /**
   * @brief Print the average running time in each duration and how many times each duration has been recorded.
   * @note The unit of time is millisecond (ms), where 1s = 1000 ms.
   * This function should be called only once, after the loop.
   */
  void printTimeMilliseconds() const {
    for (int idx = 0; idx < num_stamp_; ++idx) {
      if (!counter_[idx]) continue;
      double time = sum_time_[idx].count() / 1000000.0 / counter_[idx];
      std::cout << "[LoopTimer] stamp" << std::setw(3) << idx << ":  " << std::setw(7)
        << std::fixed << std::setprecision(3) << time << " ms with counter = " << counter_[idx] << "\n";
    }
  }

  /**
   * @brief Print the average running time in each duration and how many times each duration has been recorded.
   * @note The unit of time is microsecond (us), where 1s = 1000000 us.
   * This function should be called only once, after the loop.
   */
  void printTimeMicroseconds() const {
    for (int idx = 0; idx < num_stamp_; ++idx) {
      if (!counter_[idx]) continue;
      double time = sum_time_[idx].count() / 1000.0 / counter_[idx];
      std::cout << "[LoopTimer] stamp" << std::setw(3) << idx << ":  " << std::setw(7)
        << std::fixed << std::setprecision(3) << time << " us with counter = " << counter_[idx] << "\n";
    }
  }

 private:
  int num_stamp_;  ///< The number of time stamps to be placed in the loop/function.

  std::chrono::steady_clock::time_point tic_;  ///< The startpoint of a time duration.

  std::chrono::steady_clock::time_point toc_;  ///< The endpoint of a time duration.

  std::vector<int> counter_;  ///< Container to keep how many times a designated time duration has repeated.

  /**
   * @brief Container to keep the sum of running time for each designated time duration.
   * @note chrono::nanoseconds is using signed long long int to store time;
   * 9,223,372,036,854,775,807 nanoseconds is equivalent to 106,752 days;
   * it will work fine as long as the total running time does not exceed this limit :)
   */
  std::vector<std::chrono::nanoseconds> sum_time_;
};

}  // namespace my

#endif  // MY_LOOP_TIMER_H
