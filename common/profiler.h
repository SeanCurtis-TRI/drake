#pragma once

#include <chrono>
#include <ratio>
#include <string>

#include "drake/common/type_safe_index.h"

namespace drake {
namespace common {

/** Index used to identify a registered timer.  */
using TimerIndex = TypeSafeIndex<class TimerTag>;

// TODO(SeanCurtis-TRI): Need to figure out how to prevent calling timer
// functions *before* calling start.

/**  Basic timer. A timer that can start and report the amount of time elapsed
 since calling start().

   times    t₀         t₁      t₂       t₃         t₄            t₅
          ┄┄┾━━━━━━━━━━┿━━━━━━━┿━━━━━━━━┿━━━━━━━━━━┿━━━━━━━━━━━━━┽┄┄┄┄
   f()      s()        e()     e()      e()        e()           e()

 The timeline above shows a series of %Timer method invocations: `s()` and
 `e()`, representing start() and elapsed(), respectively. At each call of
 elapsed, the duration elapsed between that time and the time that start() was
 called is returned (i.e., `tᵢ - t₀`, for `i > 0`).  */
class Timer {
 public:
  Timer() = default;

  /**  Starts the timer running.  */
  void start();

  /**  Reports the time elapsed between this call and the last invocation of
   start() in the units specified.
   @tparam Units  the units in which the elapsed time will be reported.  */
  template <typename Units>
  double elapsed() const {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, Units>(end - start_).count();
  }

 protected:
  /**  The time (in clock cycles) at which the timer was started.  */
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

///////////////////////////////////////////////////////////////////////////

/**  Lap timer. A timer which supports the idea of "laps", a sequence of timer
 milestones in an otherwise uninterrupted window of time.

   times    t₀         t₁      t₂       t₃         t₄            t₅     t₆
          ┄┄┾━━━━━━━━━━┿━━━━━━━┿━━━━━━━━┿━━━━━━━━━━┿━━━━━━━━━━━━━┽┄┄┄┄┄┄┤
   f()      s()        l()     l()      l()        l()           l()    a()
   laps           1         2        3         4            5

 The timeline above shows a series of %LapTimer method invocations: `s()`,
 `l()`, and `a()`, representing start(), lap(), and average(), respectively.
 At each call of lap(), the duration elapsed between that call and the
 previous invocation is reported, but the clock never stops. Finally, the
 call to average() reports to the total time spanned by the invocation to
 start() and the _last_ call to lap() and divides it by the
 number of calls to lap() (i.e., `(t₅ - t₀) / 5`).

 This class does not store the times for the individual laps; if that
 granularity of information is needed, the caller should restore the returned
 lap times.

 The lap timer can also be used to measure the average duration of intervals by
 alternating calls to start() and lap().

   times    t₀         t₁      t₂       t₃         t₄            t₅     t₆
          ┄┄┾━━━━━━━━━━┽┄┄┄┄┄┄┄┾━━━━━━━━┽┄┄┄┄┄┄┄┄┄┄┾━━━━━━━━━━━━━┽┄┄┄┄┄┄┤
   f()      s()        l()     s()      l()        s()           l()    a()
   laps           1                  2                      3

 The timeline above shows the measurement of three, non-contiguous intervals,
 each bound by a pair of calls to start() and lap(). In this case, average()
 will report the average interval length as `(t₅ - t₄ + t₃ - t₂ + t₁ + t₀) / 3`.
 */
class LapTimer : public Timer {
 public:
  using Units = std::chrono::nanoseconds::period;

  LapTimer() = default;

  /**  Reports the time elapsed from the previous call to lap() or start() to
   this call.
   @tparam Units  the units in which the elapsed time will be reported.  */
  template <typename Units>
  double lap()  {
    auto now = std::chrono::high_resolution_clock::now();
    auto delta = now - start_;
    start_ = now;
    total_ += std::chrono::nanoseconds(delta);
    ++lapCount_;
    return std::chrono::duration<double, Units>(delta).count();
  }

  /**  Reports the average lap time across all recorded laps.
   @tparam Units  the units in which the elapsed time will be reported.  */
  template <typename Units>
  double average() const {
    const double total_time =
        std::chrono::duration_cast<std::chrono::duration<double, Units>>(total_)
            .count();
    return total_time / lapCount_;
  }

  /**  Reports the total time measured.
   @tparam Units  the units in which the elapsed time will be reported.  */
  template <typename Units>
  double total() const {
    return std::chrono::duration_cast<std::chrono::duration<double, Units>>(
               total_)
        .count();
  }

  /**  Reports the number of calls to laps.  */
  int laps() const { return lapCount_; }

  /** Rest the timer. */
  void reset() {
    start_ = std::chrono::high_resolution_clock::now();
    total_ = std::chrono::nanoseconds(0);
    lapCount_ = 0;
  }

 protected:
  //  The total accrued time of timed intervals (in nanoseconds).
  std::chrono::nanoseconds total_{0};

  //  The total number of calls to lap().
  int lapCount_{0};
};

}  // namespace common
}  // namespace drake

#ifdef ENABLE_TIMERS

/**  Creates a lap timer which uses the given label for display.

 @param  displayString  The string to display when reporting the profiling
                        results.
 @returns  The identifier for the created timer.  */
drake::common::TimerIndex addTimer(std::string displayString);

/**  Starts the timer with the given identifier.

 @param  index  The timer identifier supplied by addTimer.  */
void startTimer(drake::common::TimerIndex index);

/**  Stops the timer with the given identifier.

 @param  index  The timer identifier supplied by addTimer.  */
void stopTimer(drake::common::TimerIndex index);

/**  Lap the ith timer.

 @param  index  The timer identifier supplied by addTimer.  */
void lapTimer(drake::common::TimerIndex index);

/**  Reports the average time of the ith timer in seconds.

 @param  index  The timer identifier supplied by addTimer.
 @returns  The average time of all laps.  */
double averageTimeInSec(drake::common::TimerIndex index);

/** Resets all timers accumulated time; the timer remains extant. */
void reset();

/** Clears all registered timers.  */
void clearAllTimers();

/**  Creates a string representing a table of all of the averages.  */
std::string TableOfAverages();

#else  // not defined ENABLE_TIMERS

#define addTimer(displayString) (common::TimerIndex(0))
#define startTimer(index) ((void)index)
#define stopTimer(index) ((void)index)
#define lapTimer(index) ((void)index)
#define averageTimeInSec(index) ((void)index)
#define reset()
#define clearAllTimers()
#define TableOfAverages() ("Profiling turned off")

#endif  // ENABLE_TIMERS
